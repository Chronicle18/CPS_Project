import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import sys
import os


# ============================================================
#  CONFIG
# ============================================================
TIME_STEP = 1.0 / 240.0
CUBE_MASS_RATIO = 0.25          # cube = 25% of car mass
CUBE_Y_LIMIT_RATIO = 0.80     
CUBE_X_LIMIT_RATIO = 0.70       

# PID Tuning for Pitch (Y-axis movement)
PID_PITCH_KP = 8.0
PID_PITCH_KI = 0.1
PID_PITCH_KD = 1.2

# PID Tuning for Roll (X-axis movement)
PID_ROLL_KP = 6.0
PID_ROLL_KI = 0.05
PID_ROLL_KD = 0.8

TARGET_PITCH = 0.0              # keep car level during flight
TARGET_ROLL = 0.0               # keep car level during flight
RAMP_ANGLE = 25 * math.pi / 180 # degrees → radians

# ============================================================
#  SPEED PROFILES
# ============================================================
SPEED_PROFILES = {
    'SLOW': {
        'target_velocity': 40,
        'max_force': 100,
        'description': 'Slow and steady'
    },
    'NORMAL': {
        'target_velocity': 80,
        'max_force': 180,
        'description': 'Standard speed'
    },
    'FAST': {
        'target_velocity': 100,
        'max_force': 180,
        'description': 'High speed'
    },
    'VERY_FAST': {
        'target_velocity': 160,
        'max_force': 180,
        'description': 'Maximum speed'
    }
}

# Select speed mode here
SELECTED_SPEED = 'FAST'  # Change to: SLOW, NORMAL, FAST, or VERY_FAST

# Success/Failure Criteria
MAX_ROLL_ANGLE = 45 * math.pi / 180     # 45 degrees
MAX_PITCH_ANGLE = 60 * math.pi / 180    # 60 degrees
MIN_LANDING_HEIGHT = 0.15                # below this = landed
LANDING_VELOCITY_THRESHOLD = 0.5         # velocity must be low
STABLE_DURATION = 0.5                    # seconds to confirm stable landing


# ============================================================
#  UTILITIES
# ============================================================
def world_to_local(body, world_xyz):
    """Convert WORLD → LOCAL coordinates for an arbitrary point."""
    pos, orn = p.getBasePositionAndOrientation(body)
    inv_pos, inv_orn = p.invertTransform(pos, orn)
    local_xyz, _ = p.multiplyTransforms(inv_pos, inv_orn, world_xyz, [0,0,0,1])
    return local_xyz


def local_to_world(body, local_xyz):
    """Convert LOCAL → WORLD coordinates."""
    pos, orn = p.getBasePositionAndOrientation(body)
    world_xyz, _ = p.multiplyTransforms(pos, orn, local_xyz, [0,0,0,1])
    return world_xyz


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


# ============================================================
#  PID CONTROLLER
# ============================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def step(self, error, dt=TIME_STEP):
        self.integral += error * dt
        # Anti-windup: clamp integral
        self.integral = max(-1.0, min(1.0, self.integral))
        
        deriv = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*deriv

    def reset(self):
        self.integral = 0
        self.prev_error = 0


# ============================================================
#  SUCCESS MONITOR
# ============================================================
class SuccessMonitor:
    def __init__(self):
        self.status = "DRIVING"  # DRIVING, AIRBORNE, LANDING, SUCCESS, FAILED
        self.failure_reason = None
        self.landing_start_time = None
        self.flight_start_time = None
        self.max_height = 0
        self.history = {
            'time': [],
            'height': [],
            'pitch': [],
            'roll': [],
            'velocity': [],
            'status': []
        }
    
    def update(self, car, current_time):
        """Update status and check for success/failure conditions"""
        car_pos, car_orn = p.getBasePositionAndOrientation(car)
        car_vel, car_ang_vel = p.getBaseVelocity(car)
        
        roll, pitch, yaw = p.getEulerFromQuaternion(car_orn)
        height = car_pos[2]
        velocity = math.sqrt(sum(v**2 for v in car_vel))
        
        # Normalize angles
        roll = normalize_angle(roll)
        pitch = normalize_angle(pitch)
        
        # Track max height
        self.max_height = max(self.max_height, height)
        
        # Log history
        self.history['time'].append(current_time)
        self.history['height'].append(height)
        self.history['pitch'].append(pitch)
        self.history['roll'].append(roll)
        self.history['velocity'].append(velocity)
        self.history['status'].append(self.status)
        
        # State machine
        if self.status == "DRIVING":
            if height > 0.4:  # Takeoff detected
                self.status = "AIRBORNE"
                self.flight_start_time = current_time
                
        elif self.status == "AIRBORNE":
            # Check for catastrophic mid-air failure
            if abs(roll) > MAX_ROLL_ANGLE:
                self.status = "FAILED"
                self.failure_reason = f"Excessive roll during flight: {math.degrees(roll):.1f}°"
                return
            
            if abs(pitch) > MAX_PITCH_ANGLE:
                self.status = "FAILED"
                self.failure_reason = f"Excessive pitch during flight: {math.degrees(pitch):.1f}°"
                return
            
            # Transition to landing
            if height < MIN_LANDING_HEIGHT + 0.1:
                self.status = "LANDING"
                self.landing_start_time = current_time
                
        elif self.status == "LANDING":
            # Check if car has stabilized
            time_on_ground = current_time - self.landing_start_time
            
            # Failure conditions after landing
            if abs(roll) > MAX_ROLL_ANGLE:
                self.status = "FAILED"
                self.failure_reason = f"Car flipped on landing: roll={math.degrees(roll):.1f}°"
                return
            
            if abs(pitch) > MAX_PITCH_ANGLE:
                self.status = "FAILED"
                self.failure_reason = f"Car flipped on landing: pitch={math.degrees(pitch):.1f}°"
                return
            
            # Success condition: stable for STABLE_DURATION
            if time_on_ground > STABLE_DURATION:
                if abs(roll) < 20*math.pi/180 and abs(pitch) < 20*math.pi/180:
                    self.status = "SUCCESS"
                    return
        
        return
    
    def get_report(self):
        """Generate final report"""
        report = f"\n{'='*60}\n"
        report += f"FINAL STATUS: {self.status}\n"
        report += f"{'='*60}\n"
        
        if self.failure_reason:
            report += f"Failure Reason: {self.failure_reason}\n"
        
        if len(self.history['time']) > 0:
            report += f"Max Height Reached: {self.max_height:.2f}m\n"
            
            if self.flight_start_time:
                flight_duration = self.history['time'][-1] - self.flight_start_time
                report += f"Flight Duration: {flight_duration:.2f}s\n"
            
            final_pitch = math.degrees(self.history['pitch'][-1])
            final_roll = math.degrees(self.history['roll'][-1])
            report += f"Final Orientation: Pitch={final_pitch:.1f}°, Roll={final_roll:.1f}°\n"
        
        report += f"{'='*60}\n"
        return report


# ============================================================
#  PYBULLET INITIALIZATION
# ============================================================
def init_sim():
    cid = p.connect(p.GUI)          # GUI mode ON
    p.resetDebugVisualizerCamera(
        cameraDistance=5,
        cameraYaw=30,
        cameraPitch=-20,
        cameraTargetPosition=[0, 0, 0.5]
    )
    p.setTimeStep(TIME_STEP)
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Add debug text
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    
    return cid


# ============================================================
#  LOAD WORLD
# ============================================================
def load_world():
    plane = p.loadURDF("plane.urdf")

    # ----- Ramp -----
    ramp_len = 2
    ramp_height = ramp_len * math.sin(RAMP_ANGLE)
    ramp_thickness = 0.1

    col_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[ramp_len/2, 1.0, ramp_thickness/2]
    )
    vis_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[ramp_len/2, 1.0, ramp_thickness/2],
        rgbaColor=[0.8, 0.2, 0.2, 1]
    )

    ramp_pos = [6, 0, 0.2]
    ramp_orn = p.getQuaternionFromEuler([0, -RAMP_ANGLE, 0])

    ramp = p.createMultiBody(
        baseCollisionShapeIndex=col_id,
        baseVisualShapeIndex=vis_id,
        basePosition=ramp_pos,
        baseOrientation=ramp_orn
    )

    return plane, ramp


# ============================================================
#  LOAD CAR + INTERNAL MASS
# ============================================================
def load_car_with_mass():
    car = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.2], globalScaling=2.0)

    for _ in range(10):
        p.stepSimulation()

    # Get car mass
    dyn = p.getDynamicsInfo(car, -1)
    car_mass = dyn[0]
    cube_mass = car_mass * CUBE_MASS_RATIO

    # Get car bounding box to determine size
    car_aabb = p.getAABB(car, -1)
    car_min, car_max = car_aabb
    car_length = car_max[0] - car_min[0]  # X dimension (front-back)
    car_width = car_max[1] - car_min[1]   # Y dimension (left-right)
    car_height = car_max[2] - car_min[2]  # Z dimension (up-down)

    cube_half_extent = 0.05
        # ----- internal cube (small but heavy) -----
    cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_half_extent]*3)
    cube_vis = p.createVisualShape(
        p.GEOM_BOX, 
        halfExtents=[cube_half_extent]*3,
        rgbaColor=[0, 1, 0, 1]
    )

    cube = p.createMultiBody(
        baseMass=cube_mass,
        baseCollisionShapeIndex=cube_col,
        baseVisualShapeIndex=cube_vis,
        basePosition=[0, 0, 0.2]  # Start centered above scaled car
    )

    print(f"Car mass: {car_mass:.2f}kg")
    print(f"Cube mass: {cube_mass:.2f}kg ({CUBE_MASS_RATIO*100:.0f}% of car)")
    print(f"Car dimensions (L×W×H): {car_length:.2f}m × {car_width:.2f}m × {car_height:.2f}m")
    print(f"Cube size: {cube_half_extent*2:.3f}m (5% of avg car dimension)")

    return car, cube, cube_mass, car_length, car_width, car_height


# ============================================================
#  MAIN SIM LOGIC
# ============================================================
def run_sim():
    # Get speed profile
    speed_config = SPEED_PROFILES[SELECTED_SPEED]
    FAST_TARGET_VEL = speed_config['target_velocity']
    MAX_WHEEL_FORCE = speed_config['max_force']
    
    print(f"\n{'='*60}")
    print(f"SPEED MODE: {SELECTED_SPEED}")
    print(f"Description: {speed_config['description']}")
    print(f"Target Velocity: {FAST_TARGET_VEL} | Max Force: {MAX_WHEEL_FORCE}")
    print(f"{'='*60}\n")
    
    # Initialize controllers
    pid_pitch = PID(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD)
    pid_roll = PID(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD)
    monitor = SuccessMonitor()

    init_sim()
    load_world()
    car, cube, cube_mass, car_length, car_width, car_height = load_car_with_mass()
    
    # Calculate cube movement limits based on car dimensions
    CUBE_Y_LIMIT = car_length * CUBE_Y_LIMIT_RATIO  # Along car length (forward/back)
    CUBE_X_LIMIT = car_width * CUBE_X_LIMIT_RATIO   # Along car width (left/right)
    
    print(f"Cube Y-axis limit: ±{CUBE_Y_LIMIT:.3f}m ({CUBE_Y_LIMIT_RATIO*100:.0f}% of car length)")
    print(f"Cube X-axis limit: ±{CUBE_X_LIMIT:.3f}m ({CUBE_X_LIMIT_RATIO*100:.0f}% of car width)")
    print(f"{'='*60}\n")

    # Wheels = joints 2 and 3 for rear drive
    wheel_joints = [2, 3]

    # Reduce damping so wheels spin freely
    for j in wheel_joints:
        p.changeDynamics(car, j, lateralFriction=1.2, 
                        spinningFriction=0.02, rollingFriction=0.0)

    # Initial cube position (local coords) - centered
    cube_local_x = 0.0
    cube_local_y = 0.0
    cube_local_z = 0.2  # Adjusted for 2x scaled car

    # ---------------------------------------------------------
    # START MP4 RECORDING
    # ---------------------------------------------------------

    original_stderr = sys.stderr
    sys.stderr = open(os.devnull, 'w')
    log_id = p.startStateLogging(
        p.STATE_LOGGING_VIDEO_MP4,
        "car_flip_controller_enhanced.mp4"
    )

    # Simulation loop
    MAX_STEPS = 1000
    start_time = time.time()
    
    for step in range(MAX_STEPS):
        current_time = step * TIME_STEP
        
        # ----------------------------------------------------------
        # DRIVE FORWARD
        # ----------------------------------------------------------
        for j in wheel_joints:
            p.setJointMotorControl2(
                bodyUniqueId=car,
                jointIndex=j,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=FAST_TARGET_VEL,
                force=MAX_WHEEL_FORCE
            )

        # ----------------------------------------------------------
        # CAR STATE
        # ----------------------------------------------------------
        car_pos, car_orn = p.getBasePositionAndOrientation(car)
        car_vel, car_ang_vel = p.getBaseVelocity(car)
        
        roll, pitch, yaw = p.getEulerFromQuaternion(car_orn)
        roll = normalize_angle(roll)
        pitch = normalize_angle(pitch)
        
        height = car_pos[2]
        velocity = math.sqrt(sum(v**2 for v in car_vel))
        
        # Estimate time to collision
        if height > MIN_LANDING_HEIGHT and car_vel[2] < 0:
            time_to_collision = math.sqrt(2 * (height - MIN_LANDING_HEIGHT) / 9.8)
        else:
            time_to_collision = 0

        # ----------------------------------------------------------
        # CONTINUOUS BALANCE CONTROL (2D: pitch and roll)
        # Active at all times - driving, airborne, and landing
        # ----------------------------------------------------------
        # Pitch control (Y-axis cube movement)
        pitch_error = TARGET_PITCH - pitch
        cube_shift_y = pid_pitch.step(pitch_error)
        cube_shift_y = max(-CUBE_Y_LIMIT, min(CUBE_Y_LIMIT, cube_shift_y))
        
        # Roll control (X-axis cube movement)
        roll_error = TARGET_ROLL - roll
        cube_shift_x = pid_roll.step(roll_error)
        cube_shift_x = max(-CUBE_X_LIMIT, min(CUBE_X_LIMIT, cube_shift_x))
        
        cube_local_x = cube_shift_x
        cube_local_y = cube_shift_y

        # ----------------------------------------------------------
        # UPDATE CUBE POSITION
        # ----------------------------------------------------------
        cube_local_pos = [cube_local_x, cube_local_y, cube_local_z]
        cube_world = local_to_world(car, cube_local_pos)
        p.resetBasePositionAndOrientation(cube, cube_world, car_orn)

        # ----------------------------------------------------------
        # UPDATE SUCCESS MONITOR
        # ----------------------------------------------------------
        monitor.update(car, current_time)
        
        # Display status in console
        # if step % 60 == 0:  # Every 0.25 seconds
        #     print(f"t={current_time:.2f}s | Status: {monitor.status:10s} | "
        #           f"Height: {height:.2f}m | Pitch: {math.degrees(pitch):6.1f}° | "
        #           f"Roll: {math.degrees(roll):6.1f}° | Cube: ({cube_local_x:.2f}, {cube_local_y:.2f})")

        # ----------------------------------------------------------
        # FOLLOW CAMERA
        # ----------------------------------------------------------
        p.resetDebugVisualizerCamera(
            cameraDistance=5,
            cameraYaw=30,
            cameraPitch=-20,
            cameraTargetPosition=car_pos
        )

        # Check for early termination
        if monitor.status in ["SUCCESS", "FAILED"]:
            print(f"\nSimulation ended at t={current_time:.2f}s")
            break

        # Step simulation
        p.stepSimulation()
        time.sleep(TIME_STEP)

    # ---------------------------------------------------------
    # STOP MP4 RECORDING
    # ---------------------------------------------------------
    p.stopStateLogging(log_id)

    # ---------------------------------------------------------
    # FINAL REPORT
    # ---------------------------------------------------------
    print(monitor.get_report())
    print(f"Speed Mode: {SELECTED_SPEED}")
    print(f"MP4 Saved as: car_flip_controller_enhanced.mp4\n")


# ============================================================
#  ENTRY
# ============================================================
if __name__ == "__main__":
    run_sim()
