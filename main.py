import pybullet as p
import pybullet_data
import time
import math
from tqdm import tqdm


# ============================================================
#  CONFIG
# ============================================================
TIME_STEP = 1.0 / 240.0
CUBE_MASS_RATIO = 0.25          # cube = 25% of car mass
CUBE_Y_LIMIT = 0.5              # meters forward/back allowed
PID_KP = 6.0
PID_KI = 0.0
PID_KD = 0.6
TARGET_PITCH = 0.0              # keep car level during flight
FORWARD_FORCE = 200.0            # driving force toward ramp
RAMP_ANGLE = 0.45 #25 * math.pi / 180 # degrees → radians
MAX_STEPS = 800

task_states = {"ascend" : False, "launched" : False, "landed" : False}


# ============================================================
#  UTILITIES
# ============================================================
def check_task_state(car, plane, ramp, time_step):
    global task_states
    # car_pos, car_orn = p.getBasePositionAndOrientation(car)
    # _, pitch, _ = p.getEulerFromQuaternion(car_orn)
    
    if len(p.getContactPoints(car, ramp)) > 0:
        print("On Ramp at step:", time_step)
        task_states["launched"] = False
        task_states["ascend"] = True
    
    if task_states["ascend"] and not task_states["launched"] and len(p.getContactPoints(car, plane)) == 0:
        print("Airborne at step:", time_step)
        task_states["launched"] = True
    
    if task_states["ascend"] and task_states["launched"] and len(p.getContactPoints(car, plane)) > 0 and not task_states["landed"]:
        print("Landed at step:", time_step)
        task_states["landed"] = True
        time_step = MAX_STEPS - 100     # end simulation on landing
    
    return time_step

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

    def step(self, error):
        self.integral += error
        deriv = error - self.prev_error
        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*deriv


# ============================================================
#  PYBULLET INITIALIZATION
# ============================================================
def init_sim():
    cid = p.connect(p.GUI)          # GUI mode ON
    p.resetDebugVisualizerCamera(
        cameraDistance=4,
        cameraYaw=0,
        cameraPitch=-20,
        cameraTargetPosition=[0, 0, 0.5]
    )
    p.setTimeStep(TIME_STEP)
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    return cid


# ============================================================
#  LOAD WORLD
# ============================================================
def load_world():
    plane = p.loadURDF("plane.urdf")

    # ----- Ramp -----
    # A long thin box rotated about Y-axis
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

    # Position the ramp in front of the car
    ramp_pos = [8, 0, 0.2]
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
    car_scale = 1.0
    cube_size = 0.05 * car_scale
    car = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.1], globalScaling=2.0)

    # Get car mass
    dyn = p.getDynamicsInfo(car, -1)
    car_mass = dyn[0]
    cube_mass = car_mass * CUBE_MASS_RATIO

    # ----- internal cube -----
    cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_size, cube_size, cube_size])
    cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_size, cube_size, cube_size],
                                   rgbaColor=[0, 1, 0, 1])

    cube = p.createMultiBody(
        baseMass=cube_mass,
        baseCollisionShapeIndex=cube_col,
        baseVisualShapeIndex=cube_vis,
        basePosition=[0.2, 0, 0.2]
    )

    return car, cube, cube_mass


# ============================================================
#  MAIN SIM LOGIC
# ============================================================
def run_sim():
    pid = PID(PID_KP, PID_KI, PID_KD)

    init_sim()
    plane,ramp = load_world()
    car, cube, cube_mass = load_car_with_mass()

    # Wheels = joints 2 and 3 for rear drive
    wheel_joints = [2, 3 ,5, 7]

    for j in wheel_joints:
        p.changeDynamics(car, j, lateralFriction=1.2, spinningFriction=0.02, rollingFriction=0.0)

    # Move cube initially at center
    current_local_cube_pos = [0, 0, 0.2]

    time_step = 0

    with tqdm(total=MAX_STEPS) as pbar:
        while time_step < MAX_STEPS:
            time_step += 1
            pbar.update(1)
            # ----------------------------------------------------------
            # DRIVE FORWARD
            # ----------------------------------------------------------
            for j in wheel_joints:
                if j in [2, 3]:  # rear wheels
                    p.setJointMotorControl2(
                        bodyUniqueId=car,
                        jointIndex=j,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=100,
                        force=FORWARD_FORCE
                    )
                # else:           # front wheels
                #     p.setJointMotorControl2(
                #         bodyUniqueId=car,
                #         jointIndex=j,
                #         controlMode=p.VELOCITY_CONTROL,
                #         targetVelocity=100,
                #         force=FORWARD_FORCE * 0.2
                #     )

            # ----------------------------------------------------------
            # CAR STATE
            # ----------------------------------------------------------
            car_pos, car_orn = p.getBasePositionAndOrientation(car)
            _, pitch, _ = p.getEulerFromQuaternion(car_orn)

            # vertical distance from ground
            height = car_pos[2]

            # time before landing (simple estimate)
            if height > 0.3:
                time_to_collision = math.sqrt(2 * (height - 0.2) / 9.8)
            else:
                time_to_collision = 0

            time_step = check_task_state(car, plane, ramp, time_step)
            pbar.n = time_step

            # ----------------------------------------------------------
            # MID-AIR CONTROL (only apply when above ground)
            # ----------------------------------------------------------
            if height > 0.3:
                pitch_error = TARGET_PITCH - pitch
                cube_shift = pid.step(pitch_error)

                # Limit shift
                cube_shift = max(-CUBE_Y_LIMIT, min(CUBE_Y_LIMIT, cube_shift))

                current_local_cube_pos = [cube_shift, 0,  0.2]

            # ----------------------------------------------------------
            # UPDATE CUBE POSITION
            # ----------------------------------------------------------
            cube_world = local_to_world(car, current_local_cube_pos)
            p.resetBasePositionAndOrientation(cube, cube_world, car_orn)

            # ----------------------------------------------------------
            # FOLLOW CAMERA
            # ----------------------------------------------------------
            p.resetDebugVisualizerCamera(
                cameraDistance=4,
                cameraYaw=20,
                cameraPitch=-20,
                cameraTargetPosition=car_pos
            )

            # Step simulation
            p.stepSimulation()
            time.sleep(TIME_STEP)
    
    p.disconnect()
    print("Simulation complete.")


# ============================================================
#  ENTRY
# ============================================================
if __name__ == "__main__":
    run_sim()