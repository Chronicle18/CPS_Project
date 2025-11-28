import pybullet as p
import yaml
import datetime
import argparse
import numpy as np
import os
from tqdm import tqdm
from utils.videowriter import VideoWriter
from modules.controllers import PID2D, PID
from modules.custom_envs import CarJumpEnv
import utils.geom_utils as geom
from utils.monitor_info import MonitorInfo

parser = argparse.ArgumentParser(description="Car Jump Simulation with Internal Mass Control")
parser.add_argument('--config', type=str, default='cfg/config.yaml', help='Path to configuration YAML file')
parser.add_argument('--mode', type=str, default=None, choices=['GUI', 'HEADLESS'], help='Simulation mode')
parser.add_argument('--output', type=str, default=None, help='Output video file name')
args = parser.parse_args()


# ============================================================
#  CONFIG
# ============================================================
with open(args.config, "r") as f:
    cfg = yaml.safe_load(f)

if args.mode:
    cfg['simulation']['mode'] = args.mode

if args.output:
    cfg['logging']['video_file'] = args.output
else:
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    cfg['logging']['video_file'] = f"car_jump_{timestamp}.mp4"

TASK_STATES = {"ascend" : False, "launched" : False, "landed" : False}
MAX_STEPS = cfg['simulation']['max_steps']

os.makedirs(cfg['logging']['save_dir'], exist_ok=True)


# ============================================================
#  MAIN SIM LOGIC
# ============================================================
def run_sim(cfg):

    # initialize environment
    env = CarJumpEnv(cfg)
    plane, ramp = env.plane, env.ramp
    car, cube = env.car, env.cube
    main_cam = env.main_cam
    pov_cam = env.pov_cam

    if cube is not None:
        # Move cube initially at shifted center (0.2m forward)
        cube_center_x = cfg['cube']['local_initial_pos'][0]
        cube_center_y = cfg['cube']['local_initial_pos'][1]
        current_local_cube_pos = cfg['cube']['local_initial_pos']

    time_step = 0

    # initialize 2D controller for pitch and roll
    pid_2d = PID2D(
        cfg['pid']['kp'], cfg['pid']['ki'], cfg['pid']['kd'],
        cfg['pid']['kp_roll'], cfg['pid']['ki_roll'], cfg['pid']['kd_roll']
    )
    
    # Initialize yaw PID for straight line control
    yaw_pid = PID(kp=2.0, ki=0.0, kd=0.1)  # Tune these values as needed
    
    # Cube movement limits
    if cube is not None:
        limit_x_forward = cfg['cube']['limit_x_forward']
        limit_x_backward = cfg['cube']['limit_x_backward']
        limit_y = cfg['cube']['limit_y']
    
    # History tracking for smoothed decision making (last 5 steps)
    # cube_shift_history = deque(maxlen=5)
    # for _ in range(5):
    #     cube_shift_history.append([0.0, 0.0])  # [x_shift, y_shift]

    # logging setup
    video_path = os.path.join(cfg['logging']['save_dir'], cfg['logging']['video_file'])
    vid_writer = VideoWriter(video_path, frame_size=(cfg['camera']['width'], cfg['camera']['height']), fps=cfg['logging']['video_fps'])
    monitor = MonitorInfo(cfg)

    # Get speed profile settings
    selected_profile = cfg['car'].get('speed_profile', 'NORMAL')
    speed_config = cfg['speed_profiles'][selected_profile]
    target_velocity = speed_config['target_velocity']
    forward_force = speed_config['forward_force']
    
    print(f"\n{'='*60}")
    print(f"SPEED MODE: {selected_profile}")
    print(f"Description: {speed_config['description']}")
    print(f"Target Velocity: {target_velocity}")
    print(f"Forward Force: {forward_force}")
    print(f"{'='*60}\n")

    # print("\n" + "="*60)
    # print("JOINT INFORMATION:")
    # print("="*60)
    # for i in range(p.getNumJoints(car)):
    #     joint_info = p.getJointInfo(car, i)
    #     joint_name = joint_info[1].decode('utf-8')
    #     joint_type = joint_info[2]
    #     print(f"Joint {i}: {joint_name} (Type: {joint_type})")
    # print("="*60 + "\n")

    # Acceleration control
    ACCELERATION_START_TIME = cfg['car'].get('acceleration_delay', 0.5)  # Default seconds
    ACCELERATION_RAMP_DURATION = 1.0  # Total time to reach 100% (adjust as needed)
    acceleration_active = False
    acceleration_start_timestamp = None
    
    rear_wheel_joints = cfg['car']['rear_whls']
    front_wheel_joints = cfg['car']['front_whls']
    steering_joints = [4,6]
    
    # Get car dimensions for wheel positions (for landing angle calculation)
    aabb_min, aabb_max = p.getAABB(car)
    car_length = aabb_max[0] - aabb_min[0]
    wheelbase = car_length * 0.7  # Approximate wheelbase
    print(f"Car length: {car_length:.3f}m, Estimated wheelbase: {wheelbase:.3f}m")

    # Simulation loop
    with tqdm(total=MAX_STEPS) as pbar:
        frame_number = 0
        while frame_number < MAX_STEPS:
            frame_number += 1
            time_step += 1
            pbar.update(1)
            
            current_time = time_step * cfg['simulation']['time_step']

            # ----------------------------------------------------------
            # CAR STATE
            # ----------------------------------------------------------
            car_pos, car_orn = p.getBasePositionAndOrientation(car)
            roll, pitch, yaw = p.getEulerFromQuaternion(car_orn)
            current_speed, curr_ang_vel = monitor.get_velocity(car)
            
            # ----------------------------------------------------------
            # CHECK AIRBORNE STATUS (with safety check for None)
            # ----------------------------------------------------------
            # Check if touching ramp (any part of car)
            current_airtime = monitor.check_airborne_status(car, plane, ramp, time_step)

            # ----------------------------------------------------------
            # STEERING CONTROL - Keep car on straight line
            # ----------------------------------------------------------
            yaw_error = 0 - yaw  # Target yaw is 0 (straight)
            steering_angle = yaw_pid.step(yaw_error)
            
            for j in steering_joints:
                p.setJointMotorControl2(
                    bodyUniqueId=car,
                    jointIndex=j,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=steering_angle,  # Adjust steering to correct yaw
                    force=1000
                )
            if yaw_error != 0:
                # Apply motor control to rear wheels
                for j in front_wheel_joints:
                    p.setJointMotorControl2(
                        bodyUniqueId=car,
                        jointIndex=j,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=target_velocity,
                        force=forward_force
                    )

            

            # ----------------------------------------------------------
            # ACCELERATION LOGIC
            # ----------------------------------------------------------
            # Start acceleration after delay
            if current_time >= ACCELERATION_START_TIME and not acceleration_active and not monitor.is_airborne:
                acceleration_active = True
                acceleration_start_timestamp = current_time
                print(f"[{current_time:.2f}s] Acceleration started!")
            
            # Stop acceleration when airborne
            # if monitor.is_airborne and acceleration_active:
            #     acceleration_active = False
            #     print(f"[{current_time:.2f}s] Airborne - stopping acceleration")

            # ----------------------------------------------------------
            # DRIVE FORWARD WITH GRADUAL ACCELERATION
            # ----------------------------------------------------------
            if acceleration_active and acceleration_start_timestamp is not None:
                # Calculate elapsed time since acceleration started
                elapsed_acceleration_time = current_time - acceleration_start_timestamp
                
                # Gradual acceleration: 0% -> 20% -> 40% -> 60% -> 80% -> 100%
                if elapsed_acceleration_time < ACCELERATION_RAMP_DURATION * 0.2:  # 0-20%
                    power_fraction = 0.2
                elif elapsed_acceleration_time < ACCELERATION_RAMP_DURATION * 0.4:  # 20-40%
                    power_fraction = 0.4
                elif elapsed_acceleration_time < ACCELERATION_RAMP_DURATION * 0.6:  # 40-60%
                    power_fraction = 0.6
                elif elapsed_acceleration_time < ACCELERATION_RAMP_DURATION * 0.8:  # 60-80%
                    power_fraction = 0.8
                else:  # 80-100%
                    power_fraction = 1.0
                
                current_velocity = target_velocity * power_fraction
                current_force = forward_force * power_fraction
                
                # Apply motor control to rear wheels
                for j in rear_wheel_joints:
                    p.setJointMotorControl2(
                        bodyUniqueId=car,
                        jointIndex=j,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=current_velocity,
                        force=current_force
                    )
                
                # DEBUG: Print wheel states every 50 steps
                # if time_step % 50 == 0:
                #     print(f"\n[DEBUG {current_time:.2f}s] Power: {power_fraction*100:.0f}%")
                #     print(f"Car position: {car_pos}")
                #     print(f"Car orientation (yaw): {p.getEulerFromQuaternion(car_orn)[2]:.3f} rad")
                #     for j in rear_wheel_joints:
                #         joint_state = p.getJointState(car, j)
                #         print(f"  Wheel {j}: velocity={joint_state[1]:.2f}")
            else:
                # No acceleration - wheels coast
                for j in rear_wheel_joints:
                    p.setJointMotorControl2(
                        bodyUniqueId=car,
                        jointIndex=j,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=0,
                        force=0
                    )

            # ----------------------------------------------------------
            # CHECK LANDING
            # ----------------------------------------------------------
            if monitor.hasLanded(car, plane, ramp, time_step):
                print("Landed at step:", time_step)
                frame_number = MAX_STEPS - 200
                pbar.n = frame_number
                if monitor.landing_pitch is None and monitor.landing_roll is None:
                    monitor.landing_pitch, monitor.landing_roll = pitch, roll

            # ----------------------------------------------------------
            # MID-AIR CONTROL - 2D Cube Movement with Decision Smoothing
            # ----------------------------------------------------------
            if cube is not None:
                if monitor.is_airborne:
                    # Calculate pitch and roll errors
                    # Physics: When pitch is POSITIVE (nose up), shift cube FORWARD (positive X)
                    # to create torque that brings nose down.
                    # When pitch is NEGATIVE (nose down), shift cube BACKWARD (negative X)
                    pitch_error = pitch - cfg['pid']['target_pitch']  # Inverted error for correct direction
                    roll_error = roll - cfg['pid']['target_roll']     # Inverted error for correct direction
                    
                    # Get raw PID outputs
                    raw_x_shift, raw_y_shift = pid_2d.step(pitch_error, roll_error)
                    
                    # Add to history for smoothing
                    # cube_shift_history.append([raw_x_shift, raw_y_shift])
                    
                    # Smoothed decision: average of last 5 steps to minimize jitter
                    # history_array = np.array(cube_shift_history)
                    # smoothed_x_shift = np.mean(history_array[:, 0])
                    # smoothed_y_shift = np.mean(history_array[:, 1])
                    
                    # Apply limits (shifted center: x=0.45, y=0)
                    
                    # X-axis: Range from (center - backward) to (center + forward)
                    # Center is at 0.45, so range is [0.20, 0.90]
                    cube_shift_x = np.clip(raw_x_shift, -limit_x_backward, limit_x_forward)
                    
                    # Y-axis: Range from -0.15 to +0.15 (symmetric around center)
                    cube_shift_y = 0  # Only x direction for initial testing
                    
                    # LANDING ANGLE PREDICTION: Adjust for 4-wheel touchdown
                    # Predict wheel heights based on current pitch and angular velocity
                    # Target: rear and front wheels at same height when landing
                    
                    # Get wheel heights in world frame
                    front_wheel_states = [p.getLinkState(car, j) for j in front_wheel_joints]
                    rear_wheel_states = [p.getLinkState(car, j) for j in rear_wheel_joints]
                    
                    front_wheel_avg_z = np.mean([state[0][2] for state in front_wheel_states])
                    rear_wheel_avg_z = np.mean([state[0][2] for state in rear_wheel_states])
                    
                    wheel_height_diff = front_wheel_avg_z - rear_wheel_avg_z
                    
                    # Predict landing pitch: if front is higher, we need nose-down correction
                    # Angular velocity also indicates rotation direction
                    pitch_velocity = curr_ang_vel[1]  # Pitch rate (rad/s)
                    
                    # Predictive correction for level landing
                    # If front wheels are higher, shift cube FORWARD (positive) to bring nose down
                    # If rear wheels are higher, shift cube BACKWARD (negative) to bring nose up
                    LANDING_PREDICTION_GAIN = 2.0
                    landing_correction_x = wheel_height_diff * LANDING_PREDICTION_GAIN
                    landing_correction_x += pitch_velocity * 0.5  # If pitching up, shift forward
                    
                    # Apply landing correction to X-shift
                    cube_shift_x += landing_correction_x
                    cube_shift_x = np.clip(cube_shift_x, -limit_x_backward, limit_x_forward)
                    
                    # Final cube position in local frame (relative to shifted center)
                    current_local_cube_pos = [cube_center_x + cube_shift_x, cube_center_y + cube_shift_y, 0.2]
                else:
                    # On ground: Keep cube at shifted center
                    current_local_cube_pos = [cube_center_x, cube_center_y, 0.2]

                # ----------------------------------------------------------
                # UPDATE CUBE POSITION
                # ----------------------------------------------------------
                cube_world = geom.local_to_world(car, current_local_cube_pos)
                p.resetBasePositionAndOrientation(cube, cube_world, car_orn)

            # ----------------------------------------------------------
            # FOLLOW CAMERA
            # ----------------------------------------------------------
            if cfg['simulation']['mode'] == "GUI":
                p.resetDebugVisualizerCamera(
                    cameraDistance=4,
                    cameraYaw=20,
                    cameraPitch=-20,
                    cameraTargetPosition=car_pos
                )

            # Step simulation
            p.stepSimulation()
            rgb = main_cam.get_image()
            
            # Prepare overlay data
            # target_linear_speed = target_velocity * wheelbase
            overlay_data = {
                'timestep': time_step,
                'current_speed': current_speed * 10,
                # 'target_speed': target_velocity,  # Convert to m/s if needed
                'airtime': current_airtime if monitor.is_airborne else monitor.total_airtime,
                'monitor.is_airborne': monitor.is_airborne
            }
            
            vid_writer.write_frame(rgb, postprocess=True, overlay_data=overlay_data)

    if monitor.task_states['landed']:
        monitor.evaluate_episode()
    else:
        print("Car never landed.")
    
    p.disconnect()
    vid_writer.release()
    
    print(f"\nTotal airtime: {monitor.total_airtime:.2f}s")
    print("Video saved.")
    print("Simulation complete.")


if __name__ == "__main__":
    run_sim(cfg)