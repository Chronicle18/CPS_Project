import pybullet as p
import yaml
import datetime
import argparse
import numpy as np
import os
from tqdm import tqdm
from utils.videowriter import VideoWriter
from modules.custom_envs import CarJumpEnv
from modules.camera import Camera
import utils.geom_utils as geom
import utils.monitor_info as monitor

parser = argparse.ArgumentParser(description="Car Jump Simulation WITHOUT Internal Mass Control (Baseline)")
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
    cfg['logging']['video_file'] = f"car_jump_no_cube_{timestamp}.mp4"

TASK_STATES = {"ascend" : False, "launched" : False, "landed" : False}
MAX_STEPS = cfg['simulation']['max_steps']

os.makedirs(cfg['logging']['save_dir'], exist_ok=True)


# ============================================================
#  MAIN SIM LOGIC (NO CUBE CONTROL - BASELINE)
# ============================================================
def run_sim(cfg):

    # initialize environment
    env = CarJumpEnv(cfg)
    plane, ramp = env.plane, env.ramp
    car, cube = env.car, env.cube
    main_cam = env.main_cam
    pov_cam = env.pov_cam

    # Cube stays at initial position (no control)
    initial_cube_pos = cfg['cube']['local_initial_pos']
    time_step = 0

    # NO PID controller - this is the baseline version

    # logging setup
    video_path = os.path.join(cfg['logging']['save_dir'], cfg['logging']['video_file'])
    vid_writer = VideoWriter(video_path, frame_size=(cfg['camera']['width'], cfg['camera']['height']), fps=cfg['logging']['video_fps'])

    # Get speed profile settings
    selected_profile = cfg['car'].get('speed_profile', 'NORMAL')
    speed_config = cfg['speed_profiles'][selected_profile]
    target_velocity = speed_config['target_velocity']
    forward_force = speed_config['forward_force']
    
    print(f"\n{'='*60}")
    print(f"BASELINE MODE: NO CUBE CONTROL")
    print(f"SPEED MODE: {selected_profile}")
    print(f"Description: {speed_config['description']}")
    print(f"Target Velocity: {target_velocity}")
    print(f"Forward Force: {forward_force}")
    print(f"{'='*60}\n")

    # Acceleration control
    ACCELERATION_START_TIME = cfg['car'].get('acceleration_delay', 0.5)  # Default seconds
    ACCELERATION_RAMP_DURATION = 1.0  # Total time to reach 100% (adjust as needed)
    acceleration_active = False
    acceleration_start_timestamp = None
    
    # Airtime tracking
    airborne_start_time = None
    total_airtime = 0.0
    is_airborne = False
    
    rear_wheel_joints = cfg['car']['rear_whls']
    front_wheel_joints = cfg['car']['front_whls']
    steering_joints = [4,6]

    # Simulation loop
    with tqdm(total=MAX_STEPS) as pbar:
        while time_step < MAX_STEPS:
            time_step += 1
            pbar.update(1)
            
            current_time = time_step * cfg['simulation']['time_step']

            # ----------------------------------------------------------
            # CAR STATE
            # ----------------------------------------------------------
            car_pos, car_orn = p.getBasePositionAndOrientation(car)
            car_vel, car_ang_vel = p.getBaseVelocity(car)
            _, pitch, _ = p.getEulerFromQuaternion(car_orn)

            # Calculate current speed
            current_speed = np.linalg.norm(car_vel)
            
            # ----------------------------------------------------------
            # CHECK AIRBORNE STATUS (with safety check for None)
            # ----------------------------------------------------------
            wheel_contacts = []
            for wheel_joint in [2, 3, 5, 7]:  # all wheels
                contacts = p.getContactPoints(bodyA=car, linkIndexA=wheel_joint)
                if contacts is not None:  # Safety check
                    wheel_contacts.extend(contacts)
            
            was_airborne = is_airborne
            is_airborne = len(wheel_contacts) == 0
            
            # Track airtime
            if is_airborne and not was_airborne:
                airborne_start_time = current_time
                print(f"[{current_time:.2f}s] Airborne!")
            elif not is_airborne and was_airborne:
                if airborne_start_time is not None:
                    flight_duration = current_time - airborne_start_time
                    total_airtime += flight_duration
                    print(f"[{current_time:.2f}s] Landed! Flight: {flight_duration:.2f}s")
                    airborne_start_time = None
            
            # Calculate current airtime
            if is_airborne and airborne_start_time is not None:
                current_airtime = current_time - airborne_start_time
            else:
                current_airtime = 0.0

            # ----------------------------------------------------------
            # KEEP FRONT WHEELS STRAIGHT
            # ----------------------------------------------------------
            for j in steering_joints:
                p.setJointMotorControl2(
                    bodyUniqueId=car,
                    jointIndex=j,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=0,  # Straight ahead
                    force=1000
                )

            # ----------------------------------------------------------
            # ACCELERATION LOGIC
            # ----------------------------------------------------------
            # Start acceleration after delay
            if current_time >= ACCELERATION_START_TIME and not acceleration_active and not is_airborne:
                acceleration_active = True
                acceleration_start_timestamp = current_time
                print(f"[{current_time:.2f}s] Acceleration started!")
            
            # Stop acceleration when airborne
            if is_airborne and acceleration_active:
                acceleration_active = False
                print(f"[{current_time:.2f}s] Airborne - stopping acceleration")

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
            if monitor.hasLanded(car, plane, ramp, TASK_STATES):
                print("Landed at step:", time_step)
                time_step = MAX_STEPS - 200
                pbar.n = time_step

            # ----------------------------------------------------------
            # NO MID-AIR CONTROL - CUBE STAYS FIXED
            # ----------------------------------------------------------
            # Cube remains at initial position (no PID control)
            cube_world = geom.local_to_world(car, initial_cube_pos)
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
            overlay_data = {
                'current_speed': current_speed,
                'target_speed': target_velocity / 10,  # Convert to m/s if needed
                'airtime': current_airtime if is_airborne else total_airtime,
                'is_airborne': is_airborne
            }
            
            vid_writer.write_frame(rgb, postprocess=True, overlay_data=overlay_data)
    
    p.disconnect()
    vid_writer.release()
    
    print(f"\nTotal airtime: {total_airtime:.2f}s")
    print("Video saved.")
    print("Simulation complete (BASELINE - NO CUBE CONTROL).")


if __name__ == "__main__":
    run_sim(cfg)

