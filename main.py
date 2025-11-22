import pybullet as p
import yaml
import datetime
import argparse
import numpy as np
import os
from tqdm import tqdm
from utils.videowriter import VideoWriter
from modules.controllers import CubeController
from modules.custom_envs import CarJumpEnv
from modules.camera import Camera
import utils.geom_utils as geom
import utils.monitor_info as monitor

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

    # Calculate asymmetric limits
    aabb_min, aabb_max = p.getAABB(car)
    car_length = aabb_max[0] - aabb_min[0]
    aabb_center_world = [
        (aabb_min[0] + aabb_max[0]) / 2,
        (aabb_min[1] + aabb_max[1]) / 2,
        (aabb_min[2] + aabb_max[2]) / 2
    ]
    local_center = geom.world_to_local(car, aabb_center_world)
    local_front_x = geom.world_to_local(car, [aabb_max[0], 0, 0])[0]
    local_back_x = geom.world_to_local(car, [aabb_min[0], 0, 0])[0]
    cube_size = cfg['cube']['size_factor'] * cfg['car']['scale']
    limit_front = local_front_x - local_center[0] - (cube_size / 2.0)
    limit_back = local_center[0] - local_back_x - (cube_size / 2.0)
    limit_front = min(cfg['cube']['limit_x'] * 2, limit_front)
    limit_back = min(cfg['cube']['limit_x'], limit_back) # Use config limit for back as well

    # initialize controller
    controller = CubeController(cfg['pid'], limit_front, limit_back, local_center[0])
    
    # Initial cube position
    current_local_cube_pos = [controller.local_center_x, 0, 0.2]
    time_step = 0


    # logging setup
    video_path = os.path.join(cfg['logging']['save_dir'], cfg['logging']['video_file'])
    vid_writer = VideoWriter(video_path, frame_size=(cfg['camera']['width'], cfg['camera']['height']), fps=cfg['logging']['video_fps'])

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

    # Airtime tracking
    airborne_start_time = None
    total_airtime = 0.0
    is_airborne = False
    
    wheel_joints = cfg['car']['wheel_joints']

    # Simulation loop
    with tqdm(total=MAX_STEPS) as pbar:
        while time_step < MAX_STEPS:
            time_step += 1
            pbar.update(1)
            
            current_time = time_step * cfg['simulation']['time_step']

            # ----------------------------------------------------------
            # DRIVE FORWARD
            # ----------------------------------------------------------
            for j in wheel_joints:
                if j in [4, 5]:  # front wheels (FWD)
                    p.setJointMotorControl2(
                        bodyUniqueId=car,
                        jointIndex=j,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=target_velocity,
                        force=forward_force
                    )
                else: # rear wheels (free spinning)
                    p.setJointMotorControl2(
                        bodyUniqueId=car,
                        jointIndex=j,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=0,
                        force=0
                    )

            # ----------------------------------------------------------
            # CAR STATE
            # ----------------------------------------------------------
            car_pos, car_orn = p.getBasePositionAndOrientation(car)
            car_vel, car_ang_vel = p.getBaseVelocity(car)
            _, pitch, _ = p.getEulerFromQuaternion(car_orn)

            # Calculate current speed
            current_speed = np.linalg.norm(car_vel)
            
            # ----------------------------------------------------------
            # CHECK AIRBORNE STATUS
            # ----------------------------------------------------------
            wheel_contacts = []
            for wheel_joint in [2, 3]:  # rear wheels
                contacts = p.getContactPoints(bodyA=car, linkIndexA=wheel_joint)
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
            # CHECK LANDING
            # ----------------------------------------------------------
            if monitor.hasLanded(car, plane, ramp, TASK_STATES):
                print("Landed at step:", time_step)
                time_step = MAX_STEPS - 200
                pbar.n = time_step

            # ----------------------------------------------------------
            # MID-AIR CONTROL
            # ----------------------------------------------------------
            
            # ----------------------------------------------------------
            # MID-AIR CONTROL
            # ----------------------------------------------------------
            
            if is_airborne:
                cube_shift, local_center_x = controller.get_control_action(pitch, time_step)
            else:
                # Center the cube when on ground/ramp
                cube_shift = 0.0
                local_center_x = controller.local_center_x
                
            current_local_cube_pos = [cube_shift + local_center_x, 0, 0.2]

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
            overlay_data = {
                'current_speed': current_speed,
                'target_speed': target_velocity,
                'airtime': current_airtime if is_airborne else total_airtime,
                'is_airborne': is_airborne
            }
            
            vid_writer.write_frame(rgb, postprocess=True, overlay_data=overlay_data)
    
    p.disconnect()
    vid_writer.release()
    
    print(f"\nTotal airtime: {total_airtime:.2f}s")
    print("Video saved.")
    print("Simulation complete.")


if __name__ == "__main__":
    run_sim(cfg)