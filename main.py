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

    # Calculate cube movement limits based on car geometry
    aabb_min, aabb_max = p.getAABB(car)
    car_length = aabb_max[0] - aabb_min[0]
    aabb_center_world = [
        (aabb_min[0] + aabb_max[0]) / 2,
        (aabb_min[1] + aabb_max[1]) / 2,
        (aabb_min[2] + aabb_max[2]) / 2
    ]
    local_center = geom.world_to_local(car, aabb_center_world)
    cube_size = cfg['cube']['size_factor'] * cfg['car']['scale']
    max_shift = (car_length / 2) - (cube_size / 2)  # Full length minus cube size
    limit_front = max(max_shift, 0.5)  # Ensure at least 0.5m
    limit_back = max(max_shift, 0.02)
    print(f"Car length: {car_length:.3f}, Cube size: {cube_size:.3f}, Max shift: {max_shift:.3f}")

    # Adjust center to make limits equidistant from cube center
    cube_center_x = (limit_front + limit_back) / 2
    print(f"Adjusted cube center: {cube_center_x:.3f}")

    # initialize controller
    controller = CubeController(cfg['pid'], limit_front, limit_back, cube_center_x)
    
    # Initial cube position
    current_local_cube_pos = [controller.local_center_x, 0, 0.2]
    time_step = 0


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
    ACCELERATION_RAMP_DURATION = 2.0  # Increased duration for smoother acceleration
    acceleration_active = False
    acceleration_start_timestamp = None
    
    rear_wheel_joints = cfg['car']['rear_whls']
    front_wheel_joints = cfg['car']['front_whls']
    steering_joints = [4,6]
    
    # Estimate wheel radius from car scale (typical racecar wheel ~0.03-0.05m radius scaled by car scale)
    wheel_radius = 0.04 * cfg['car']['scale']  # Approximate wheel radius in meters
    print(f"Estimated wheel radius: {wheel_radius:.3f}m")

    # Simulation loop
    with tqdm(total=MAX_STEPS) as pbar:
        while time_step < MAX_STEPS:
            time_step += 1
            pbar.update(1)
            
            current_time = time_step * cfg['simulation']['time_step']

            # ----------------------------------------------------------
            # DRIVE FORWARD
            # ----------------------------------------------------------
            for j in rear_wheel_joints:
                    p.setJointMotorControl2(
                        bodyUniqueId=car,
                        jointIndex=j,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=target_velocity,
                        force=forward_force
                    )
            
            # Allow front wheels to spin freely (passive rotation)
            for j in front_wheel_joints:
                p.setJointMotorControl2(
                    bodyUniqueId=car,
                    jointIndex=j,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=0  # No force allows free spinning
                )

            # ----------------------------------------------------------
            # CAR STATE
            # ----------------------------------------------------------
            car_pos, car_orn = p.getBasePositionAndOrientation(car)
            roll, pitch, yaw = p.getEulerFromQuaternion(car_orn)
            current_speed, curr_ang_vel = monitor.get_velocity(car)
            
            # ----------------------------------------------------------
            # CHECK LANDING
            # ----------------------------------------------------------
            if monitor.hasLanded(car, plane, ramp, time_step):
                print("Landed at step:", time_step)
                time_step = MAX_STEPS - 200
                pbar.n = time_step
                if not monitor.landing_roll and not monitor.landing_pitch:
                    monitor.landing_pitch, monitor.landing_roll = pitch, roll

            # CHECK AIRBORNE STATUS
            current_airtime = monitor.check_airborne_status(car, plane, time_step)

            # ----------------------------------------------------------
            # KEEP FRONT WHEELS STRAIGHT
            # ----------------------------------------------------------
            for j in steering_joints:
                p.setJointMotorControl2(
                    bodyUniqueId=car,
                    jointIndex=j,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=0,  # Straight ahead
                    force=5000  # Increased force for better stability
                )
            
            # ----------------------------------------------------------
            # YAW STABILIZATION - Keep car driving straight
            # ----------------------------------------------------------
            # _, _, yaw = p.getEulerFromQuaternion(car_orn)
            if not monitor.is_airborne:
                # Apply corrective torque to keep car straight
                yaw_error = 0 - yaw  # Target is 0 yaw (straight)
                corrective_torque = yaw_error * 2000  # Increased proportional control
                # Also dampen yaw angular velocity
                yaw_damping = -curr_ang_vel[2] * 100
                total_torque = corrective_torque + yaw_damping
                p.applyExternalTorque(car, -1, [0, 0, total_torque], flags=p.WORLD_FRAME)

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
            # MID-AIR CONTROL
            # ----------------------------------------------------------
            if monitor.is_airborne:
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
            # target_velocity is angular velocity (rad/s), convert to linear speed: v = ω * r
            target_linear_speed = target_velocity * wheel_radius
            overlay_data = {
                'timestep' : time_step,
                'current_speed': current_speed,
                'target_speed': target_linear_speed / 10,  # Convert to m/s if needed
                'airtime': current_airtime if monitor.is_airborne else monitor.total_airtime,
                'is_airborne': monitor.is_airborne
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