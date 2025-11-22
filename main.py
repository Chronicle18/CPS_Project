import pybullet as p
import yaml
import datetime
import argparse
import numpy as np
import os
import math
from tqdm import tqdm
from utils.videowriter import VideoWriter
from modules.controllers import PID
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
    plane,ramp = env.plane, env.ramp
    car, cube = env.car, env.cube
    main_cam = env.main_cam
    pov_cam = env.pov_cam


    # Move cube initially at center
    current_local_cube_pos = cfg['cube']['local_initial_pos']
    time_step = 0

    # initialize controller
    pid = PID(cfg['pid']['kp'], cfg['pid']['ki'], cfg['pid']['kd'])

    # logging setup
    video_path = os.path.join(cfg['logging']['save_dir'], cfg['logging']['video_file'])
    vid_writer = VideoWriter(video_path, frame_size=(cfg['camera']['width'], cfg['camera']['height']), fps=cfg['logging']['video_fps'])

    # Simulation loop
    with tqdm(total=MAX_STEPS) as pbar:
        while time_step < MAX_STEPS:
            time_step += 1
            pbar.update(1)

            # ----------------------------------------------------------
            # DRIVE FORWARD
            # ----------------------------------------------------------
            for j in cfg['car']['wheel_joints']:
                if j in [2,3]:  # rear wheels
                    p.setJointMotorControl2(
                        bodyUniqueId=car,
                        jointIndex=j,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=cfg['car']['target_velocity'],
                        force=cfg['car']['forward_force']
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
            # height = car_pos[2]

            # time before landing (simple estimate)
            # if height > 0.3:
            #     time_to_collision = math.sqrt(2 * (height - 0.2) / 9.8)
            # else:
            #     time_to_collision = 0

            if monitor.hasLanded(car, plane, ramp, TASK_STATES):
                print("Landed at step:", time_step)
                time_step = MAX_STEPS - 200
                pbar.n = time_step

            # ----------------------------------------------------------
            # MID-AIR CONTROL
            # ----------------------------------------------------------
            # if height > 0.3:
            pitch_error = cfg['pid']['target_pitch'] - pitch
            cube_shift = pid.step(pitch_error)

            # Limit shift
            cube_shift = np.clip(cube_shift, -cfg['cube']['limit_x'], cfg['cube']['limit_x']) #max(-CUBE_X_LIMIT, min(CUBE_X_LIMIT, cube_shift))

            current_local_cube_pos = [cube_shift, 0,  0.2]

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
            vid_writer.write_frame(rgb, postprocess=True)
            # time.sleep(cfg['simulation']['time_step'])
    
    p.disconnect()
    vid_writer.save_and_release()
    print("Video saved.")
    print("Simulation complete.")


if __name__ == "__main__":
    run_sim(cfg)