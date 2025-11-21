import pybullet as p
import pybullet_data
import time
import math
from tqdm import tqdm
from utils.videowriter import VideoWriter
import yaml
import datetime
import argparse
import numpy as np
import os

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
#  UTILITIES
# ============================================================
def check_task_state(car, plane, ramp, time_step):
    # car_pos, car_orn = p.getBasePositionAndOrientation(car)
    # _, pitch, _ = p.getEulerFromQuaternion(car_orn)
    
    if len(p.getContactPoints(car, ramp)) > 0:
        # print("On Ramp at step:", time_step)
        TASK_STATES["launched"] = False
        TASK_STATES["ascend"] = True
    
    if TASK_STATES["ascend"] and not TASK_STATES["launched"] and len(p.getContactPoints(car, plane)) == 0:
        print("Airborne at step:", time_step)
        TASK_STATES["launched"] = True
    
    if TASK_STATES["ascend"] and TASK_STATES["launched"] and len(p.getContactPoints(car, plane)) > 0 and not TASK_STATES["landed"]:
        print("Landed at step:", time_step)
        TASK_STATES["landed"] = True
        time_step = MAX_STEPS - 200     # end simulation on landing
    
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
    if cfg['simulation']['mode'] == "GUI":
        cid = p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=4,
            cameraYaw=0,
            cameraPitch=-20,
            cameraTargetPosition=[0, 0, 0.5]
        )
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)              # hides GUI side panels
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    elif cfg['simulation']['mode'] == "HEADLESS":
        cid = p.connect(p.DIRECT)      # p.GUI     # GUI mode ON
   
    p.setTimeStep(cfg['simulation']['time_step'])
    p.setGravity(*cfg['simulation']['gravity'])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    return cid

def reset_sim():
    pass

class Camera:
    def __init__(self, car, camera_cfg):
        self.car = car

        self.cam_distance = camera_cfg['distance']
        self.cam_yaw = camera_cfg['yaw']
        self.cam_pitch = camera_cfg['pitch']
        self.W = camera_cfg['width']
        self.H = camera_cfg['height']
        self.fov = camera_cfg['fov']
        self.aspect = self.W / self.H
        self.near = camera_cfg['near']
        self.far = camera_cfg['far']
    
    def update_position(self):
        car_pos, _ = p.getBasePositionAndOrientation(self.car)
        target = car_pos

        # cam_pos = [
        #     target[0] - self.cam_distance * math.cos(math.radians(self.cam_yaw)),
        #     target[1] - self.cam_distance * math.sin(math.radians(self.cam_yaw)),
        #     target[2] + 2.0
        # ]

        # cam_pos= target

        # Build view matrix
        # Use Bullet helper to compute a chase camera around the target
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=target,
            distance=self.cam_distance,
            yaw=self.cam_yaw,
            pitch=self.cam_pitch,
            roll=0,
            upAxisIndex=2
        )

        # view = p.computeViewMatrix(
        #     cameraEyePosition=cam_pos,
        #     cameraTargetPosition=target,
        #     cameraUpVector=[0,0,1]
        # )

        # Projection matrix
        projection = p.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect =self.aspect,
            nearVal=self.near,
            farVal =self.far
        )
        return view, projection

    def get_image(self):
        view, proj = self.update_position()
        # choose renderer: use TINY_RENDERER in headless (DIRECT) mode which is reliable;
        # use hardware OpenGL in GUI when available.
        renderer = p.ER_BULLET_HARDWARE_OPENGL if cfg['simulation']['mode'] == "GUI" else p.ER_TINY_RENDERER

        img = p.getCameraImage( 
            width=self.W,
            height=self.H,
            viewMatrix=view,
            projectionMatrix=proj,
            renderer=renderer
        )
        rgb = img[2]

        return rgb



# ============================================================
#  LOAD WORLD
# ============================================================
def load_world():
    plane = p.loadURDF(cfg['world']['plane_urdf'])

    # ----- Ramp -----
    # A long thin box rotated about Y-axis
    ramp_len = cfg['ramp']['length']
    ramp_width = cfg['ramp']['width']
    # ramp_height = ramp_len * math.sin(cfg['ramp']['angle_rad'])
    ramp_thickness = cfg['ramp']['thickness']

    col_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[ramp_len/2, ramp_width/2, ramp_thickness/2]
    )
    vis_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[ramp_len/2, ramp_width/2, ramp_thickness/2],
        rgbaColor=cfg['ramp']['color']
    )

    # Position the ramp in front of the car
    ramp_pos = cfg['ramp']['position']
    ramp_orn = p.getQuaternionFromEuler([0, -cfg['ramp']['angle_rad'], 0])

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
    car_scale = cfg['car']['scale']
    car = p.loadURDF(cfg['car']['urdf'], cfg['car']['position'], globalScaling=car_scale)

    # Get car mass
    dyn = p.getDynamicsInfo(car, -1)
    car_mass = dyn[0]
    print("Car mass:", car_mass)
    

    # ----- internal cube -----
    cube_size = cfg['cube']['size_factor'] * car_scale
    cube_mass = car_mass * cfg['cube']['mass_ratio']

    cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_size, cube_size, cube_size])
    cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_size, cube_size, cube_size],
                                   rgbaColor=cfg['cube']['color'])

    cube = p.createMultiBody(
        baseMass=cube_mass,
        baseCollisionShapeIndex=cube_col,
        baseVisualShapeIndex=cube_vis,
        basePosition=cfg['cube']['position']
    )

    # Wheels = joints 2 and 3 for rear drive
    # wheel_joints = [2, 3 ,5, 4]

    for j in cfg['car']['wheel_joints']:
        p.changeDynamics(car, j, 
                         lateralFriction=cfg['car']['lateral_friction'], 
                         spinningFriction=cfg['car']['spinning_friction'], 
                         rollingFriction=cfg['car']['rolling_friction'])

    return car, cube


# ============================================================
#  MAIN SIM LOGIC
# ============================================================
def run_sim(cfg):
    pid = PID(cfg['pid']['kp'], cfg['pid']['ki'], cfg['pid']['kd'])

    init_sim()
    plane,ramp = load_world()
    car, cube = load_car_with_mass()
    cam = Camera(car, camera_cfg=cfg['camera'])

    # Move cube initially at center
    current_local_cube_pos = cfg['cube']['local_initial_pos']
    time_step = 0

    video_path = os.path.join(cfg['logging']['save_dir'], cfg['logging']['video_file'])
    vid_writer = VideoWriter(video_path, frame_size=(cfg['camera']['width'], cfg['camera']['height']), fps=cfg['logging']['video_fps'])

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
            # if height > 0.3:
            pitch_error = cfg['pid']['target_pitch'] - pitch
            cube_shift = pid.step(pitch_error)

            # Limit shift
            cube_shift = np.clip(cube_shift, -cfg['cube']['limit_x'], cfg['cube']['limit_x']) #max(-CUBE_X_LIMIT, min(CUBE_X_LIMIT, cube_shift))

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
            rgb = cam.get_image()
            vid_writer.write_frame(rgb, postprocess=True)
            # time.sleep(cfg['simulation']['time_step'])
    
    p.disconnect()
    vid_writer.release()
    print("Video saved.")
    print("Simulation complete.")


# ============================================================
#  ENTRY
# ============================================================
if __name__ == "__main__":
    run_sim(cfg)