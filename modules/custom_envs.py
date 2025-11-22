import pybullet as p
import pybullet_data
import sys
sys.path.append("modules/")
from actor import Actor
from camera import Camera

class CarJumpEnv:
    def __init__(self, cfg):
        self.cfg = cfg
        self.cid = self.init_sim()

        self.plane, self.ramp = self.load_world()
        self.car, self.cube = self.load_car_with_cube()
        self.main_cam = Camera(self.car, camera_cfg=cfg['camera'], sim_mode=cfg['simulation']['mode'])
        self.pov_cam = Camera(self.car, camera_cfg=cfg['back_camera'], sim_mode=cfg['simulation']['mode']) if 'back_camera' in cfg else None
    
    def init_sim(self):
        if self.cfg['simulation']['mode'] == "GUI":
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
        elif self.cfg['simulation']['mode'] == "HEADLESS":
            cid = p.connect(p.DIRECT)      # p.GUI     # GUI mode ON
    
        p.setTimeStep(self.cfg['simulation']['time_step'])
        p.setGravity(*self.cfg['simulation']['gravity'])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        return cid
    
    def reset_sim():
        pass
    
    def load_world(self):
        plane = p.loadURDF(self.cfg['world']['plane_urdf'])

        # ----- Ramp -----
        # A long thin box rotated about Y-axis
        ramp_len = self.cfg['ramp']['length']
        ramp_width = self.cfg['ramp']['width']
        # ramp_height = ramp_len * math.sin(cfg['ramp']['angle_rad'])
        ramp_thickness = self.cfg['ramp']['thickness']

        col_id = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[ramp_len/2, ramp_width/2, ramp_thickness/2]
        )
        vis_id = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[ramp_len/2, ramp_width/2, ramp_thickness/2],
            rgbaColor=self.cfg['ramp']['color']
        )

        # Position the ramp in front of the car
        ramp_pos = self.cfg['ramp']['position']
        ramp_orn = p.getQuaternionFromEuler([0, -self.cfg['ramp']['angle_rad'], 0])

        ramp = p.createMultiBody(
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=ramp_pos,
            baseOrientation=ramp_orn
        )

        return plane, ramp

    def load_car_with_cube(self):
        car = self.load_car()
        
        # Get car mass
        dyn = p.getDynamicsInfo(car, -1)
        car_mass = dyn[0]
        print("Car mass:", car_mass)
        
        cube = self.create_cube(car_mass)
        self.setup_car_dynamics(car)

        return car, cube

    def load_car(self):
        car_scale = self.cfg['car']['scale']
        car = p.loadURDF(self.cfg['car']['urdf'], self.cfg['car']['position'], globalScaling=car_scale)
        return car

    def create_cube(self, car_mass):
        car_scale = self.cfg['car']['scale']
        cube_size = self.cfg['cube']['size_factor'] * car_scale
        cube_mass = car_mass * self.cfg['cube']['mass_ratio']

        cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_size, cube_size, cube_size])
        cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_size, cube_size, cube_size],
                                    rgbaColor=self.cfg['cube']['color'])

        cube = p.createMultiBody(
            baseMass=cube_mass,
            baseCollisionShapeIndex=cube_col,
            baseVisualShapeIndex=cube_vis,
            basePosition=self.cfg['cube']['position']
        )
        return cube

    def setup_car_dynamics(self, car):
        for j in self.cfg['car']['wheel_joints']:
            p.changeDynamics(car, j, 
                            lateralFriction=self.cfg['car']['lateral_friction'], 
                            spinningFriction=self.cfg['car']['spinning_friction'], 
                            rollingFriction=self.cfg['car']['rolling_friction'])



