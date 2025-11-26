import pybullet as p

class Camera:
    def __init__(self, target_id, camera_cfg, sim_mode):
        self.target_id = target_id
        self.sim_mode = sim_mode

        self.cam_distance = camera_cfg['distance']
        self.cam_yaw = camera_cfg['yaw']
        self.cam_pitch = camera_cfg['pitch']
        self.cam_roll = camera_cfg.get('roll', 0)
        self.W = camera_cfg['width']
        self.H = camera_cfg['height']
        self.fov = camera_cfg['fov']
        self.aspect = self.W / self.H
        self.near = camera_cfg['near']
        self.far = camera_cfg['far']
    
    def update_position(self):
        target_pos, _ = p.getBasePositionAndOrientation(self.target_id)
        

        # cam_pos = [
        #     target[0] - self.cam_distance * math.cos(math.radians(self.cam_yaw)),
        #     target[1] - self.cam_distance * math.sin(math.radians(self.cam_yaw)),
        #     target[2] + 2.0
        # ]


        # Build view matrix
        # Use Bullet helper to compute a chase camera around the target
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=target_pos,
            distance=self.cam_distance,
            yaw=self.cam_yaw,
            pitch=self.cam_pitch,
            roll=self.cam_roll,
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
        renderer = p.ER_BULLET_HARDWARE_OPENGL if self.sim_mode == "GUI" else p.ER_TINY_RENDERER

        img = p.getCameraImage( 
            width=self.W,
            height=self.H,
            viewMatrix=view,
            projectionMatrix=proj,
            renderer=renderer
        )
        rgb = img[2]

        return rgb