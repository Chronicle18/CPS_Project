import pybullet as p
import pybullet_data
import time
import numpy as np
import imageio
import cv2

# -------------------------------
# SETUP
# -------------------------------
p.connect(p.GUI)   # GUI mode (macOS OK)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

# Ground
plane = p.loadURDF("plane.urdf")

# -------------------------------
# RAMP
# -------------------------------
ramp_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2,2,0.2])
ramp_body = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=ramp_col,
    basePosition=[0,0,0.2],
    baseOrientation=p.getQuaternionFromEuler([0, -0.45, 0])  # incline
)

# -------------------------------
# CAR BODY
# -------------------------------
# car_mass = 100
# car_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1,0.5,0.3])
# car = p.createMultiBody(
#     car_mass,
#     car_col,
#     basePosition=[0, -8, 0],
#     baseOrientation=p.getQuaternionFromEuler([0,0,90*np.pi/180])
# )

car = p.loadURDF("racecar/racecar.urdf", basePosition=[-8,0,0.2], globalScaling=1.5)

# -------------------------------
# HELPER: get car state
# -------------------------------
def get_car_pose():
    pos, orn = p.getBasePositionAndOrientation(car)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    return pos, (roll, pitch, yaw)

"""# -------------------------------
# INTERNAL MOVING CUBE
# -------------------------------
cube_offset = [0, 0, 0.6]  # initial offset relative to car
cube_mass = 5      # small enough to not destabilize on ground
cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,0.2,0.2])
car_x, car_y, car_z = get_car_pose()[0]
cube = p.createMultiBody(
    cube_mass,
    cube_col,
    basePosition=[car_x + cube_offset[0], car_y + cube_offset[1], car_z + cube_offset[2]] #[0, -8, 0.6]
)

# disable damping so cube follows control exactly
p.changeDynamics(cube, -1, linearDamping=0, angularDamping=0)


# create a fixed constraint at initial offset
cube_constraint = p.createConstraint(
    parentBodyUniqueId=car,
    parentLinkIndex=-1,
    childBodyUniqueId=cube,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,   # physically attached
    jointAxis=[0,0,0],
    parentFramePosition=cube_offset,
    childFramePosition=[0,0,0]
)"""

# -------------------------------
# CAMERA CONFIG (3D RENDER)
# -------------------------------
W, H = 640, 480
frames = []


# -------------------------------
# PID controller for cube
# -------------------------------
Kp, Ki, Kd = 30, 0, 6
prev_err = 0
integral = 0

def pid(target, current, dt):
    global prev_err, integral
    err = target - current
    integral += err * dt
    deriv = (err - prev_err) / dt
    prev_err = err
    return Kp*err + Ki*integral + Kd*deriv


# def move_cube_relative(dx=0, dy=0):
#     """
#     Move cube relative to the car along its local XY plane.
#     dx, dy = offset in meters
#     """
#     global cube_offset
#     # Update offset
#     cube_offset[0] += dx
#     cube_offset[1] += dy
    
#     # Optionally clip to car dimensions
#     cube_offset[0] = np.clip(cube_offset[0], -1, 1)  # car length ±1
#     cube_offset[1] = np.clip(cube_offset[1], -0.5, 0.5)  # car width ±0.5

#     # Update constraint frame
#     p.changeConstraint(
#         cube_constraint,
#         jointChildPivot=[0,0,0],
#         jointParentFramePosition=cube_offset,
#         maxForce=500
#     )


# -------------------------------
# MAIN LOOP
# -------------------------------
dt = 1/240
jump_force_applied = False

for step in range(1500):

    # ----- Apply forward force (WORLD frame) -----
    # p.applyExternalForce(
    #     car, -1,
    #     [8000, 0, 0],
    #     [0,0,0],
    #     p.WORLD_FRAME
    # )

    # ----- Car pose -----
    (x,y,z), (roll, pitch, yaw) = get_car_pose()

    # detect if car is airborne
    grounded = len(p.getContactPoints(car, plane)) > 0

    # ----- Control cube -----
    # if grounded:
    #     # keep cube centered on ground
    #     p.resetBasePositionAndOrientation(
    #         cube,
    #         [x, y, z + 0.6],
    #         [0,0,0,1]
    #     )
    # else:
    #     # apply flipping torque by sliding cube sideways
    #     u = pid(0, pitch, dt)
    #     cube_y = np.clip(u * 0.03, -0.6, 0.6)
    #     p.resetBasePositionAndOrientation(
    #         cube,
    #         [x, y + cube_y, z + 0.6],
    #         [0,0,0,1]
    #     )

    # ----- Step simulation -----
    p.stepSimulation()

    # ----- Camera tracking -----
    cam_target = [x, y, z]
    cam_pos = [x - 6, y + 2, z + 2]

    view = p.computeViewMatrix(
        cameraEyePosition=cam_pos,
        cameraTargetPosition=cam_target,
        cameraUpVector=[0,0,1]
    )
    proj = p.computeProjectionMatrixFOV(
        fov=70, aspect=W/H, nearVal=0.1, farVal=100
    )

    _,_, rgb, _, _ = p.getCameraImage(
        W, H, view, proj
    )

    frame = np.reshape(rgb, (H, W, 4))[:,:,:3]
    frames.append(frame)

# -------------------------------
# SAVE VIDEO
# -------------------------------
writer = imageio.get_writer("car_flip.mp4", fps=30)
for f in frames:
    writer.append_data(f)
writer.close()

print("Saved video: car_flip.mp4")