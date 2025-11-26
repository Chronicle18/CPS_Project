import pybullet as p
import pybullet_data
import numpy as np
import imageio

# -----------------------------
# Setup
# -----------------------------
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane = p.loadURDF("plane.urdf")
car = p.loadURDF("racecar/racecar.urdf", [0,0,0.2])

# -----------------------------
# Camera parameters
# -----------------------------
W, H = 640, 480
fov = 60
aspect = W / H
near = 0.1
far = 100

frames = []

for step in range(300):
    
    p.applyExternalForce(
        objectUniqueId=car,
        linkIndex=-1,
        forceObj=[5000, 0, 0],   # push car forward in +X
        posObj=[0,0,0],
        flags=p.LINK_FRAME
    )
    p.stepSimulation()

    # ---- Camera setup ----
    pos, _ = p.getBasePositionAndOrientation(car)

    cam_pos = [pos[0] - 5, pos[1], pos[2] + 2]
    cam_target = pos

    view = p.computeViewMatrix(
        cam_pos,
        cam_target,
        [0, 0, 1]
    )

    proj = p.computeProjectionMatrixFOV(
        fov, aspect, near, far
    )

    # ---- Get camera image ----
    width, height, rgbPixels, depthPixels, segPixels = p.getCameraImage(
        W, H, view, proj,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    # rgbPixels is a flat uint8 array in order RGBA
    frame = np.reshape(rgbPixels, (H, W, 4))[:, :, :3].astype(np.uint8)

    frames.append(frame)

# -----------------------------
# Save output
# -----------------------------
imageio.mimsave("render3d.gif", frames, fps=30)
print("Saved render3d.gif")