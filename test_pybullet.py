import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)  # GUI mode

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
cube = p.loadURDF("r2d2.urdf")

p.setGravity(0, 0, -9.8)

for i in range(500):
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()
