import pybullet as p # type: ignore
import pybullet_data
import time

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
time_step = 1.0 / 240.0
p.setTimeStep(time_step)

# camera settings
p.resetDebugVisualizerCamera(
    cameraDistance=3.0,
    cameraYaw=45,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0],
)

#load the floor
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# creating a cube of sid 0.5m 
half = 0.25  # for 0.5m cube
col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, half, half])
vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, half, half], rgbaColor=[0.0, 0.5, 0.5, 1.0])

cube_id = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=col,
    baseVisualShapeIndex=vis,
    basePosition=[0, 0, half],
    baseOrientation=[0, 0, 0, 1],
)


try:
    while p.isConnected():
        p.stepSimulation()
        time.sleep(time_step)
except KeyboardInterrupt:
    pass
finally:
    if p.isConnected():
        p.disconnect()