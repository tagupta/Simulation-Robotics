import pybullet as p # type: ignore
import pybullet_data
import time
# import numpy as np

# connecting to server => GUI, DIRECT, SHARED_MEMORY, UDP, TCP
p.connect(p.GUI)
p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
time_step = 1.0 / 240.0
p.setTimeStep(time_step)

#load the floor
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
# spawn the cube on the floor, not half buried
cube_id = p.loadURDF("cube.urdf", [0, 0, 0.25], [0, 0, 0, 1], globalScaling=0.5)
p.changeVisualShape(cube_id, -1, rgbaColor=[1, 0.5, 0.1, 1])


# Smaller cameraDistance = more zoomed in. You can still scroll in the window.
p.resetDebugVisualizerCamera(
    cameraDistance=3.0,
    cameraYaw=45,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0],
)

p.resetBaseVelocity(
    cube_id,
    linearVelocity=[3, 0, 0],      # forward
    angularVelocity=[0, 0, 0],     # no roll-like spin
)

try:
    step_count = 0
    while p.isConnected():
        if step_count < 50:  # 480 steps ≈ 2 s at 240 Hz
            p.applyExternalForce(cube_id, -1, [4, 0, 0], [0, 0, 0], p.WORLD_FRAME)
            # p.applyExternalForce(cube_id, -1, [10, 0, 0], [0, 0, 0], p.WORLD_FRAME)
        p.stepSimulation()
        time.sleep(time_step)
        step_count += 1
except KeyboardInterrupt:
    pass
finally:
    if p.isConnected():
        p.disconnect()
