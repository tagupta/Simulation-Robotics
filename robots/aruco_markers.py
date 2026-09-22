import pybullet as p # type: ignore
import pybullet_data
import time
import numpy as np
import cv2  as cv

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
    cameraTargetPosition=[0, 0, 0.1],
)

# load the plane
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# load the box with aruco marker
p.loadURDF("../URDFS/ar_marker_box.urdf", [0, 0, 0.25], p.getQuaternionFromEuler([1.57, 0, 0]),useFixedBase=True)

# setting simulated camera view matrix using yaw, pitch, and roll
# view_matrix = p.computeViewMatrixFromYawPitchRoll(
#     cameraTargetPosition=[0, 0, 0.1],
#     distance=3.0,
#     yaw=45,
#     pitch=-35,
#     roll=0,
#     upAxisIndex=2,
# )

# projection matrix
# width, height = 640, 480
# projection_matrix = p.computeProjectionMatrixFOV(
#     fov=60,
#     aspect=width / height,
#     nearPlane=0.1,
#     farPlane=100.0,
# )

# result = p.getCameraImage(
#     width,
#     height,
#     view_matrix,
#     projection_matrix,
#     renderer=p.ER_BULLET_HARDWARE_OPENGL,
# )
# result: (width, height, rgbaPixels, depth, segmentation)
# rgba = result[2]

# frame = np.array(rgba, dtype=np.uint8).reshape(height, width, 4)
# image = cv.cvtColor(frame, cv.COLOR_RGBA2BGR)

# cv.imshow("image", image)
# cv.waitKey(1)

try:
    while p.isConnected():
        p.stepSimulation()
        time.sleep(time_step)
except KeyboardInterrupt:
    pass
finally:
    if p.isConnected():
        p.disconnect()