import cv2  as cv
import pybullet as p # type: ignore
import pybullet_data
import time
import numpy as np

# print("Hello World!!")
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# load assets
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
targid = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
obj_of_focus = targid

# while p.isConnected():
#     p.stepSimulation()
#     time.sleep(1.0 / 240.0)

# print("p.getNumJoints(targid):", p.getNumJoints(targid))
jointid = 4
jlower = p.getJointInfo(targid, jointid)[8]
jupper = p.getJointInfo(targid, jointid)[9]
print("jlower:", jlower, "jupper:", jupper)

# changing joint angles
for step in range(1000):
    joint_2_target_angle = np.random.uniform(jlower, jupper)
    joint_4_target_angle = np.random.uniform(jlower, jupper)
    p.setJointMotorControl2(targid, [2, 4], p.POSITION_CONTROL, targetPosition=[joint_2_target_angle, joint_4_target_angle])
    p.stepSimulation()
    time.sleep(0.01)

# for step in range(300):
#     focus_position, _ = p.getBasePositionAndOrientation(obj_of_focus)
#     p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=focus_position)
#     p.stepSimulation()
#     time.sleep(0.01)