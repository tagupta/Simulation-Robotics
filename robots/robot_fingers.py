import pybullet as p # type: ignore
import pybullet_data
import numpy as np
import time

class ArmEnv():
    def __init__(self):
        self.state = self.init_state()
        self.step_count = 0

    def init_state(self):
        p.connect(p.GUI)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # so you’re not stuck at a bad default view.
        self._zoom_camera()
        p.setGravity(0, 0, -9.81)
        # Hide the extra RGB/depth preview panes so the 3D view stays large.
        # control extra preview panels in the PyBullet
        # p.configureDebugVisualizer(option, value)
        # 0 means OFF, 1 means ON
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        self.panda_id = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
        
        # for i in range(p.getNumJoints(self.panda_id)):
        #     print( "Here: ", i, p.getJointInfo(self.panda_id, i)[12])
    
        # getting the joint limits for the joint 4
        jointid = 4
        jlower = p.getJointInfo(self.panda_id, jointid)[8]
        jupper = p.getJointInfo(self.panda_id, jointid)[9]
        # getting the position of the finger
        finger_pos = p.getLinkState(self.panda_id, 9)[0]
        obs = np.array(finger_pos)
        return obs, 0, jlower, jupper

    def _zoom_camera(self):
        # Smaller cameraDistance = closer zoom. Aim at the arm, not the floor.
        p.resetDebugVisualizerCamera(
            cameraDistance=0.9,
            cameraYaw=45, # Orbit left/right (degrees)
            cameraPitch=-25, # Look up/down (negative = look down)
            cameraTargetPosition=[0.35, 0.0, 0.45],
        )

    def reset(self):
        p.disconnect()
        self.state = self.init_state()
        self.step_count = 0

    def step(self, new_position):
        self.step_count += 1
        start_position = p.getJointState(self.panda_id, 4)[0]
        # Interpolate so each command is a visible sweep, not a one-frame jump.
        for alpha in np.linspace(0.0, 1.0, 80):
            # Interpolate — 80 steps from start → new_position so motion is smooth, not a snap.
            interpolated = start_position + alpha * (new_position - start_position)
            p.setJointMotorControl2(
                self.panda_id,
                4,
                p.POSITION_CONTROL,
                targetPosition=interpolated,
                force=200,
                maxVelocity=0.6,
            )
            p.stepSimulation()
            self._zoom_camera()
            time.sleep(0.03)
        finger_pos = p.getLinkState(self.panda_id, 9)[0]

        # condition for termination
        if self.step_count >= 50:
            self.reset()
            reward = -1
            done = True
            return reward, done

        obs =  np.array(finger_pos)
        self.state = obs, self.state[1], self.state[2], self.state[3]
        done = False
        reward = -1
        return reward, done

# jointid = 4
# jlower = p.getJointInfo(targid, jointid)[8]
# jupper = p.getJointInfo(targid, jointid)[9]
# print("jlower:", jlower, "jupper:", jupper)

env = ArmEnv()
for step in range(100):
    new_position = np.random.uniform(env.state[2], env.state[3])
    reward, done = env.step(new_position)
    print(env.state)

# while p.isConnected():
#     new_position = np.random.uniform(env.state[2], env.state[3])
#     reward, done = env.step(new_position)
#     print(env.state)
#     p.stepSimulation()
#     time.sleep(1.0 / 240.0)

# p.disconnect()