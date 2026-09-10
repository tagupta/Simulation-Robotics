import pybullet as p # type: ignore
import time
import pybullet_data

physics_client = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,3]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)

# returns isConnected and connection method
print("Connection info:", p.getConnectionInfo(physics_client)) 

# while p.isConnected():
#     p.stepSimulation()
#     cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#     print("cubePos:", cubePos, "cubeOrn:", cubeOrn)
#     time.sleep(1.0 / 240.0)

p.disconnect()
