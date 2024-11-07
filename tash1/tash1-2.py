import pybullet as p
import pybullet_data as pd
import math
import time

p.connect(p.GUI)  # 打开视窗
p.setGravity(0, 0, -10)  # 设置重力
p.setAdditionalSearchPath(pd.getDataPath())  # 设置数据路径
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.5, -0.5, 0.5])
# 设置相机参数
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)  # 加载机械臂
tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])  # 加载桌子
objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.1])  # 加载随机物体
time.sleep(3)

while True:
    jointIndices = [0, 1, 2, 3, 4, 5, 6]
    targetPositions = [0, math.pi / 4, 0, -math.pi / 2, 0, 3 * math.pi / 4, -math.pi / 4]

    p.setJointMotorControlArray(pandaUid, jointIndices, p.POSITION_CONTROL, targetPositions)
    p.stepSimulation()
