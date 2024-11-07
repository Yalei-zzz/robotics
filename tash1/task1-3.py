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
objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.75, 0, 0.1])  # 加载随机物体
Protime = time.time()

while True:
    current_time = time.time()
    if current_time < Protime + 1:
        # 开夹子
        p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.08)
        p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.08)
    elif current_time < Protime + 2:
        jointIndices = [0, 1, 2, 3, 4, 5, 6]
        targetPositions = [0, math.pi / 3, 0, -5 * math.pi / 12, 0, 3 * math.pi / 4, -50 * math.pi / 180]
        p.setJointMotorControlArray(pandaUid, jointIndices, p.POSITION_CONTROL, targetPositions)
    elif current_time < Protime + 3:
        #  关夹子
        p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, force=200)
        p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, force=200)
    # elif current_time < Protime + 3:
    elif current_time < Protime + 4:
        jointIndices = [0, 1, 2, 3, 4, 5, 6]
        targetPositions = [0, math.pi / 4, 0, -5 * math.pi / 12, 0, 3 * math.pi / 4, -50 * math.pi / 180]
        p.setJointMotorControlArray(pandaUid, jointIndices, p.POSITION_CONTROL, targetPositions)
    elif current_time < Protime + 5:
        p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.08)
        p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.08)
    p.stepSimulation()
    time.sleep(1 / 240.)  # 根据模拟的频率来休眠
