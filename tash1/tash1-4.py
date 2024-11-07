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
objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0])  # 加载随机物体
target_orientation = p.getQuaternionFromEuler([math.pi, 0, math.pi/2])

robot_home = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
for i in range(7):
   p.resetJointState(pandaUid, i, robot_home[i])
# 设置一个定时器
Protime = time.time()

while True:
    current_time = time.time()

    if current_time < Protime+1:  # 一秒后
        # 开夹子
        p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.08)
        p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.08)

        jointIndices = [0, 1, 2, 3, 4, 5, 6]
        jointPoses = p.calculateInverseKinematics(pandaUid, 11, [0.7, 0, 0], targetOrientation=target_orientation[0:7])
        p.setJointMotorControlArray(pandaUid, jointIndices, p.POSITION_CONTROL, jointPoses[0:7])

    elif current_time < Protime+2:  # 两秒后
        # 关夹子
        p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, force=200)
        p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, force=200)

    elif current_time < Protime+3:  # 三秒后
        jointIndices = [0, 1, 2, 3, 4, 5, 6]
        jointPoses = p.calculateInverseKinematics(pandaUid, 11, [0.6, 0, 0.2], targetOrientation=target_orientation[0:7])
        p.setJointMotorControlArray(pandaUid, jointIndices, p.POSITION_CONTROL, jointPoses[0:7])

    elif current_time < Protime+4:  # 四秒后
        jointIndices = [0, 1, 2, 3, 4, 5, 6]
        time.sleep(1)
        jointPoses = p.calculateInverseKinematics(pandaUid, 11, [0.6, 0.5, 0.2], targetOrientation=target_orientation[0:7])
        p.setJointMotorControlArray(pandaUid, jointIndices, p.POSITION_CONTROL, jointPoses[0:7])

    p.stepSimulation()
    time.sleep(1 / 240.)  # 根据模拟的频率来休眠
