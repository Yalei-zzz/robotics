import pybullet as p
import pybullet_data as pd
import math
import time

p.connect(p.GUI)  # 连接到pybullet的GUI服务器
p.setGravity(0, 0, -10)  # 设置重力
p.setAdditionalSearchPath(pd.getDataPath())  # 设置数据路径
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.5, -0.5, 0.5])  # 设置相机参数

pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)  # 加载机械臂
tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])  # 加载桌子
objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0])  # 加载随机物体
target_orientation = p.getQuaternionFromEuler([math.pi, math.pi/2, math.pi/2])  # 目标姿态

robot_home = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
for i in range(7):
   p.resetJointState(pandaUid, i, robot_home[i])  # 设置机械臂的初始关节角度
#
# Protime = time.time()  # 设置一个定时器
targetPosition = [0.7, 0, 0.2]
while True:
    jointIndices = [0, 1, 2, 3, 4, 5, 6]
    # 计算逆运动学，使得机械臂末端执行器到达目标位置和姿态
    jointPoses = p.calculateInverseKinematics(pandaUid, 11, targetPosition, targetOrientation=target_orientation)
    jointPoses = jointPoses[0:7]
    p.setJointMotorControlArray(pandaUid, jointIndices, p.POSITION_CONTROL, jointPoses)
    p.stepSimulation()
