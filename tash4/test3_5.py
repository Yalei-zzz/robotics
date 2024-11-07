import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os
import cv2
# 初始化PyBullet环境
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 加载平面和物体
plane_id = p.loadURDF("plane.urdf")
pandaUid = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0.72], useFixedBase=True)  # 加载机械臂

table_id = p.loadURDF('table/table.urdf',
                      [0.0, -0.65, 0.1],
                       p.getQuaternionFromEuler([0, 0, 0]),
                       useFixedBase=True)
table_id = p.loadURDF('table/table.urdf',
                      [0.0, 0.35, 0.1],
                       p.getQuaternionFromEuler([0, 0, 0]),
                       useFixedBase=True)
table_id = p.loadURDF('table/table.urdf',
                      [1.5, 0.35, 0.1],
                       p.getQuaternionFromEuler([0, 0, 0]),
                       useFixedBase=True)
table_id = p.loadURDF('table/table.urdf',
                      [1.5, -0.65, 0.1],
                       p.getQuaternionFromEuler([0, 0, 0]),
                       useFixedBase=True)
object1Uid = p.loadURDF("cube_small.urdf", basePosition=[0., -0.65, 0.72])
object1Uid = p.loadURDF("cube_small.urdf", basePosition=[0.65, 0.3, 0.72])
class Camera:
    def __init__(self, cam_pos, cam_target, near, far, size, fov):
        self.x, self.y, self.z = cam_pos
        self.x_t, self.y_t, self.z_t = cam_target
        self.width, self.height = size
        self.near, self.far = near, far
        self.fov = fov

        aspect = self.width / self.height
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov, aspect, near, far)
        self.view_matrix = p.computeViewMatrix(cam_pos, cam_target, [0, 1, 0])

    def get_cam_img(self):
        _w, _h, rgb, depth, seg = p.getCameraImage(self.width, self.height,
                                                   self.view_matrix, self.projection_matrix,
                                                   )
        return rgb[:, :, 0:3], depth, seg, self.projection_matrix


def get_transform_matrix(x, y, z, rot):
    return np.array([
                    [1,             0,              0,  x],
                    [0,   np.cos(rot),    np.sin(rot),  y],
                    [0,  -np.sin(rot),    np.cos(rot),  z],
                    [0,             0,              0,  1]
                    ])


def detect_objects(image, thresh_min, thresh_max):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray_image, thresh_min, thresh_max, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for cnt in contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append((cX, cY))
    return centers


if __name__ == '__main__':
    # 通过相机捕获物体坐标，通过坐标变换实现抓取定位
    center_x, center_y = 0.52, 0

    pos_z = 1.87
    target_z = 0.785

    img_width = 224
    camera_rotaion = np.pi

    near = 0.2
    far = 2.0
    width = 224
    height = 224
    fov = 40
    thresh_min = 230
    thresh_max = 255
    camera = Camera((center_x, center_y, pos_z), (center_x,
                    center_y, target_z), near, far, (width, height), fov)
    # 设置末端方向，注意是与世界坐标系的旋转
    target_orientation = p.getQuaternionFromEuler([np.pi, 0, np.pi / 2])

    # while True:
    rgb_img, depth_img, _, projection_matrix = camera.get_cam_img()
    fx = projection_matrix[0] * width / 2
    fy = projection_matrix[5] * height / 2
    cx = width /2
    cy = height / 2
    centers = detect_objects(rgb_img, thresh_min, thresh_max)
    if centers:
        for center in centers:
            x_p, y_p = center
            z_p = depth_img[x_p,y_p]
            print(f"检测到的物体中心点 ({x_p}, {y_p}) 的深度缓冲值: {z_p}")

    z_p = far * near / (far - (far - near) * z_p)

    x_camera = (x_p - cx) * z_p / fx
    y_camera = (y_p - cy) * z_p / fy
    z_camera = z_p
    print("x_camera, y_camera, z_camera", x_camera, y_camera, z_camera)

    # Convert image space to camera's 3D space
    img_xyz = np.array([x_camera, y_camera, z_camera, 1])
    cam_space = img_xyz
    print('cam_space', cam_space)

    cam_to_robot_base = get_transform_matrix(camera.x,
                                             camera.y,
                                             camera.z,
                                             camera_rotaion)
    # Convert camera's 3D space to robot frame of reference
    robot_frame_ref = np.matmul(cam_to_robot_base, cam_space)
    print('robot_frame_ref', robot_frame_ref)
    # 机器人移动前需要加上，保证末端位于前方并指向正下方
    robot_home = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
    for i in range(7):
        p.resetJointState(pandaUid, i, robot_home[i])
    p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, 0.08)
    p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, 0.08)
    targetPosition = [robot_frame_ref[0], robot_frame_ref[1], robot_frame_ref[2]+0.1]

    while True:
        # 逆运动学实验
        jointIndices = [0, 1, 2, 3, 4, 5, 6]
        jointPoses = p.calculateInverseKinematics(pandaUid,
                                                  11,
                                                  targetPosition,
                                                  targetOrientation=target_orientation)[0:7]
        p.setJointMotorControlArray(pandaUid, jointIndices, p.POSITION_CONTROL, list(jointPoses))

        p.stepSimulation()

