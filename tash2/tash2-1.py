import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
# FuncAnimation用于创建动画
# PillowWriter用于保存动画


def init():
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.grid(True)
    return arm1_line, arm2_line, arm3_line, endpoint


def update(frame):
    # 第二段臂的末端位置
    x2 = trajectory_x[frame]
    y2 = trajectory_y[frame]
    theta2 = trajectory_angle[frame]
    # 计算第三段臂的末端位置
    x3 = x2 + L3 * np.cos(theta2)
    y3 = y2 + L3 * np.sin(theta2)
    # 找到第一段臂的末端位置 (x1, y1)
    d = np.sqrt(x2 ** 2 + y2 ** 2)
    if d > (L1 + L2):  # 如果目标点超出臂长范围
        scale = (L1 + L2) / d
        x2 *= scale
        y2 *= scale
        d = L1 + L2  # 更新距离
    # 使用余弦定理计算各角度
    theta2 = np.arccos((L1 ** 2 + d ** 2 - L2 ** 2) / (2 * L1 * d))  # 第一段臂的角度
    theta1 = np.arctan2(y2, x2) - theta2
    # 第一段臂的末端位置
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    # 更新机械臂的图形对象
    arm1_line.set_data([0, x1], [0, y1])
    arm2_line.set_data([x1, x2], [y1, y2])
    arm3_line.set_data([x2, x3], [y2, y3])
    endpoint.set_data([x2], [y2])
    return arm1_line, arm2_line, arm3_line, endpoint


if __name__ == '__main__':
    # 机械臂参数
    L1 = 4  # 第一段臂的长度
    L2 = 3  # 第二段臂的长度
    L3 = 1  # 第三段臂的长度
    # 定义轨迹点和时间
    points = np.array([[-4, 0, 90], [0, 3, 45], [3, 3, 30], [4, 0, 0]])
    t_points = np.array([0, 2, 4, 7])
    # 时间参数
    num_points = 50
    t = np.linspace(0, 7, num_points)
    # 三次多项式拟合
    x_coeffs = np.polyfit(t_points, points[:, 0], 3)
    y_coeffs = np.polyfit(t_points, points[:, 1], 3)
    angle_coeffs = np.polyfit(t_points, points[:, 2], 3)
    # 生成轨迹
    trajectory_x = np.polyval(x_coeffs, t)
    trajectory_y = np.polyval(y_coeffs, t)
    trajectory_angle = np.deg2rad(np.polyval(angle_coeffs, t))
    # 创建画布
    fig, ax = plt.subplots()
    ax.plot(points[:, 0], points[:, 1], 'ro', markersize=10, markerfacecolor='r')  # 绘制轨迹点
    ax.plot(trajectory_x, trajectory_y, 'k--', linewidth=1.5)  # 绘制轨迹线
    arm1_line, = ax.plot([], [], 'r', linewidth=2)  # 第一段臂
    arm2_line, = ax.plot([], [], 'b', linewidth=2)  # 第二段臂
    arm3_line, = ax.plot([], [], 'g', linewidth=2)  # 第三段臂
    endpoint, = ax.plot([], [], 'ko', markersize=10, markerfacecolor='k')  # 末端
    # 动画
    ani = FuncAnimation(fig, update, frames=num_points, init_func=init, blit=True)
    plt.show()
