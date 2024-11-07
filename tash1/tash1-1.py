import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


# 初始化绘制


def init():
    ax.quiver(*origin, *x_axis_initial, linestyle='dashed', color='r', label='Initial X axis')
    ax.text(1.3, 0, 0, 'X', color='r')
    ax.quiver(*origin, *y_axis_initial, linestyle='dashed', color='g', label='Initial Y axis')
    ax.text(0, 1.3, 0, 'Y', color='g')
    ax.quiver(*origin, *z_axis_initial, linestyle='dashed', color='b', label='Initial Z axis')
    ax.text(0, 0, 1.3, 'Z', color='b')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    # 隐藏坐标轴刻度和标签（可选）
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_xlabel('Initial X')
    ax.set_ylabel('Initial Y')
    ax.set_zlabel('Initial Z')
    ax.legend()
    ax.view_init(elev=20, azim=30)  # 调整视角
    return ax

# 更新函数


def update(frame):
    global x_axis_initial, y_axis_initial, z_axis_initial
    if frame < 30:  # 不旋转
        rotation_matrix = R
    elif frame < 60:  # 对 X 轴旋转
        rotation_matrix = Ry
    # 从初始状态开始旋转
    else:  # 对 Y 轴旋转
        rotation_matrix = Rx @ Ry
    # 从初始状态开始旋转
    x_axis_rotated = rotation_matrix @ x_axis_initial
    y_axis_rotated = rotation_matrix @ y_axis_initial
    z_axis_rotated = rotation_matrix @ z_axis_initial
    # 更新绘制
    ax.cla()
    # 绘制初始标准直角坐标系
    ax.quiver(*origin, *x_axis_initial, linestyle='dashed', color='r', label='Initial X axis')
    ax.quiver(*origin, *y_axis_initial, linestyle='dashed', color='g', label='Initial Y axis')
    ax.quiver(*origin, *z_axis_initial, linestyle='dashed', color='b', label='Initial Z axis')
    # 绘制旋转后的坐标系
    ax.quiver(*origin, *x_axis_rotated, color='r', label='X axis')
    ax.text(1.3, 0, 0, 'X', color='r')
    ax.quiver(*origin, *y_axis_rotated, color='g', label='Y axis')
    ax.text(0, 1.3, 0, 'Y', color='g')
    ax.quiver(*origin, *z_axis_rotated, color='b', label='Z axis')
    ax.text(0, 0, 1.3, 'Z', color='b')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    # 隐藏坐标轴刻度和标签（可选）
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return ax


if __name__ == '__main__':
    # 创建图形和坐标轴
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_zlim(-1.5, 1.5)
    # 初始坐标轴向量
    origin = np.array([0, 0, 0])
    x_axis_initial = np.array([1, 0, 0])
    y_axis_initial = np.array([0, 1, 0])
    z_axis_initial = np.array([0, 0, 1])
    # 角度转弧度
    angle_x = np.deg2rad(60)
    angle_y = np.deg2rad(30)
    R = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])
    # X 轴旋转矩阵 (Roll)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(angle_x), -np.sin(angle_x)],
                   [0, np.sin(angle_x), np.cos(angle_x)]])
    # Y 轴旋转矩阵 (Pitch)
    Ry = np.array([[np.cos(angle_y), 0, np.sin(angle_y)],
                   [0, 1, 0],
                   [-np.sin(angle_y), 0, np.cos(angle_y)]])
    # 动画
    ani = FuncAnimation(fig, update, frames=90, init_func=init, interval=100)
    ani.save('3d_axis_rotation_.gif', writer = PillowWriter(fps=20))
    # 显示动画
    plt.show()
