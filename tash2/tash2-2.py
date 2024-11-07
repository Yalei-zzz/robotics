import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # 曲线要经过的点坐标
    points_x = [0, 2, 4]
    points_y = [2, 3, 1]

    num_points = 50
    t1 = np.linspace(0, 2, num_points)
    t2 = np.linspace(2, 4, num_points)

    T = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                  [1, 2, 4, 8, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 1, 2, 4, 8],
                  [0, 1, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1, 4, 12],
                  [0, 1, 4, 12, 0, -1, 0, 0],
                  [0, 0, 2, 12, 0, 0, -2, 0]])

    T_inv = np.linalg.inv(T)

    Theta = np.array([[2], [3], [3], [1], [0], [0], [0], [0]])

    A = T_inv @ Theta

    a10_x = A[0, 0]
    a11_x = A[1, 0]
    a12_x = A[2, 0]
    a13_x = A[3, 0]
    a20_x = A[4, 0]
    a21_x = A[5, 0]
    a22_x = A[6, 0]
    a23_x = A[7, 0]
    print(a10_x, a11_x, a12_x, a13_x)

    trajectory1_x = a10_x + a11_x * t1 + a12_x * t1 ** 2 + a13_x * t1 ** 3
    trajectory2_x = a20_x + a21_x * t1 + a22_x * t1 ** 2 + a23_x * t1 ** 3

    plt.plot(points_x[:], points_y[:], 'ro', markersize=10, markerfacecolor='r')  # 绘制轨迹点

    plt.plot(t1, trajectory1_x, color='green', linestyle='--')
    plt.plot(t2, trajectory2_x, color='green', linestyle='--')
    plt.grid(True)  # 显示网格
    plt.show()
