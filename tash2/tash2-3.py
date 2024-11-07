import numpy as np
import matplotlib.pyplot as plt


def init():
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.grid(True)


if __name__ == '__main__':
    points_x = [-4, 0, 3, 4]
    points_y = [0, 3, 3, 0]
    angle = [90, 45, 30, 0]
    numPoints = 50
    t1 = np.linspace(0, 2, numPoints)
    t2 = np.linspace(0, 2, numPoints)
    t3 = np.linspace(0, 3, numPoints)
    # 变换矩阵（8，8）
    T = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [1, 2, 4, 8, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 2, 4, 8, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 9, 27],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 6, 27],
                  [0, 1, 4, 12, 0, -1, 0, 0, 0, 0, 0, 0],
                  [0, 0, 2, 12, 0, 0, -2, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1, 4, 12, 0, -1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 2, 12, 0, 0, -2, 0]])

    # 求逆矩阵
    T_inv = np.linalg.inv(T)

    # Theta角度
    Theta = np.array(
        [[-4, 0, 90], [0, 3, 45], [0, 3, 45], [3, 3, 30], [3, 3, 30], [4, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
         [0, 0, 0], [0, 0, 0], [0, 0, 0]])

    # 计算多项式系数
    A = T_inv @ Theta  # (8,1)
    print('A', A.shape)

    a10_x = A[0, 0],
    a10_y = A[0, 1],
    a10_angle = A[0, 2]
    a11_x = A[1, 0],
    a11_y = A[1, 1],
    a11_angle = A[1, 2]
    a12_x = A[2, 0],
    a12_y = A[2, 1],
    a12_angle = A[2, 2]
    a13_x = A[3, 0],
    a13_y = A[3, 1],
    a13_angle = A[3, 2]

    a20_x = A[4, 0],
    a20_y = A[4, 1],
    a20_angle = A[4, 2]
    a21_x = A[5, 0],
    a21_y = A[5, 1],
    a21_angle = A[5, 2]
    a22_x = A[6, 0],
    a22_y = A[6, 1],
    a22_angle = A[6, 2]
    a23_x = A[7, 0],
    a23_y = A[7, 1],
    a23_angle = A[7, 2]

    a30_x = A[8, 0],
    a30_y = A[8, 1],
    a30_angle = A[8, 2]
    a31_x = A[9, 0],
    a31_y = A[9, 1],
    a31_angle = A[9, 2]
    a32_x = A[10, 0],
    a32_y = A[10, 1],
    a32_angle = A[10, 2]
    a33_x = A[11, 0],
    a33_y = A[11, 1],
    a33_angle = A[11, 2]

    print(a30_x, a31_x, a32_x, a33_x)
    print(a30_y, a31_y, a32_y, a33_y)
    print(a30_angle, a31_angle, a32_angle, a33_angle)

    trajectory1_x = a10_x + a11_x * t1 + a12_x * t1 ** 2 + a13_x * t1 ** 3
    trajectory1_y = a10_y + a11_y * t1 + a12_y * t1 ** 2 + a13_y * t1 ** 3
    trajectory1_angle = a10_angle + a11_angle * t1 + a12_angle * t1 ** 2 + a13_angle * t1 ** 3

    trajectory2_x = a20_x + a21_x * t2 + a22_x * t2 ** 2 + a23_x * t2 ** 3
    trajectory2_y = a20_y + a21_y * t2 + a22_y * t2 ** 2 + a23_y * t2 ** 3
    trajectory2_angle = a20_angle + a21_angle * t1 + a22_angle * t1 ** 2 + a23_angle * t1 ** 3

    trajectory3_x = a30_x + a31_x * t3 + a32_x * t3 ** 2 + a33_x * t3 ** 3
    trajectory3_y = a30_y + a31_y * t3 + a32_y * t3 ** 2 + a33_y * t3 ** 3
    print('trajectory3_x', trajectory3_x)
    print('trajectory3_y', trajectory3_y)

    trajectory3_angle = a30_angle + a31_angle * t3 + a32_angle * t3 ** 2 + a33_angle * t3 ** 3

    fig, ax = plt.subplots()
    init()

    ax.plot(points_x, points_y, color='r', marker='o', linestyle='')
    ax.plot(trajectory1_x, trajectory1_y, color='green', linestyle='--')
    ax.plot(trajectory2_x, trajectory2_y, color='blue', linestyle='--')
    ax.plot(trajectory3_x, trajectory3_y, color='red', linestyle='--')

    plt.show()
