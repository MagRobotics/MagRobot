import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
dt = 0.01


def Hf(sen, m, n, p, x, y, z):
    x = x - sen[0]
    y = y - sen[1]
    z = z - sen[2]
    z0 = np.zeros((3, 1))
    z0[0] = (-3.0e-7 * m * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * x * (-9 * m * x - 9 * n * y - 9 * p * z)) / math.pow(
        (x ** 2 + y ** 2 + z ** 2), 2.5)
    z0[1] = (-3.0e-7 * n * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * y * (-9 * m * x - 9 * n * y - 9 * p * z)) / math.pow(
        (x ** 2 + y ** 2 + z ** 2), 2.5)
    z0[2] = (-3.0e-7 * p * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * z * (-9 * m * x - 9 * n * y - 9 * p * z)) / math.pow(
        (x ** 2 + y ** 2 + z ** 2), 2.5)
    return z0


def Jf(sen, m, n, p, x, y, z):
    x = x - sen[0]
    y = y - sen[1]
    z = z - sen[2]
    Hm = [[-5 * x * (-3.0e-7 * m * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * x * (-9 * m * x - 9 * n * y - 9 * p * z)) / (
            (x ** 2 + y ** 2 + z ** 2) * math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)) + (
                   1.2e-6 * m * x + 9.0e-7 * n * y + 9.0e-7 * p * z) / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2),
           -5 * y * (-3.0e-7 * m * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * x * (-9 * m * x - 9 * n * y - 9 * p * z)) / (
                   (x ** 2 + y ** 2 + z ** 2) * math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)) + (
                   -6.0e-7 * m * y + 9.0e-7 * n * x) / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2),
           -5 * z * (-3.0e-7 * m * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * x * (-9 * m * x - 9 * n * y - 9 * p * z)) / (
                   (x ** 2 + y ** 2 + z ** 2) * math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)) + (
                   -6.0e-7 * m * z + 9.0e-7 * p * x) / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2), 0, 0, 0,
           (6.0e-7 * x ** 2 - 3.0e-7 * y ** 2 - 3.0e-7 * z ** 2) / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2),
           9.0e-7 * x * y / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2),
           9.0e-7 * x * z / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)], [-5 * x * (
            -3.0e-7 * n * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * y * (-9 * m * x - 9 * n * y - 9 * p * z)) / ((
                                                                                                                     x ** 2 + y ** 2 + z ** 2) * math.pow(
        (x ** 2 + y ** 2 + z ** 2), 5 / 2)) + (9.0e-7 * m * y - 6.0e-7 * n * x) / math.pow((x ** 2 + y ** 2 + z ** 2),
                                                                                           5 / 2), -5 * y * (
                                                                                   -3.0e-7 * n * (
                                                                                   x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * y * (
                                                                                           -9 * m * x - 9 * n * y - 9 * p * z)) / (
                                                                                    (
                                                                                           x ** 2 + y ** 2 + z ** 2) * math.pow(
                                                                               (x ** 2 + y ** 2 + z ** 2),
                                                                               5 / 2)) + (
                                                                                   9.0e-7 * m * x + 1.2e-6 * n * y + 9.0e-7 * p * z) / math.pow(
        (x ** 2 + y ** 2 + z ** 2), 5 / 2), -5 * z * (-3.0e-7 * n * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * y * (
            -9 * m * x - 9 * n * y - 9 * p * z)) / ((x ** 2 + y ** 2 + z ** 2) * math.pow(
        (x ** 2 + y ** 2 + z ** 2), 5 / 2)) + (-6.0e-7 * n * z + 9.0e-7 * p * y) / math.pow((x ** 2 + y ** 2 + z ** 2),
                                                                                            5 / 2), 0, 0, 0,
                                                                           9.0e-7 * x * y / math.pow(
                                                                               (x ** 2 + y ** 2 + z ** 2), 5 / 2), (
                                                                                   -3.0e-7 * x ** 2 + 6.0e-7 * y ** 2 - 3.0e-7 * z ** 2) / math.pow(
            (x ** 2 + y ** 2 + z ** 2), 5 / 2), 9.0e-7 * y * z / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)], [
              -5 * x * (-3.0e-7 * p * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * z * (
                      -9 * m * x - 9 * n * y - 9 * p * z)) / (
                      (x ** 2 + y ** 2 + z ** 2) * math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)) + (
                      9.0e-7 * m * z - 6.0e-7 * p * x) / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2), -5 * y * (
                      -3.0e-7 * p * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * z * (
                      -9 * m * x - 9 * n * y - 9 * p * z)) / (
                      (x ** 2 + y ** 2 + z ** 2) * math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)) + (
                      9.0e-7 * n * z - 6.0e-7 * p * y) / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2), -5 * z * (
                      -3.0e-7 * p * (x ** 2 + y ** 2 + z ** 2) - 1.0e-7 * z * (
                      -9 * m * x - 9 * n * y - 9 * p * z)) / (
                      (x ** 2 + y ** 2 + z ** 2) * math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)) + (
                      9.0e-7 * m * x + 9.0e-7 * n * y + 1.2e-6 * p * z) / math.pow((x ** 2 + y ** 2 + z ** 2),
                                                                                   5 / 2), 0, 0, 0,
              9.0e-7 * x * z / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2),
              9.0e-7 * y * z / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2),
              (-3.0e-7 * x ** 2 - 3.0e-7 * y ** 2 + 6.0e-7 * z ** 2) / math.pow((x ** 2 + y ** 2 + z ** 2), 5 / 2)]]
    return Hm


def Getz(real, sensor):
    # z0 = Hf(sen0, real[3], real[4], real[5], real[0], real[1], real[2])
    # z1 = Hf(sen1, real[3], real[4], real[5], real[0], real[1], real[2])
    # z2 = Hf(sen2, real[3], real[4], real[5], real[0], real[1], real[2])
    # z3 = Hf(sen3, real[3], real[4], real[5], real[0], real[1], real[2])
    # z0 = np.mat(z0)
    # z1 = np.mat(z1)
    # z2 = np.mat(z2)
    # z3 = np.mat(z3)
    # z = np.vstack((z0, z1, z2, z3))
    z = np.array([[0]])
    R_sensor = np.zeros((len(sensor), 3, 3))
    for i in range(len(sensor)):
        z_temp = Hf(sensor[i]["pose_sensor"], real[3], real[4], real[5], real[0], real[1], real[2])
        # print("-----------------------------")
        # print(z_temp)

        quat2 = sensor[i]["pose_sensor"][3:7]
        R_sensor[i] = R.from_quat(quat2).as_matrix()
        z_temp = R_sensor[i].transpose() @ z_temp
        # print(z_temp)

        temp = np.random.normal(0, sensor[i]["sensitivity"], (3, 1))
        z_single = z_temp + temp
        z_single = R_sensor[i] @ z_single

        z = np.vstack((z, z_single))

    z = z[1:]
    # print(z)

    # noise = np.array([[0]])
    # for i in range(len(sensor)):
    #     temp = np.random.normal(0, sensor[i]["sensitivity"], (3, 1))
    #     noise = np.vstack((noise, temp))
    # # noise = noise[1:]
    # z = z[1:] + noise[1:]
    # print(z)
    #
    # z = R_sensor[i] @ z
    # print(z)
    #磁场×旋转矩阵的逆再加噪声，再×旋转矩阵
    return z


def KalmanFilter(z, pos, P, sensor, EKF_Q, EKF_R):
    # pos=np.mat(pos)
    # pos=np.transpose(pos)#9x1
    '''

    Hm
    Getz
    zp

    '''
    Q = np.mat(EKF_Q)
    # print(Q)
    # print(type(EKF_Q))
    # print(pos.shape)
    # Q = np.mat([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
    # print(Q)
    # R = 1e-20 * np.diag(np.ones(12))
    R = 1e-15 * np.array(EKF_R)
    # print(type(R))
    # A = np.mat([[np.eye(3),dt*np.eye(3),np.zeros((3,3))],[np.zeros(3),np.eye(3),np.zeros((3,3))],[np.zeros((3,6)),np.eye(3)]])
    A = np.mat([[1, 0, 0, 0.01, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0.01, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0.01, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1]])

    # W =[[(dt**2)/2*np.eye(3),np.zeros((3,3))],[dt*np.eye(3),np.zeros((3,3))],[0, 0, 0, 0, dt*pos[8], -dt*pos[7]],[0, 0, 0, -dt*pos[8], 0, dt*pos[6]],[0, 0, 0, dt*pos[7], -dt*pos[6], 0]]
    W = np.mat([[dt ** 2, 0, 0, 0, 0, 0],
                [0, dt ** 2, 0, 0, 0, 0],
                [0, 0, dt ** 2, 0, 0, 0],
                [dt, 0, 0, 0, 0, 0],
                [0, dt, 0, 0, 0, 0],
                [0, 0, dt, 0, 0, 0],
                [0, 0, 0, 0, dt * pos[8,0], -dt * pos[7,0]],
                [0, 0, 0, -dt * pos[8,0], 0, dt * pos[6,0]],
                [0, 0, 0, dt * pos[7,0], -dt * pos[6,0], 0]])
    # print(W)
    xp = A * pos  # 预测方程式线性的
    Pp = A * P * np.transpose(A) + W * Q * np.transpose(W)
    # zp0 = Hf(sen0, xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
    # zp1 = Hf(sen1, xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
    # zp2 = Hf(sen2, xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
    # zp3 = Hf(sen3, xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
    # zp = np.vstack((zp0, zp1, zp2, zp3))

    zp = np.array([[0]])
    for i in range(len(sensor)):
        z_temp = Hf(sensor[i]["pose_sensor"], xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
        zp = np.vstack((zp, z_temp))
    zp = zp[1:]

    # print(zp.shape) (12,1)
    # Hm0 = Jf(sen0, xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
    # Hm1 = Jf(sen1, xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
    # Hm2 = Jf(sen2, xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
    # Hm3 = Jf(sen3, xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
    # Hm = np.vstack((Hm0, Hm1, Hm2, Hm3))

    Hm = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0]])
    for i in range(len(sensor)):
        H_temp = Jf(sensor[i]["pose_sensor"], xp[6, 0], xp[7, 0], xp[8, 0], xp[0, 0], xp[1, 0], xp[2, 0])
        Hm = np.vstack((Hm, H_temp))
    Hm = Hm[1:, :]

    # print(Hm.size)

    # print(Hm.shape) (12,9)
    # Pp=np.mat(Pp)
    # Hm=np.mat(Hm)
    fenmu = np.mat(Hm * Pp * np.transpose(Hm) + R)
    # print(fenmu)
    fenmu = fenmu.astype(np.float64)
    fenmu = np.linalg.inv(fenmu)
    K = Pp * np.transpose(Hm) * fenmu
    # print(K)
    # K=fenmu*Pp*np.transpose(Hm)
    # K=Pp*np.transpose(Hm)/(Hm*Pp*np.transpose(Hm) + R)
    # K=np.linalg.pinv(Hm*Pp*np.transpose(Hm) + R)*Pp*np.transpose(Hm)
    # K=np.mat(K)
    # print(K)

    # zp = np.mat(zp)

    # z=np.mat(z)
    pos = xp + K * (z - zp)  # K为9行3列，需要乘积为9×1
    # print(pos)
    # pos=np.array(pos)
    # print(z.shape)
    P = Pp - K * Hm * Pp  # 更新误差协方差矩阵
    return pos, P


pc = np.array([-0.075, -0.47, 0.001, 2.0, 2.0, 8.0])
# mc = np.array([[0], [0], [1]])
t = 0
i = 0
pos = np.zeros((9, 1))

t_l = []
z_l = []
alpha = 1

if __name__ == "__main__":
    sen0 = np.array([[-0.075, -0.47, 0.05], [0, 0, 0], [1, 0, 0]])
    sen1 = np.array([[-0.075, -0.42, 0.05], [0, 0, 0], [1, 0, 0]])
    sen2 = np.array([[-0.025, -0.47, 0.05], [0, 0, 0], [1, 0, 0]])
    sen3 = np.array([[-0.025, -0.42, 0.05], [0, 0, 0], [1, 0, 0]])
    while (t < 0.5):
        pc = pc + np.array([0.0, 0, 0.002, 0.0, 0, 0])
        z = Getz(pc, sen0, sen1, sen2, sen3)
        t_l.append(t)
        if i == 0:
            pos = np.mat([[-0.075], [-0.47], [0.001], [0], [0], [0], [2.0], [2.0], [8.0]])  # 9x1
            P = np.zeros((9, 9))
            i = i + 1
            z_l.append(pos[2, 0])
        else:
            x_new, P = KalmanFilter(z, pos, P, sen0, sen1, sen2, sen3)
            pos = x_new.astype(np.float)
            z_l.append(pos[2, 0] * alpha + z_l[-1] * (1 - alpha))

        #z_l.append(pos[2, 0])

        # 得到 pc_estimation， mc_estimation
        pc_estimation = np.array([pos[0], pos[1], pos[2]])
        mc_estimation = np.array([pos[6], pos[7], pos[8]])
        # print(pos)

        t += 0.01
