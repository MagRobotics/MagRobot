import numpy as np
from scipy.optimize import fsolve
from Pose_Transform.pose_transform import *

class Permanent:
    def __init__(self, p_permanent, m_permanent, num):
        """
        :param p_electromagnet: 多个电磁铁位置的矩阵 array 3*n
        :param m_hat_electromagnet: 多个电磁铁姿态的矩阵 array 3*n
        :param coef: 多个电磁铁系数的array（磁矩大小等于系数乘以电流）array n*1
        :param num: 电磁铁个数 float
        """
        self.p_permanent = p_permanent
        self.m_permanent = m_permanent
        self.permanent_num = num


    def field_permanent(self, pa, ma, pc):
        """
        计算永磁铁在空间中某位置产生的磁场和磁场梯度矩阵
        :param pa: 永磁铁的位置 array 3*1
        :param pc: 空间中某点的位置 array 3*1
        :param ma: 永磁铁的磁矩 array 3*1
        :return: 磁场 field array 3*1
        """
        k = 4 * np.pi * 1e-7
        ma = np.mat(ma)
        p = np.mat(pc - pa)

        # 磁源在某点产生的磁场强度
        field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

        return field


    def gradient_permanent(self, pa, ma, pc):
        """
        计算永磁铁在空间中某位置产生的磁场和磁场梯度矩阵
        :param pa: 永磁铁的位置 array 3*1
        :param pc: 空间中某点的位置 array 3*1
        :param ma: 永磁铁的磁矩 array 3*1
        :return: 磁场梯度矩阵 gradient 3*3
        """
        k = 4 * np.pi * 1e-7
        ma = np.mat(ma)
        p = np.mat(pc - pa)
        p_hat = p / np.linalg.norm(p)

        # 磁源在某点产生的磁场梯度矩阵
        gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))

        return gradient


    def moment_permanent(self, pa, ma, pc, mc):
        """
        计算永磁铁在空间中某位置对被驱动永磁铁产生的磁力和磁力矩
        :param pa: 永磁铁的位置 array 3*1
        :param pc: 空间中被驱动永磁铁的位置 array 3*1
        :param ma: 永磁铁的磁矩 array 3*1
        :param mc: 被驱动磁铁的磁矩 array 3*1
        :return: 磁力 force 3*1 和 磁力矩 moment 3*1
        """
        field = self.field_permanent(pa, ma, pc)
        S_mc = np.mat([[0, -mc[2, 0], mc[1, 0]],
                       [mc[2, 0], 0, -mc[0, 0]],
                       [-mc[1, 0], mc[0, 0], 0]])
        moment = S_mc * field

        return moment


    def force_permanent(self, pa, ma, pc, mc):
        """
        计算永磁铁在空间中某位置对被驱动永磁铁产生的磁力和磁力矩
        :param pa: 永磁铁的位置 array 3*1
        :param pc: 空间中被驱动永磁铁的位置 array 3*1
        :param ma: 永磁铁的磁矩 array 3*1
        :param mc: 被驱动磁铁的磁矩 array 3*1
        :return: 磁力 force 3*1 和 磁力矩 moment 3*1
        """
        gradient = self.gradient_permanent(pa, ma, pc)
        force = gradient * mc

        return force


class Electromagnet:
    def __init__(self, p_electromagnet, m_hat_electromagnet, coef, num, current):
        """
        :param p_electromagnet: 多个电磁铁位置的矩阵 array 3*n
        :param m_hat_electromagnet: 多个电磁铁姿态的矩阵 array 3*n
        :param coef: 多个电磁铁系数的array（磁矩大小等于系数乘以电流）array n*1
        :param num: 电磁铁个数 float
        """
        self.p_electromagnet = np.mat(p_electromagnet)
        self.m_electromagnet_hat = np.mat(m_hat_electromagnet)
        self.coef = np.mat(coef)
        self.num = num
        self.current = current

    def skew_matrix(self, a):
        """
        将向量转成斜矩阵, 用于力矩计算
        :param a: 向量 array 3*1
        :return: 斜矩阵 array 3*3
        """
        S = np.zeros((3, 3))
        S[0, 0] = 0
        S[1, 1] = 0
        S[2, 2] = 0
        S[0, 1] = -a[2, 0]
        S[1, 0] = a[2, 0]
        S[0, 2] = a[1, 0]
        S[2, 0] = -a[1, 0]
        S[1, 2] = -a[0, 0]
        S[2, 1] = a[0, 0]

        return np.mat(S)

    def m_matrix(self, m):
        """
        将向量转成M矩阵，用于磁力计算
        :param m: 输入向量 array 3*1
        :return: m矩阵 array 3*5
        """
        M = np.zeros((3, 5))
        M[0, 0] = m[0, 0]
        M[1, 0] = 0
        M[2, 0] = -m[2, 0]

        M[0, 1] = m[1, 0]
        M[1, 1] = m[0, 0]
        M[2, 1] = 0

        M[0, 2] = m[2, 0]
        M[1, 2] = 0
        M[2, 2] = m[0, 0]

        M[0, 3] = 0
        M[1, 3] = m[1, 0]
        M[2, 3] = -m[2, 0]

        M[0, 4] = 0
        M[1, 4] = m[2, 0]
        M[2, 4] = m[1, 0]

        return np.mat(M)

    def field_electromagnet(self, pc, current):
        """
        计算多个电磁铁在某点的磁场和磁场梯度(先算八个分量的矩阵，再乘上电流)
        :param pc: 目标位置 array 3*1
        :param current: 目标位置 array n*1
        :return: 八个关于磁场强度的向量 np.array 3*1
        """
        pc = np.mat(pc)
        current = np.mat(current)
        F = np.zeros((3, self.num))
        for i in range(self.num):
            k = 4 * np.pi * 1e-7
            p = pc - self.p_electromagnet[0:3, i]
            ma = self.m_electromagnet_hat[0:3, i] * self.coef[i, 0]

            # 磁源在某点产生的磁场强度
            field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma
            F[0:3, i:i + 1] = field
        field = F * current

        return field

    def gradient_electromagnet(self, pc, current):
        """
        计算多个电磁铁在某点的磁场和磁场梯度(先算八个分量的矩阵，再乘上电流)
        :param pc: 目标位置 array 3*1
        :param current: 目标位置 array n*1
        :return: 五个关于磁场的向量 np.array 5*1
        """
        pc = np.mat(pc)
        current = np.mat(current)
        F = np.zeros((5, self.num))
        for i in range(self.num):
            k = 4 * np.pi * 1e-7
            p = pc - self.p_electromagnet[0:3, i]
            ma = self.m_electromagnet_hat[0:3, i] * self.coef[i, 0]
            p_hat = p / np.linalg.norm(p)
            # 磁源在某点产生的磁场梯度矩阵
            gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))
            F[0, i] = gradient[0, 0]
            F[1, i] = gradient[0, 1]
            F[2, i] = gradient[0, 2]
            F[3, i] = gradient[1, 1]
            F[4, i] = gradient[1, 2]
        gradient = F * current

        return gradient

    def moment_electromagnet(self, pc, mc, current):
        """
        计算被驱动磁铁在多个电磁铁产生的磁场下受到的力矩
        :param pc: 目标位置 array 3*1
        :param mc: 被驱动磁铁的磁矩 np.array 3*1
        :param current: 电流 array n*1
        :return: 力和力矩
        """
        moment = self.skew_matrix(mc) * self.field_electromagnet(pc, current)
        return moment

    def force_electromagnet(self, pc, mc, current):
        """
        计算被驱动磁铁在多个电磁铁产生的磁场下受到的力
        :param pc: 目标位置 array 3*1
        :param mc: 被驱动磁铁的磁矩 np.array 3*1
        :param current: 电流 array n*1
        :return: 力
        """
        force = self.m_matrix(mc) * self.gradient_electromagnet(pc, current)
        return force


desired_list = []       # 存储期望磁场磁力等
desired_flag = [0, 3, 0, 3]     # 存储标志位 0：不考虑 1：磁场/。。 2：磁场方向/磁力方向 3：磁力矩/磁力

magnet_source_nun = 3

line_num = 0
row_num = 0
jacobi_matrix = np.zeros((6, 18))

p_permanent = np.array([[1, 2, 3], [1, 2, 3], [1, 2, 3]])
m_permanent = np.array([[1, 2, 3], [1, 2, 3], [1, 2, 3]])
num_permanent = 3

p_electromagnet = np.array([[]])
m_hat_electromagnet = np.array([[]])
coef = np.array([[]])
num_electromagnet = 2
current = np.array([[]])


permanent = Permanent(p_permanent=p_permanent, m_permanent=m_permanent, num=num_permanent)
electromagnet = Electromagnet(p_electromagnet, m_hat_electromagnet, coef, num_electromagnet, current)
controlled_point_plist = np.array([[0.1, 0.2], [-0.1, -0.2], [0.1, -0.2]])
controlled_point_mlist = np.array([[0.1, 0.2], [-0.1, -0.2], [0.1, -0.2]])
controlled_num = 0

def jacobi_init():
    for i in range(len(desired_flag)):
        if desired_flag[i] != 0:
            line_num += 3
    jacobi_matrix = np.zeros((line_num, row_num))


def jacobi_solve(permanent, electromagnet, jacobi_matrix):
    i = 0
    for list_num in range(len(desired_flag)):
        controlled_num = list_num//2
        if desired_flag[list_num] != 0:
            for j in range(magnet_source_nun):
                if j < permanent.permanent_num:
                    X = np.mat(np.concatenate((permanent.p_permanent[0:3, j:j+1], permanent.m_permanent[0:3, j:j+1]), axis=0))
                    h = 0.00001
                    grad = np.mat(np.zeros([3, 6]))
                    for idx in range(X.size):
                        tmp_val = X[idx, 0]
                        X[idx, 0] = tmp_val + h
                        if (list_num+1) % 2 != 0:      # i为奇数，处理磁场相关量
                            if desired_flag[list_num] == 1:
                                field1 = permanent.field_permanent(X[0:3, 0], X[3:6, 0], controlled_point_plist[0:3, controlled_num:controlled_num+1])
                                X[idx, 0] = tmp_val - h
                                field2 = permanent.field_permanent(X[0:3, 0], X[3:6, 0], controlled_point_plist[0:3, controlled_num:controlled_num+1])
                                grad[:3, idx] = (field1 - field2) / (2 * h)
                            elif desired_flag[list_num] == 2:
                                pass
                            elif desired_flag[list_num] == 3:
                                moment1 = permanent.moment_permanent(X[0:3, 0], X[3:6, 0], controlled_point_plist[0:3, controlled_num:controlled_num+1], controlled_point_mlist[0:3, controlled_num:controlled_num+1])
                                X[idx, 0] = tmp_val - h
                                moment2 = permanent.moment_permanent(X[0:3, 0], X[3:6, 0], controlled_point_plist[0:3, controlled_num:controlled_num+1], controlled_point_mlist[0:3, controlled_num:controlled_num+1])
                                grad[:3, idx] = (moment1 - moment2) / (2 * h)
                        else:       # i为偶数，处理磁力相关量
                            if desired_flag[list_num] == 3:
                                force1 = permanent.force_permanent(X[0:3, 0], X[3:6, 0], controlled_point_plist[0:3, controlled_num:controlled_num+1], controlled_point_mlist[0:3, controlled_num:controlled_num+1])
                                X[idx, 0] = tmp_val - h
                                force2 = permanent.force_permanent(X[0:3, 0], X[3:6, 0], controlled_point_plist[0:3, controlled_num:controlled_num+1], controlled_point_mlist[0:3, controlled_num:controlled_num+1])
                                grad[:3, idx] = (force1 - force2) / (2 * h)
                        X[idx, 0] = tmp_val

                    jacobi_matrix[i*3:(i+1)*3, j*6:(j+1)*6] = grad

                else:   # 处理电磁铁相关
                    j_electromagnet = j - permanent.permanent_num
                    tmp_val = electromagnet.current[j_electromagnet, 0]
                    h = 0.00001
                    grad = np.mat(np.zeros([3, 1]))
                    electromagnet.current[j_electromagnet, 0] = tmp_val + h
                    if (list_num + 1) % 2 != 0:  # i为奇数，处理磁场相关量
                        if desired_flag[list_num] == 1:
                            field1 = electromagnet.field_electromagnet(controlled_point_plist[0:3, controlled_num:controlled_num + 1], electromagnet.current)
                            electromagnet.current[j_electromagnet, 0] = tmp_val - h
                            field2 = electromagnet.field_electromagnet(controlled_point_plist[0:3, controlled_num:controlled_num + 1], electromagnet.current)
                            grad[:3, :1] = (field1 - field2) / (2 * h)
                        elif desired_flag[list_num] == 2:
                            pass
                        elif desired_flag[list_num] == 3:
                            moment1 = electromagnet.moment_electromagnet(controlled_point_plist[0:3, controlled_num:controlled_num + 1], controlled_point_mlist[0:3, controlled_num:controlled_num+1], electromagnet.current)
                            electromagnet.current[j_electromagnet, 0] = tmp_val - h
                            moment2 = electromagnet.moment_electromagnet(controlled_point_plist[0:3, controlled_num:controlled_num + 1], controlled_point_mlist[0:3, controlled_num:controlled_num+1], electromagnet.current)
                            grad[:3, :1] = (moment1 - moment2) / (2 * h)
                    else:  # i为偶数，处理磁力相关量
                        if desired_flag[list_num] == 3:
                            force1 = electromagnet.force_electromagnet(controlled_point_plist[0:3, controlled_num:controlled_num + 1], controlled_point_mlist[0:3, controlled_num:controlled_num+1], electromagnet.current)
                            electromagnet.current[j_electromagnet, 0] = tmp_val - h
                            force2 = electromagnet.force_electromagnet(controlled_point_plist[0:3, controlled_num:controlled_num + 1], controlled_point_mlist[0:3, controlled_num:controlled_num+1], electromagnet.current)
                            grad[:3, :1] = (force1 - force2) / (2 * h)

                    jacobi_matrix[i * 3:(i + 1) * 3, permanent.permanent_num * 6 + j_electromagnet:permanent.permanent_num * 6 + j_electromagnet + 1] = grad

            i += 1

    return jacobi_matrix


import time

# 记录开始时间
start_time = time.time()

# 这里放置你想要测量运行时间的代码
# 例如，一个简单的循环
for i in range(100):
    jacobi_solve(permanent, jacobi_matrix)

# 记录结束时间
end_time = time.time()

# 计算并打印运行时间
execution_time = end_time - start_time
print(f"代码运行耗时：{execution_time}秒")
































