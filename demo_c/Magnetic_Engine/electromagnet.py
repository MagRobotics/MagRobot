import numpy as np

from Pose_Transform import pose_transform


class Electromagnet:
    def __init__(self, p_electromagnet, m_hat_electromagnet, coef, num):
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

    def field_gradient(self, pc, current):
        """
        计算多个电磁铁在某点的磁场和磁场梯度(先算八个分量的矩阵，再乘上电流)
        :param pc: 目标位置 array 3*1
        :param current: 目标位置 array n*1
        :return: 八个关于磁场的向量 np.array 8*1
        """
        pc = np.mat(pc)
        current = np.mat(current)
        F = np.zeros((8, self.num))
        for i in range(self.num):
            k = 4 * np.pi * 1e-7
            p = pc - self.p_electromagnet[0:3, i]
            ma = self.m_electromagnet_hat[0:3, i] * self.coef[i, 0]
            p_hat = p / np.linalg.norm(p)

            # 磁源在某点产生的磁场强度
            field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (
                    3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

            # 磁源在某点产生的磁场梯度矩阵
            gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * \
                       (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))
            F[0:3, i:i + 1] = field
            F[3, i] = gradient[0, 0]
            F[4, i] = gradient[0, 1]
            F[5, i] = gradient[0, 2]
            F[6, i] = gradient[1, 1]
            F[7, i] = gradient[1, 2]
        result = F * current

        return result

    def moment_force(self, pc, mc, current):
        """
        计算被驱动磁铁在多个电磁铁产生的磁场下受到的力和力矩
        :param mc: 被驱动磁铁的磁矩 np.array 3*1
        :return: 力和力矩
        """
        M = np.zeros((6, 8))
        M[0:3, 0:3] = self.skew_matrix(mc)
        M[3:6, 3:8] = self.m_matrix(mc)
        M = np.mat(M)
        moment_force = M * self.field_gradient(pc, current)
        return moment_force

    def back_analytic(self, pc, mc, moment_force):
        pc = np.mat(pc)
        F = np.zeros((8, self.num))
        for i in range(self.num):
            k = 4 * np.pi * 1e-7
            p = pc - self.p_electromagnet[0:3, i]
            ma = self.m_electromagnet_hat[0:3, i] * self.coef[i, 0]
            p_hat = p / np.linalg.norm(p)

            # 磁源在某点产生的磁场强度
            field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (
                    3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

            # 磁源在某点产生的磁场梯度矩阵
            gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * \
                       (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))
            F[0:3, i:i + 1] = field
            F[3, i] = gradient[0, 0]
            F[4, i] = gradient[0, 1]
            F[5, i] = gradient[0, 2]
            F[6, i] = gradient[1, 1]
            F[7, i] = gradient[1, 2]
        F = np.mat(F)

        M = np.zeros((6, 8))
        M[0:3, 0:3] = self.skew_matrix(mc)
        M[3:6, 3:8] = self.m_matrix(mc)
        M = np.mat(M)

        matrix = M * F
        # print(matrix.shape)
        # print(np.linalg.matrix_rank(matrix))
        moment_force = np.mat(moment_force)
        # print(np.linalg.pinv(matrix))
        # print(force_moment)
        current = np.linalg.pinv(matrix) * moment_force

        return current

    def field_force(self, pc, mc, current):
        """
        计算被驱动磁铁在多个电磁铁产生的磁场下受到的力和力矩
        :param mc: 被驱动磁铁的磁矩 np.array 3*1
        :return: 力和力矩
        """
        M = np.zeros((6, 8))
        M[0:3, 0:3] = np.eye(3)
        M[3:6, 3:8] = self.m_matrix(mc)
        M = np.mat(M)
        field_force = M * self.field_gradient(pc, current)
        return field_force

    def back_analytic_field_force(self, pc, mc, field_force):
        pc = np.mat(pc)
        F = np.zeros((8, self.num))
        for i in range(self.num):
            k = 4 * np.pi * 1e-7
            p = pc - self.p_electromagnet[0:3, i]
            ma = self.m_electromagnet_hat[0:3, i] * self.coef[i, 0]
            p_hat = p / np.linalg.norm(p)

            # 磁源在某点产生的磁场强度
            field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (
                    3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

            # 磁源在某点产生的磁场梯度矩阵
            gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * \
                       (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))
            F[0:3, i:i + 1] = field
            F[3, i] = gradient[0, 0]
            F[4, i] = gradient[0, 1]
            F[5, i] = gradient[0, 2]
            F[6, i] = gradient[1, 1]
            F[7, i] = gradient[1, 2]
        F = np.mat(F)

        M = np.zeros((6, 8))
        M[0:3, 0:3] = np.eye(3)
        M[3:6, 3:8] = self.m_matrix(mc)
        M = np.mat(M)

        matrix = M * F
        field_force = np.mat(field_force)
        # print(np.linalg.pinv(matrix))
        # print(force_moment)
        current = np.linalg.pinv(matrix) * field_force

        return current

    def back_analytic_field_ignore(self, pc, mc, field_ignore):
        pc = np.mat(pc)
        F = np.zeros((8, self.num))
        for i in range(self.num):
            k = 4 * np.pi * 1e-7
            p = pc - self.p_electromagnet[0:3, i]
            ma = self.m_electromagnet_hat[0:3, i] * self.coef[i, 0]
            p_hat = p / np.linalg.norm(p)

            # 磁源在某点产生的磁场强度
            field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (
                    3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

            # 磁源在某点产生的磁场梯度矩阵
            gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * \
                       (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))
            F[0:3, i:i + 1] = field
            F[3, i] = gradient[0, 0]
            F[4, i] = gradient[0, 1]
            F[5, i] = gradient[0, 2]
            F[6, i] = gradient[1, 1]
            F[7, i] = gradient[1, 2]
        F = np.mat(F)

        M = np.zeros((3, 8))
        M[0:3, 0:3] = np.eye(3)
        # M[3:6, 3:8] = self.m_matrix(mc)
        M = np.mat(M)

        matrix = M * F
        field_ignore = np.mat(field_ignore)
        # print(np.linalg.pinv(matrix))
        # print(force_moment)
        current = np.linalg.pinv(matrix) * field_ignore

        return current

    def back_analytic_moment_ignore(self, pc, mc, moment_ignore):
        pc = np.mat(pc)
        F = np.zeros((8, self.num))
        for i in range(self.num):
            k = 4 * np.pi * 1e-7
            p = pc - self.p_electromagnet[0:3, i]
            ma = self.m_electromagnet_hat[0:3, i] * self.coef[i, 0]
            p_hat = p / np.linalg.norm(p)

            # 磁源在某点产生的磁场强度
            field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (
                    3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

            # 磁源在某点产生的磁场梯度矩阵
            gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * \
                       (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))
            F[0:3, i:i + 1] = field
            F[3, i] = gradient[0, 0]
            F[4, i] = gradient[0, 1]
            F[5, i] = gradient[0, 2]
            F[6, i] = gradient[1, 1]
            F[7, i] = gradient[1, 2]
        F = np.mat(F)

        M = np.zeros((3, 8))
        M[0:3, 0:3] = self.skew_matrix(mc)
        # M[3:6, 3:8] = self.m_matrix(mc)
        M = np.mat(M)

        matrix = M * F
        moment_ignore = np.mat(moment_ignore)
        # print(np.linalg.pinv(matrix))
        # print(force_moment)
        current = np.linalg.pinv(matrix) * moment_ignore

        return current

class Helmholtz:
    def __init__(self, radius, n):
        """
        :param radius: 三轴亥姆霍兹线圈的半径 默认分别是x、y、z方向 array 1*n
        :param n: 三轴亥姆霍兹线圈的匝数 array 1*n
        """
        self.radius = radius
        self.n = n

    def field(self, current):
        """
        计算亥姆霍兹线圈产生的磁场（默认是在中心位置）
        :param current: 电流 array 3*1
        :return: 三个关于磁场的向量 np.array 3*1
        """
        k = 4 * np.pi * 1e-7
        result = np.array([[0.72 * k * self.n[0, 0] / self.radius[0, 0] * current[0, 0]],
                           [0.72 * k * self.n[0, 1] / self.radius[0, 1] * current[1, 0]],
                           [0.72 * k * self.n[0, 2] / self.radius[0, 2] * current[2, 0]]])

        return result

    def back_analytic_field(self, field):
        """
        @param field: 三个关于磁场的向量 np.array 3*1
        @return: 电流 array 3*1
        """

        k = 4 * np.pi * 1e-7
        current = np.array([[field[0, 0] / (0.72 * k * self.n[0, 0] / self.radius[0, 0])],
                            [field[1, 0] / (0.72 * k * self.n[0, 1] / self.radius[0, 1])],
                            [field[2, 0] / (0.72 * k * self.n[0, 2] / self.radius[0, 2])]])

        return current


class Maxwell:
    def __init__(self, radius, n, euler):
        """
        :param radius: 三轴麦克斯韦线圈的半径 默认分别是x、y、z方向 array 1*n
        :param n: 三轴麦克斯韦线圈的匝数 array 1*n
        :param euler: 三轴麦克斯韦线圈的欧拉角 np.array 3*3 每一行代表一个欧拉角
        """
        self.radius = radius
        self.n = n
        self.euler = euler

    def rotation_maxwell(self, D, euler):
        r_matrix = pose_transform.euler_to_rot(euler)
        r = np.mat(r_matrix)
        r1 = np.mat(r_matrix.transpose())

        return r * np.mat(D) * r1

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

    def gradient(self, current):
        """
        计算亥姆霍兹线圈产生的磁场（默认是在中心位置）
        :param current: 电流 array 3*1
        :return: 三个关于磁场的向量 np.array 3*1
        """
        k = 4 * np.pi * 1e-7
        # 默认竖直方向
        m0 = np.array([[-1/2, 0, 0], [0, -1/2, 0], [0, 0, 1]])
        grad = np.array([[], [], [], [], []])
        for i in range(self.euler.shape[0]):
            r_matrix = pose_transform.euler_to_rot(self.euler[i])
            r = np.mat(r_matrix)
            r_t = np.mat(r_matrix.transpose())
            grad_mat = r * np.mat(m0) * r_t
            temp = np.array(
                [[grad_mat[0, 0]], [grad_mat[0, 1]], [grad_mat[0, 2]], [grad_mat[1, 1]], [grad_mat[1, 2]]])
            grad = np.hstack((grad, temp))

        m2 = np.array([[0.64 * k * self.n[0, 0] / self.radius[0, 0]**2 * current[0, 0]],
                           [0.64 * k * self.n[0, 1] / self.radius[0, 1]**2 * current[1, 0]],
                           [0.64 * k * self.n[0, 2] / self.radius[0, 2]**2 * current[2, 0]]])

        result = grad.dot(m2)
        return np.mat(result)

    def force(self, mc, current):
        """
        计算被驱动磁铁在多个电磁铁产生的磁场下受到的力
        :param mc: 被驱动磁铁的磁矩 np.array 3*1
        :return: 力 np.array 3*1
        """
        M = self.m_matrix(mc)
        force = M * self.gradient(current)
        return force

    def back_analytic_force(self, mc, force):
        """
        @param force: 三个关于磁场的向量 np.array 3*1
        @return: 电流 array 3*1
        """
        k = 4 * np.pi * 1e-7

        m0 = np.array([[-1 / 2, 0, 0], [0, -1 / 2, 0], [0, 0, 1]])
        grad = np.array([[], [], [], [], []])
        for i in range(self.euler.shape[0]):
            r_matrix = pose_transform.euler_to_rot(self.euler[i])
            r = np.mat(r_matrix)
            r_t = np.mat(r_matrix.transpose())
            grad_mat = r * np.mat(m0) * r_t
            temp = np.array(
                [[grad_mat[0, 0]], [grad_mat[0, 1]], [grad_mat[0, 2]], [grad_mat[1, 1]], [grad_mat[1, 2]]])
            grad = np.hstack((grad, temp))

        M = self.m_matrix(mc)*np.mat(grad)
        temp = np.linalg.pinv(M) * force

        current = np.array([[temp[0, 0]/(0.64 * k * self.n[0, 0] / self.radius[0, 0]**2)], [temp[1, 0]/(0.64 * k * self.n[0, 1] / self.radius[0, 1]**2)], [temp[2, 0]/(0.64 * k * self.n[0, 2] / self.radius[0, 2]**2)]])

        return current


if __name__ == "__main__":
    # p = np.array([[-1, 0, 0], [1, 0, 0]]).transpose()
    # m_hat = np.array([[0, 0, 1], [0, 0, 1]]).transpose()
    # coef = np.array([[1], [1]])
    # num = 2
    # e = electromagnet(p, m_hat, coef, num)
    # pc = np.array([[0], [0], [1]])
    # current = np.mat([[1], [1]])
    # result = e.field_gradient(pc, current)
    # # print(result)

    # mc = np.array([[0], [1], [1]])
    # # print(e.force_moment(pc, current, mc))
    # # a = np.array([[1], [2], [3]])
    # # print(e.skew_matrix(a))

    # # a = np.mat([[1, 2, 3], [4, 5, 6]])
    # # b = np.mat([[1], [2]])
    # # print(np.linalg.pinv(a)*b)
    # force_moment = np.array([[0], [0], [1], [1], [-1], [0]])
    # # print(e.back_analytic(pc, mc, force_moment))
    # print(np.eye(3))

    # # 麦克斯韦线圈测试
    # a = Maxwell(np.array([[1, 2, 3]]), np.array([[10, 20, 30]]))
    # mc = np.array([[1], [2], [2]])
    # current = np.array([[1], [5], [1]])
    # force = a.force(mc, current)
    # print(force)
    # current = a.back_analytic_force(mc, force)
    # print(current)

    # 麦克斯韦线圈测试
    a = Helmholtz(np.array([[1, 2, 3]]), np.array([[10, 20, 30]]))
    current = np.array([[2], [5], [1]])
    field = a.field(current)
    print(field)
    current = a.back_analytic_field(field)
    print(current)
    pass