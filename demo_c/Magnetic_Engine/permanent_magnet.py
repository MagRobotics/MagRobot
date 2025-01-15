from scipy.optimize import fsolve
from Pose_Transform.pose_transform import *
import signal


def field_gradient(p, ma):
    """
    计算永磁铁在空间中某位置产生的磁场和磁场梯度矩阵
    :param pa: 永磁铁的位置 array 3*1
    :param pc: 空间中某点的位置 array 3*1
    :param ma: 永磁铁的磁矩 array 3*1
    :return: 磁场 field array 3*1 和 磁场梯度矩阵 gradient 3*3
    """
    k = 4 * np.pi * 1e-7
    ma = np.mat(ma)
    p = np.mat(p)
    p_hat = p / np.linalg.norm(p)

    # 磁源在某点产生的磁场强度
    field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

    # 磁源在某点产生的磁场梯度矩阵
    gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * \
               (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))

    return field, gradient


def force_moment(p, ma, mc):
    """
    计算永磁铁在空间中某位置对被驱动永磁铁产生的磁力和磁力矩
    :param pa: 永磁铁的位置 array 3*1
    :param pc: 空间中被驱动永磁铁的位置 array 3*1
    :param ma: 永磁铁的磁矩 array 3*1
    :param mc: 被驱动磁铁的磁矩 array 3*1
    :return: 磁力 force 3*1 和 磁力矩 moment 3*1
    """
    k = 4 * np.pi * 1e-7
    ma = np.mat(ma)
    mc = np.mat(mc)
    p = np.mat(p)
    p_hat = p / np.linalg.norm(p)

    # 磁源在某点产生的磁场强度
    field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

    # 磁源在某点产生的磁场梯度矩阵
    gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * \
               (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))

    S_mc = np.mat([[0, -mc[2, 0], mc[1, 0]],
                   [mc[2, 0], 0, -mc[0, 0]],
                   [-mc[1, 0], mc[0, 0], 0]])

    force = gradient * mc
    force[2, 0] = force[2, 0]
    moment = S_mc * field
    return force, moment


class Back_Solve_force_moment:
    def __init__(self, ma_norm, mc, pc):
        self.ma_norm = ma_norm
        self.mc = np.mat(mc)
        self.force = None
        self.moment = None
        self.pc = np.mat(pc)

    def __back_function(self, p_ma):
        """
        逆解函数(私有函数)
        :param p_ma: 位置和姿态 一级list length==6（其实是5个数，最后一个没有使用）
        :return: 待解列表 == 0
        """
        p = np.array([[p_ma[0]], [p_ma[1]], [p_ma[2]]])
        alpha = p_ma[3]
        theta = p_ma[4]
        ma_hat = np.mat([[np.sin(alpha) * np.cos(theta)], [np.sin(alpha) * np.sin(theta)], [np.cos(alpha)]])
        force, moment = force_moment(p, ma_hat * self.ma_norm, self.mc)

        # 下面两个减去的是胶囊的初始位姿
        force_new = force - self.force
        moment_new = moment - self.moment
        return [force_new[0, 0], force_new[1, 0], force_new[2, 0], moment_new[0, 0], moment_new[1, 0], moment_new[2, 0]]

    def back_numerical(self, X0=[0, 0.1, -0.23, 0, 0, 0], force=np.array([[0], [0], [0]]),
                       moment=np.array([[0], [0], [0]])):
        """
        求力和力矩逆解的函数（fsolve）
        :param X0: 迭代初始值
        :return:
        """
        self.force = force
        self.moment = moment
        # X0 = [0, 0.1, -0.23, 0, 0, 0] # 迭代初始值
        result = list(fsolve(self.__back_function, X0))
        alpha = result[3]
        theta = result[4]
        result[3] = np.sin(alpha) * np.cos(theta)
        result[4] = np.sin(alpha) * np.sin(theta)
        result[5] = np.cos(alpha)

        p = np.array([[result[0]], [result[1]], [result[2]]])
        ma_hat = np.array([[result[3]], [result[4]], [result[5]]])
        pa = np.array(self.pc - p)

        pose = position_moment_to_pose(pa, ma_hat)
        return pose


def force_moment_jacob(p, ma, mc):
    X = np.mat(np.concatenate((p, ma, mc), axis=0))
    h = 0.00001
    grad = np.mat(np.zeros([6, 9]))

    for idx in range(X.size):
        tmp_val = X[idx, 0]
        X[idx, 0] = tmp_val + h
        p = X[0:3, 0]
        ma = X[3:6, 0]
        mc = X[6:9, 0]

        force1, moment1 = force_moment(p, ma, mc)
        X[idx, 0] = tmp_val - h
        p = X[0:3, 0]
        ma = X[3:6, 0]
        mc = X[6:9, 0]
        force2, moment2 = force_moment(p, ma, mc)

        grad[:3, idx] = (force1 - force2) / (2 * h)
        grad[3:6, idx] = (moment1 - moment2) / (2 * h)
        X[idx, 0] = tmp_val
    return np.mat(grad)


def force_direction(p, ma, mc_norm):
    """
    用于姿态的二自由度开环控制，位置的三自由度闭环控制
    :param p: 空间中被驱动永磁铁的位置相对于永磁铁的位置 array 3*1
    :param ma: 永磁铁的磁矩 array 3*1
    :param mc_norm: 被驱动磁铁的磁矩大小 float
    :return: 磁力 force 3*1 和 磁场方向（姿态） direction 3*1
    """
    p = np.mat(p)
    p_norm = np.linalg.norm(p)
    p_hat = p / p_norm
    ma = np.mat(ma)
    ma_norm = np.linalg.norm(ma)
    ma_hat = ma / ma_norm
    # fw = np.array([[0], [0], [-5e-4]]) #需要考虑浮力就加上这个
    k = 4 * np.pi * 1e-7

    D = 3 * p_hat * p_hat.T - np.eye(3)
    # len1 = ((D * ma_hat)[0, 0] ** 2 + (D * ma_hat)[1, 0] ** 2 + (D * ma_hat)[2, 0] ** 2) ** 0.5
    # len2 = (ma_hat.T * p_hat)[0, 0]
    force = 3 * k * ma_norm * mc_norm / (4 * np.pi * p_norm ** 4 * np.linalg.norm(D * ma_hat)) * (
            ma_hat * ma_hat.T - (1 + 4 * (ma_hat.T * p_hat)[0, 0] ** 2) * np.eye(3)) * p_hat

    force[2, 0] = force[2, 0]
    direction = D * ma_hat / np.linalg.norm(D * ma_hat)

    return np.array(force), np.array(direction)


def force_direction_jacob(p_, ma_, mc_norm):
    """
    用于求姿态二自由度开环控制和位置三自由度闭环控制的雅可比矩阵
    :param p_: 被驱动磁铁相对于驱动磁铁的位置 array 3*1
    :param ma_: 驱动磁铁的磁矩 array 3*1
    :param mc_norm: 被驱动磁铁的磁矩大小 float
    :return: 雅可比矩阵 np.mat 6*6
    """
    X = np.mat(np.concatenate((p_, ma_), axis=0))
    h = 0.00001
    grad = np.mat(np.zeros([6, 6]))

    for idx in range(X.size):
        tmp_val = X[idx, 0]
        X[idx, 0] = tmp_val + h
        p = X[0:3, 0]
        ma = X[3:6, 0]

        force1, direction1 = force_direction(p, ma, mc_norm)
        X[idx, 0] = tmp_val - h
        p = X[0:3, 0]
        ma = X[3:6, 0]
        force2, direction2 = force_direction(p, ma, mc_norm)

        grad[:3, idx] = (force1 - force2) / (2 * h)
        grad[3:6, idx] = (direction1 - direction2) / (2 * h)
        X[idx, 0] = tmp_val

    return np.mat(grad)


class Back_Solve_force_direction:
    def __init__(self, ma_norm, mc_norm):
        self.force = None
        self.direction = None
        self.ma_norm = ma_norm
        self.mc_norm = mc_norm
        self.pc = np.mat(pc)

    def back_function(self, p_ma):
        p = np.array([[p_ma[0]], [p_ma[1]], [p_ma[2]]])
        alpha = p_ma[3]
        theta = p_ma[4]
        ma_hat = np.mat([[np.sin(alpha) * np.cos(theta)], [np.sin(alpha) * np.sin(theta)], [np.cos(alpha)]])
        fm, h_hat = force_direction(p, ma_hat * self.ma_norm, self.mc_norm)
        # h_hat_ = np.mat([[math.acos(h_hat[2, 0])], [math.atan2(h_hat[1, 0], h_hat[0, 0])]])

        # 下面两个减去的是胶囊的初始位姿
        fm_new = fm - self.force
        # h_new = h_hat_ - self.direction
        h_new = h_hat - self.direction
        return [fm_new[0, 0], fm_new[1, 0], fm_new[2, 0], h_new[0, 0], h_new[1, 0], h_new[2, 0]]

    def back_numerical(self, force, direction, X0):
        self.force = force
        self.direction = direction
        # self.ma_norm = ma_norm
        # self.mc_norm = mc_norm
        # X0 = np.array([0, 0.1, -0.23, 0, 0, 0])  # 迭代初始值
        result = list(fsolve(self.back_function, X0))
        alpha = result[3]
        theta = result[4]
        result[3] = np.sin(alpha) * np.cos(theta)
        result[4] = np.sin(alpha) * np.sin(theta)
        result[5] = np.cos(alpha)

        p = np.array([[result[0]], [result[1]], [result[2]]])
        ma_hat = np.array([[result[3]], [result[4]], [result[5]]])
        pa = np.array(self.pc - p)

        pose = position_moment_to_pose(pa, ma_hat)
        return pose

def force_hat_field(p, ma, mc):
    """
    计算永磁铁在空间中某位置对被驱动永磁铁产生的磁力和磁力矩
    :param pa: 永磁铁的位置 array 3*1
    :param pc: 空间中被驱动永磁铁的位置 array 3*1
    :param p: 空间中被驱动永磁铁和驱动永磁铁的相对位置 array 3*1
    :param ma: 永磁铁的磁矩 array 3*1
    :param mc: 被驱动磁铁的磁矩 array 3*1
    :return: 磁力 force 3*1 和 磁力矩 moment 3*1
    """
    k = 4 * np.pi * 1e-7
    ma = np.mat(ma)
    mc = np.mat(mc)
    p = np.mat(p)
    p_hat = p / np.linalg.norm(p)

    # 磁源在某点产生的磁场强度
    field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

    # 磁源在某点产生的磁场梯度矩阵
    gradient = 3 * k / (4 * np.pi * pow(np.linalg.norm(p), 4)) * \
               (ma * p_hat.T + p_hat * ma.T + float((p_hat.T * ma)) * (np.eye(3) - 5 * (p_hat * p_hat.T)))

    force = gradient * mc
    # print(ma)
    # print(mc)
    force_hat = force / np.linalg.norm(force)
    return force_hat, field


class TimeoutException(Exception):
    pass

class Back_Solve_force_hat_field:
    def __init__(self, ma_norm, mc_hat, pc):
        self.ma_norm = ma_norm
        self.mc_hat = np.mat(mc_hat)
        self.force_hat = None
        self.field = None
        self.pc = np.mat(pc)

    def __back_function(self, p_ma):
        """
        逆解函数(私有函数)
        ma不包含大小，就是单纯表示姿态
        :param p_ma: 位置和姿态 一级list length==6（其实是5个数，最后一个没有使用）
        :return: 待解列表 == 0
        """
        p = np.array([[p_ma[0]], [p_ma[1]], [p_ma[2]]])
        alpha = p_ma[3]
        theta = p_ma[4]
        ma_hat = np.mat([[np.sin(alpha) * np.cos(theta)], [np.sin(alpha) * np.sin(theta)], [np.cos(alpha)]])
        force_hat, field = force_hat_field(p, ma_hat * self.ma_norm, self.mc_hat)
        print(11111)
        # 下面两个减去的是胶囊的初始位姿
        force_hat_new = force_hat - self.force_hat
        field_new = field - self.field
        return [force_hat_new[0, 0], force_hat_new[1, 0], force_hat_new[2, 0], field_new[0, 0], field_new[1, 0],
                field_new[2, 0]]

    def back_numerical(self, force_hat, field):
        """
        求力和力矩逆解的函数（fsolve）
        :param X0: 迭代初始值 X0=[-0.318067083, -0.318063404, 0.000000001156, 1, 1, 1]
        :return:
        """
        self.force_hat = force_hat / np.linalg.norm(force_hat)
        self.field = field
        print(self.force_hat)
        print(self.field)
        print(111111111111111111111)

        # print("eeeeeeeeeeee")
        # print(force_hat)
        # print("eeeeeeeeeeee")
        force_hat = np.array([[0.0], [-6.45547836e-01], [7.63719838e-01]])
        phi, theta = moment_to_angle(force_hat)
        # X0 = [-0.00001, -7.03547084e-04, -0.01,  phi, theta-0.5,  0.00000000e+00]       #优质1
        # X0 = [-0.00003641515, 0.000018999998, -0.018618078,  phi, theta,  0.00000000e+00]       #优质2(方向得对，位置尽量小)
        X0 = [-0.00003641515, 0.0000018999998, -0.000018618078,  phi, theta,  0.00000000e+00]       #优质2(方向得对，位置尽量小)
        #  [[6.45547836e-01]
        # [3.64166829e-07]
        # [7.63719838e-01]]
        # phi, theta = moment_to_angle(force_hat)
        # print(phi, theta)
        # X0 = [0, 0, 1, phi, theta, 0] # 迭代初始值
        result = X0
        while self.compare_fun(result) >= 1e-3:
            # X0[0:3] = list((np.array([X0[0:3]]).transpose()-force_hat*0.1).transpose()[0])
            result = list(fsolve(self.__back_function, X0))
            alpha = result[3]
            theta = result[4]
            result[3] = np.sin(alpha) * np.cos(theta)
            result[4] = np.sin(alpha) * np.sin(theta)
            result[5] = np.cos(alpha)
        # print("dddddddddddddddd")
        # print(self.compare_fun(result))
        # print("dddddddddddddddd")
        # p = np.array([[result[0]], [result[1]], [result[2]]])
        # ma_hat = np.array([[result[3]], [result[4]], [result[5]]])
        p = np.array([result[0:3]]).transpose()
        ma_hat = np.array([result[3:6]]).transpose()
        pa = np.array(self.pc - p)

        pose = position_moment_to_pose(pa, ma_hat)
        return pose

    def compare_fun(self, result):
        p = np.array([result[0:3]]).transpose()
        ma = np.array([result[3:6]]).transpose()*self.ma_norm
        force_hat_temp, field_temp = force_hat_field(p, ma, self.mc_hat)
        loss = np.linalg.norm(force_hat_temp-self.force_hat) + np.linalg.norm(field_temp-self.field)
        return loss


def force_hat_field_jacob(pc, pa, ma_hat, mc_hat, ma_norm):
    X = np.mat(np.concatenate((pa, ma_hat), axis=0))
    h = 0.00001
    grad = np.mat(np.zeros([6, 6]))

    for idx in range(X.size):
        tmp_val = X[idx, 0]
        X[idx, 0] = tmp_val + h
        p = pc - X[0:3, 0]
        ma = X[3:6, 0]*ma_norm

        force_hat1, field1 = force_hat_field(p, ma, mc_hat)
        X[idx, 0] = tmp_val - h
        p = pc - X[0:3, 0]
        ma = X[3:6, 0]*ma_norm
        force_hat2, field2 = force_hat_field(p, ma, mc_hat)

        grad[:3, idx] = (force_hat1 - force_hat2) / (2 * h)
        grad[3:6, idx] = (field1 - field2) / (2 * h)
        X[idx, 0] = tmp_val
    return np.mat(grad)


def field1(p, ma):
    """
    计算永磁铁在空间中某位置对被驱动永磁铁产生的磁力和磁力矩
    :param pa: 永磁铁的位置 array 3*1
    :param pc: 空间中被驱动永磁铁的位置 array 3*1
    :param p: 空间中被驱动永磁铁和驱动永磁铁的相对位置 array 3*1
    :param ma: 永磁铁的磁矩 array 3*1
    :param mc: 被驱动磁铁的磁矩 array 3*1
    :return: 磁力 force 3*1 和 磁力矩 moment 3*1
    """
    k = 4 * np.pi * 1e-7
    ma = np.mat(ma)
    p = np.mat(p)

    # 磁源在某点产生的磁场强度
    field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma

    return field


class Back_Solve_field:
    def __init__(self, ma_norm, mc_hat, pc):
        self.ma_norm = ma_norm
        self.mc_hat = np.mat(mc_hat)
        self.field = None
        self.pc = np.mat(pc)

    def __back_function(self, p_ma):
        """
        逆解函数(私有函数)
        ma不包含大小，就是单纯表示姿态
        :param p_ma: 位置和姿态 一级list length==6（其实是5个数，最后一个没有使用）
        :return: 待解列表 == 0
        """
        p = np.array([[p_ma[0]], [p_ma[1]], [p_ma[2]]])
        alpha = p_ma[3]
        theta = p_ma[4]
        ma_hat = np.mat([[np.sin(alpha) * np.cos(theta)], [np.sin(alpha) * np.sin(theta)], [np.cos(alpha)]])
        field = field1(p, ma_hat * self.ma_norm)
        # 下面两个减去的是胶囊的初始位姿
        field_new = field - self.field
        print(field)
        print(self.field)
        print(field_new)
        print(p_ma)
        print('=========================')
        return [field_new[0, 0], field_new[1, 0], field_new[2, 0], 0, 0, 0]

    def back_numerical(self, field):
        """
        求力和力矩逆解的函数（fsolve）
        :param X0: 迭代初始值 X0=[-0.318067083, -0.318063404, 0.000000001156, 1, 1, 1]
        :return:
        """
        self.field = field
        # phi, theta = moment_to_angle(self.field)
        X0 = [-0.018067083, -0.018063404, -0.35, 1, 1, 0] # 迭代初始值  [[ 2.90450994 -2.54924197  2.10474482 12.79998144 15.76597308 -8.71332623]]肺部介入机械臂初始最佳关节角
        result = X0
        while self.compare_fun(result) >= 1e-10:
            result = list(fsolve(self.__back_function, X0))
            alpha = result[3]
            theta = result[4]
            result[3] = np.sin(alpha) * np.cos(theta)
            result[4] = np.sin(alpha) * np.sin(theta)
            result[5] = np.cos(alpha)
        print(result)
        # p = np.array([[result[0]], [result[1]], [result[2]]])
        # ma_hat = np.array([[result[3]], [result[4]], [result[5]]])
        p = np.array([result[0:3]]).transpose()
        ma_hat = np.array([result[3:6]]).transpose()
        pa = np.array(self.pc - p)

        pose = position_moment_to_pose(pa, ma_hat)
        return pose

    def compare_fun(self, result):
        p = np.array([result[0:3]]).transpose()
        ma = np.array([result[3:6]]).transpose()*self.ma_norm
        field_temp = field1(p, ma)
        loss = np.linalg.norm(field_temp-self.field)
        return loss


# class Back_Solve_field:
#     def __init__(self, ma_norm, mc_hat, pc):
#         self.ma_norm = ma_norm
#         print(self.ma_norm)
#         self.mc_hat = np.mat(mc_hat)
#         self.field = None
#         self.pc = np.mat(pc)
#
#     def __back_function(self, p_ma):
#         """
#         逆解函数(私有函数)
#         ma不包含大小，就是单纯表示姿态
#         :param p_ma: 位置和姿态 一级list length==6（其实是5个数，最后一个没有使用）
#         :return: 待解列表 == 0
#         """
#         p = np.array([[p_ma[0]], [p_ma[1]], [p_ma[2]]])
#         # alpha = p_ma[3]
#         # theta = p_ma[4]
#         # ma_hat = np.mat([[np.sin(alpha) * np.cos(theta)], [np.sin(alpha) * np.sin(theta)], [np.cos(alpha)]])
#         ma_hat = np.array([[p_ma[3]], [p_ma[4]], [p_ma[5]]])
#         field = field1(p, ma_hat * self.ma_norm)
#
#         # 下面两个减去的是胶囊的初始位姿
#         field_new = field - self.field
#         print(field)
#         print(self.field)
#         print(field_new)
#         print(p_ma)
#         print('=========================')
#         return [field_new[0, 0], field_new[1, 0], field_new[2, 0], 0, 0, 0]
#
#     def back_numerical(self, field=np.array([[0], [0], [1]])):
#         """
#         求力和力矩逆解的函数（fsolve）
#         :param X0: 迭代初始值 X0=[-0.318067083, -0.318063404, 0.000000001156, 1, 1, 1]
#         :return:
#         """
#         self.field = field
#         # phi, theta = moment_to_angle(self.field)
#
#         X0 = [0.03831398, 0.0, -0.518442,  0.51149299, -2.47977652,  0.22173309] # 迭代初始值
#         result = X0
#         while self.compare_fun(result) >= 1e-10:
#             result = list(fsolve(self.__back_function, X0))
#             # alpha = result[3]
#             # theta = result[4]
#             # result[3] = np.sin(alpha) * np.cos(theta)
#             # result[4] = np.sin(alpha) * np.sin(theta)
#             # result[5] = np.cos(alpha)
#
#         # p = np.array([[result[0]], [result[1]], [result[2]]])
#         # ma_hat = np.array([[result[3]], [result[4]], [result[5]]])
#         p = np.array([result[0:3]]).transpose()
#         ma_hat = np.array([result[3:6]]).transpose()
#         pa = np.array(self.pc - p)
#         print(pa)
#
#         pose = position_moment_to_pose(pa, ma_hat)
#         return pose
#
#     def compare_fun(self, result):
#         p = np.array([result[0:3]]).transpose()
#         ma = np.array([result[3:6]]).transpose()*self.ma_norm
#         field_temp = field1(p, ma)
#         loss = np.linalg.norm(field_temp-self.field)
#         return loss


def field_jacob(pc, pa, ma_hat, ma_norm):
    X = np.mat(np.concatenate((pa, ma_hat), axis=0))
    h = 0.00001
    grad = np.mat(np.zeros([3, 6]))

    for idx in range(X.size):
        tmp_val = X[idx, 0]
        X[idx, 0] = tmp_val + h
        p = pc - X[0:3, 0]
        ma = X[3:6, 0]*ma_norm

        field = field1(p, ma)
        X[idx, 0] = tmp_val - h
        p = pc - X[0:3, 0]
        ma = X[3:6, 0]*ma_norm
        field2 = field1(p, ma)

        grad[0:3, idx] = (field - field2) / (2 * h)
        X[idx, 0] = tmp_val
    return np.mat(grad)


if __name__ == "__main__":
    # pa = np.array([[0], [0], [0]])
    # ma = np.array([[0], [26.2], [0]])
    pc, mc = pose_to_position_moment([-0.075,-0.47,0.001,-0.6532814768306,-0.27059803438712005,0.27059803438712005,0.6532814768306])
    # pc = np.array([[0.1], [0.1], [0.1]])
    # mc = np.array([[0], [0], [1]])
    # p = pc - pa
    # ma_norm = 26.2
    # mc_norm = 0.126
    # ma_hat = ma / ma_norm

    # field, gradient = field_gradient(pc - pa, ma)
    # print(f"The field is {field}")
    # print(f"The gradient is {gradient}")
    #
    # force, moment = force_moment(pc - pa, ma, mc)
    # print(f"The force is {force}")
    # print(f"The moment is {moment}")
    # print(p)
    # print(ma)
    # print(mc_norm)
    # force, direction = force_direction(p, ma, mc_norm)
    # print(f"The force is {force}")
    # print(f"The direction is {direction}")
    #
    # pc = np.mat(pc)
    # pa = np.mat(pa)
    # ma = np.mat(ma)
    # k = 4 * np.pi * pow(10, -7)
    # rr = pc - pa  # 相对位置
    # r_hat = rr / np.linalg.norm(rr)  # 相对位置单位向量
    # b = k / (4 * np.pi * pow(np.linalg.norm(rr), 3)) * (3 * (r_hat * r_hat.T) - np.eye(3)) * ma  # 磁源在某点产生的磁场强度
    #
    # print(b/np.linalg.norm(b))
    # print(force_direction_jacob(p, ma, mc_norm))
    # print(force_moment_jacob(p, ma, mc))

    # a = Back_Solve_force_direction(np.mat([[0], [0], [5e-4]]), np.mat([[0], [0]]), 26.2, 0.126)
    # print(a.back_numerical())
    # X0 = np.array([0, 0.1, -0.23, 0, 0, 0])
    # b = Back_Solve_force_moment(26.2, np.array([[0], [0], [-0.126]]), np.array([[0.12], [0], [0.07]]), 0)
    # print(b.back_numerical(X0, np.mat([[0], [0], [5e-4]]), np.mat([[0], [0], [0]])))

    # p = np.array([[1], [1], [1]])
    # ma = np.array([[0], [0], [1]])
    # a, b = field_gradient(p, ma)
    # print(a)
    # print(b)
    # print(b[1, 1])

    # jf_val = force_direction_jacob(p, ma, mc_norm)
    # jf_val[0:3, 0:6] = jf_val[0:3, 0:6] * 1000000
    # jf_val[0:6, 3:6] = jf_val[0:6, 3:6] * ma_norm
    # print(jf_val)
    # print("------------------------------------")
    # jf_val = jacob_numerical(np.mat(p), np.mat(ma_hat))
    # print(jf_val)

    X0 = np.array([0.24, -0.151, 0.001, 1, 1, 0])
    print(mc)
    print(pc)
    b = Back_Solve_force_hat_field(640, mc, pc)
    c = b.back_numerical(X0=X0, force_hat=mc, field=mc*0.001414)
    print(c)
    pa, ma = pose_to_position_moment(c)
    # pa, ma = pose_to_position_moment([0.243067083,-0.151936576,0.000999998844,-1.1191633e-25,4.26471861e-09,0.382675455,0.923882837])
    a = force_hat_field(pc-pa, ma*640, mc)
    print(a)
    # print(pose_to_position_moment(c))
    # (f, h) = pose_to_position_moment([0.243067083,-0.151936576,0.000999998844,-1.1191633e-25,4.26471861e-09,0.382675455,0.923882837])
    # mm = moment_to_angle(h)
    # print(f)
    # print(mm)

