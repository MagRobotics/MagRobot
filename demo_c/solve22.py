from scipy.optimize import fsolve
import numpy as np
from numpy import *
from math import sin, cos
import math
# import Dual_planning
from scipy.optimize import minimize
import kinematics
# import Pose_Transform.pose_transform
# import view
from Position_Control import *
from Pose_Transform.pose_transform import *
from scipy.optimize import root
import matplotlib

import Position_Control


def T_param(theta, a, d, alpha):
    """
    获得相邻连杆之间的齐次变换矩阵函数(具体参数值参考机械臂的DH参数表)
    :param theta: 关节转角 float
    :param a: 连杆长度 float
    :param d: 连杆偏移 float
    :param alpha: 连杆扭转角 float
    :return: 齐次变换矩阵 np.mat 4*4
    """
    T = np.mat([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]])
    return T


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
    force_hat = force / np.linalg.norm(force)
    return force_hat, field

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

# def position_moment_to_pose(position, moment):
#     """
#     位置和磁矩得到对应的四元数
#     :param position: 位置 np.array 3*1
#     :param moment: 磁矩 np.array 3*1
#     :return: 四元数表示的位姿 list len==7
#     """
#     pose = list()
#     pose.extend(list(position.transpose()[0]))
#
#     phi, theta = moment_to_angle(moment)
#     angle, axis = angle_to_axis_radian(phi, theta)
#     rot_matrix = axis_radian_to_rmatrix(angle, axis)
#     quat = rmatrix_to_quat(rot_matrix)
#     pose.extend(quat)
#
#     return pose

def calculate_angles(m_hat):

    r = math.sqrt(m_hat[0,0] ** 2 + m_hat[1,0] ** 2 + m_hat[2,0] ** 2)
    if r ==0:
        return np.array([[0],[0]])
    else:
        alpha = math.acos(m_hat[2,0] / r)  # 俯仰角
        theta = math.atan2(m_hat[1,0], m_hat[0,0])
        X=np.array([[alpha],[theta]])

    # 方位角，需要根据象限调整
        return X

def calculate_axis(X):

    x = math.sin(X[0,0]) * math.cos(X[1,0])
    y = math.sin(X[0,0]) * math.sin(X[1,0])
    z = math.cos(X[0,0])
    m_hat = np.array([[x], [y], [z]])
    return m_hat

def check_solution (check):
    split_check = np.array_split(check, [3, 6, 8])
    df1_check = split_check[0]
    df2_check = split_check[1]
    dh1_check = split_check[2]
    dh2_check = split_check[3]
    dh1_check = calculate_axis(dh1_check)
    dh2_check = calculate_axis(dh2_check)
    # print("df1", df1_check, "dh1", dh1_check, "df2", df2_check, "dh2", dh2_check)
    return df1_check,df2_check,dh1_check,dh2_check

def split_solution(matrix):
    ss = np.array_split(matrix, [3, 6, 8])
    dpa1 = ss[0]
    dpa2 = ss[1]
    dm1 = ss[2]
    dm2 = ss[3]
    return dpa1,dpa2,dm1,dm2



def calculate_force_now(pa1,pa2,pc1,pc2,ma1,ma2,mc1,mc2):
    pa1c1 = pc1 - pa1
    pa2c1 = pc1 - pa2
    pa1c2 = pc2 - pa1
    pa2c2 = pc2 - pa2
    pc1c2 = pc2 - pc1
    pc2c1 = pc1 - pc2
    fo1, m1 = force_moment(pa1c1, ma1, mc1)
    fo1_, m1_ = force_moment(pa2c1, ma2, mc1)
    fo1__, m1__ = force_moment(pc2c1, mc2, mc1)
    c, fe1 = force_hat_field(pa1c1, ma1, mc1)
    c, fe1_ = force_hat_field(pa2c1, ma2, mc1)
    c, fe1__ = force_hat_field(pc2c1, mc2, mc1)
    f1o = fo1 + fo1_ + fo1__
    f1e = fe1 + fe1_ + fe1__
    m1 = m1 + m1_ + m1__
    force1, feild1 = f1o, f1e
    fo2, m2 = force_moment(pa1c2, ma1, mc2)
    fo2_, m2_ = force_moment(pa2c2, ma2, mc2)
    fo2__, m2__ = force_moment(pc1c2, mc1, mc2)
    c, fe2 = force_hat_field(pa1c2, ma1, mc2)
    c, fe2_ = force_hat_field(pa2c2, ma2, mc2)
    c, fe2__ = force_hat_field(pc1c2, mc1, mc2)
    f2o = fo2 + fo2_ + fo2__
    f2e = fe2 + fe2_ + fe2__
    m2 = m2 + m2_ + m2__
    force2, feild2 = f2o, f2e
    force0 = fo2 + fo2_
    print(force0,fo2__,force2)
    return force1,feild1,m1,force2,feild2,m2

def calculate_force_now_single(pa1,pa2,pc1,ma1,ma2,mc1):
    pa1c1 = pc1 - pa1
    pa2c1 = pc1 - pa2

    fo1, m1 = force_moment(pa1c1, ma1, mc1)
    fo1_, m1_ = force_moment(pa2c1, ma2, mc1)
    c, fe1 = force_hat_field(pa1c1, ma1, mc1)
    c, fe1_ = force_hat_field(pa2c1, ma2, mc1)
    f1o = fo1 + fo1_
    f1e = fe1 + fe1_
    m = m1 + m1_
    force1, feild1 = f1o, f1e

    return force1, feild1, m

class Back_Solve_force_moment_2:
    def __init__(self, ma_norm1,ma_norm2, mc1,mc2, pc1, pc2):
        self.ma_norm1 = ma_norm1
        self.ma_norm2 = ma_norm2
        self.mc1 = np.mat(mc1)
        self.mc2 = np.mat(mc2)
        self.force = None
        self.moment = None
        self.pc1 = np.mat(pc1)
        self.pc2 = np.mat(pc2)

    def __back_function_2(self, p_ma):
        """
        逆解函数(私有函数)
        :param p_ma: 位置和姿态 一级list length==6（其实是5个数，最后一个没有使用）
        :return: 待解列表 == 0
        根据驱动磁体位姿生成的场
        """
        pa1 = np.array([[p_ma[0]], [p_ma[1]], [p_ma[2]]])
        pca1 = self.pc1 - pa1
        pc2a1 = self.pc2 - pa1
        pc1c2 = self.pc2 - self.pc1
        pc2c1 = self.pc1 - self.pc2
        alpha1 = p_ma[3]
        theta1 = p_ma[4]
        pa2 = np.array([[p_ma[6]], [p_ma[ 7]], [p_ma[8]]])
        pca2 = self.pc1 - pa2
        pc2a2 = self.pc2 - pa2
        alpha2 = p_ma[9]
        theta2 = p_ma[10]
        ma_hat1 = np.mat([[np.sin(alpha1) * np.cos(theta1)], [np.sin(alpha1) * np.sin(theta1)], [np.cos(alpha1)]])
        ma_hat2 = np.mat([[np.sin(alpha2) * np.cos(theta2)], [np.sin(alpha2) * np.sin(theta2)], [np.cos(alpha2)]])
        force1, moment1 = force_moment(pca1, ma_hat1 * self.ma_norm1, self.mc1)
        force2,moment2 =  force_moment(pca2, ma_hat2 * self.ma_norm2, self.mc1)
        force3,moment3 = force_moment(pc2c1, self.mc2, self.mc1)
        force1_, moment1_ = force_moment(pc2a1, ma_hat1 * self.ma_norm1, self.mc2)
        force2_,moment2_ =  force_moment(pc2a2, ma_hat2 * self.ma_norm2, self.mc2)
        force3_, moment3_ = force_moment(pc1c2, self.mc1, self.mc2)
        force_c1 = force1+force2+force3
        moment_c1 = moment1 +moment2 +moment3
        force_c2 = force2_ +force1_ +force3_
        moment_c2 = moment1_ +moment2_ +moment3_


        # 下面两个减去的是胶囊的初始位姿
        force_new_c1 = (force_c1 - self.force)*1000
        moment_new_c1 = (moment_c1 - self.moment)*1000
        force_new_c2 = (force_c2 - self.force)*1000
        moment_new_c2 = (moment_c2 - self.moment)*1000
        return [force_new_c1[0, 0], force_new_c1[1, 0], force_new_c1[2, 0], moment_new_c1[0, 0], moment_new_c1[1, 0], moment_new_c1[2, 0] , force_new_c2[0, 0], force_new_c2[1, 0], force_new_c2[2, 0], moment_new_c2[0, 0], moment_new_c2[1, 0], moment_new_c2[2, 0]]

    def back_numerical_2(self, X0, force=np.array([[0], [0], [0.5e-3]]),
                       moment=np.array([[0], [0], [0]])):
        """
        求力和力矩逆解的函数（fsolve）
        :param X0: 迭代初始值
        :return:
        """
        self.force = force
        self.moment = moment
        # X0 =X0=[0.2, 0, 0.1, 0, 0, 0, 0.2, 0, 0.1, 0, 0, 0] # 迭代初始值

        result = root(self.__back_function_2, X0,tol = 1e-20)
        result = list(result.x)
        alpha1 = result[3]
        theta1 = result[4]
        result[3] = np.sin(alpha1) * np.cos(theta1)
        result[4] = np.sin(alpha1) * np.sin(theta1)
        result[5] = np.cos(alpha1)
        alpha2 = result[9]
        theta2 = result[10]
        result[9] = np.sin(alpha2) * np.cos(theta2)
        result[10] = np.sin(alpha2) * np.sin(theta2)
        result[11] = np.cos(alpha2)

        pa1 = np.array([[result[0]], [result[1]], [result[2]]])
        ma_hat1 = np.array([[result[3]], [result[4]], [result[5]]])
        pa2 = np.array([[result[6]], [result[7]], [result[8]]])
        ma_hat2 = np.array([[result[9]], [result[10]], [result[11]]])
        return pa1,ma_hat1,pa2,ma_hat2

class double_ma_solve:
    def __init__(self, mc1_norm, mc2_norm, ma1_norm, ma2_norm):
        self.force = None
        self.direction = None
        self.ma1_norm = ma1_norm
        self.mc1_norm = mc1_norm
        self.mc2_norm = mc2_norm
        self.ma2_norm = ma2_norm

    def force_hat_field_jacob_for_2_a(self, pc1, pc2, pa1, ma1_hat, mc1_hat, mc2_hat, pa2, ma2_hat):
        """
        考虑两个驱动磁源时求解雅可比矩阵，返回为10*16
        """
        X = np.mat(np.concatenate((pa1, pa2, calculate_angles(ma1_hat), calculate_angles(ma2_hat), pc1, pc2), axis=0))
        h = 0.00000001
        grad = np.mat(np.zeros([10, 16]))

        for idx in range(X.size):
            tmp_val = X[idx, 0]

            # 前向差分
            X[idx, 0] = tmp_val + h
            results_plus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 后向差分
            X[idx, 0] = tmp_val - h
            results_minus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 恢复原始值
            X[idx, 0] = tmp_val

            # 计算梯度
            grad[0:3, idx] = (results_plus['force1'] - results_minus['force1']) / (2 * h)
            grad[3:6, idx] = (results_plus['force2'] - results_minus['force2']) / (2 * h)
            grad[6:8, idx] = (calculate_angles(results_plus['feild1']) - calculate_angles(results_minus['feild1'])) / (2 * h)
            grad[8:10, idx] = (calculate_angles(results_plus['feild2']) - calculate_angles(results_minus['feild2'])) / (2 * h)

        return np.mat(grad)

    def compute_forces_and_fields(self, X, mc1_hat, mc2_hat):
        """
        计算力和力场
        """
        pa1c1 = X[10:13, 0] - X[0:3, 0]
        pa2c1 = X[10:13, 0] - X[3:6, 0]
        pa1c2 = X[13:16, 0] - X[0:3, 0]
        pa2c2 = X[13:16, 0] - X[3:6, 0]
        pc1c2 = X[13:16, 0] - X[10:13, 0]
        pc2c1 = X[10:13, 0] - X[13:16, 0]

        ma1 = calculate_axis(X[6:8, 0]) * self.ma1_norm
        ma2 = calculate_axis(X[8:10, 0]) * self.ma2_norm
        mc1 = self.mc1_norm * mc1_hat
        mc2 = self.mc2_norm * mc2_hat

        # 计算 force1
        fo1, m1 = force_moment(pa1c1, ma1, mc1)
        fo1_, m1_ = force_moment(pa2c1, ma2, mc1)
        fo1__, m1__ = force_moment(pc2c1, mc2, mc1)
        c, fe1 = force_hat_field(pa1c1, ma1, mc1)
        c, fe1_ = force_hat_field(pa2c1, ma2, mc1)
        c, fe1__ = force_hat_field(pc2c1, mc2, mc1)
        f1o = fo1 + fo1_ + fo1__
        f1e = fe1 + fe1_ + fe1__
        force1, feild1 = f1o, f1e

        # 计算 force2
        fo2, m2 = force_moment(pa1c2, ma1, mc2)
        fo2_, m2_ = force_moment(pa2c2, ma2, mc2)
        fo2__, m2__ = force_moment(pc1c2, mc1, mc2)
        c, fe2 = force_hat_field(pa1c2, ma1, mc2)
        c, fe2_ = force_hat_field(pa2c2, ma2, mc2)
        c, fe2__ = force_hat_field(pc1c2, mc1, mc2)
        f2o = fo2 + fo2_ + fo2__
        f2e = fe2 + fe2_ + fe2__
        force2, feild2 = f2o, f2e

        return {
            'force1': f1o,
            'force2': f2o,
            'feild1': f1e,
            'feild2': f2e
        }

    def compute_jacobian_forces( self,pa1, pa2, ma1_hat, ma2_hat, mc1_hat, mc2_hat, pc1, pc2):
        """
        计算 force1 和 force2 关于 pa1, pa2, ma1, ma2 的雅可比矩阵。
        """
        h = 0.00000001
        grad = np.mat(np.zeros([6, 16]))  # 6个输出（force1和force2的x,y,z），16个输入（pa1, pa2, ma1, ma2,pc1,pc2）

        X = np.mat(np.concatenate((pa1, pa2, calculate_angles(ma1_hat), calculate_angles(ma2_hat), pc1, pc2), axis=0))

        for idx in range(16):  # 对于每个输入变量
            tmp_val = X[idx, 0]

            # 前向差分
            X[idx, 0] = tmp_val + h
            results_plus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 后向差分
            X[idx, 0] = tmp_val - h
            results_minus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 恢复原始值
            X[idx, 0] = tmp_val

            # 计算梯度
            grad[0:3, idx] = (results_plus['force1'] - results_minus['force1']) / (2 * h)
            grad[3:6, idx] = (results_plus['force2'] - results_minus['force2']) / (2 * h)

        return grad

    def compute_jacobian_fields(self, pa1, pa2, ma1_hat, ma2_hat, mc1_hat, mc2_hat, pc1, pc2):
        """
        计算 force1 和 force2 关于 pa1, pa2, ma1, ma2 的雅可比矩阵。
        """
        h = 0.00000001
        grad = np.mat(np.zeros([4, 16]))  # 6个输出（force1和force2的x,y,z），16个输入（pa1, pa2, ma1, ma2,pc1,pc2）

        X = np.mat(np.concatenate((pa1, pa2, calculate_angles(ma1_hat), calculate_angles(ma2_hat), pc1, pc2), axis=0))

        for idx in range(16):  # 对于每个输入变量
            tmp_val = X[idx, 0]

            # 前向差分
            X[idx, 0] = tmp_val + h
            results_plus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 后向差分
            X[idx, 0] = tmp_val - h
            results_minus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 恢复原始值
            X[idx, 0] = tmp_val

            # 计算梯度
            grad[0:2, idx] = (calculate_angles(results_plus['feild1']) - calculate_angles(results_minus['feild1']))/ (2 * h)
            grad[2:4, idx] = (calculate_angles(results_plus['feild2']) - calculate_angles(results_minus['feild2']))/ (2 * h)

        return grad


class Back_Solve_force_moment_2_single:
    def __init__(self, ma_norm1,ma_norm2, mc1, pc1):
        self.ma_norm1 = ma_norm1
        self.ma_norm2 = ma_norm2
        self.mc1 = np.mat(mc1)
        self.force = None
        self.moment = None
        self.pc1 = np.mat(pc1)

    def __back_function_2(self, p_ma):
        """
        逆解函数(私有函数)
        :param p_ma: 位置和姿态 一级list length==6（其实是5个数，最后一个没有使用）
        :return: 待解列表 == 0
        根据驱动磁体位姿生成的场
        """
        pa1 = np.array([[p_ma[0]], [p_ma[1]], [p_ma[2]]])
        pca1 = self.pc1 - pa1
        alpha1 = p_ma[3]
        theta1 = p_ma[4]
        pa2 = np.array([[p_ma[6]], [p_ma[ 7]], [p_ma[8]]])
        pca2 = self.pc1 - pa2
        alpha2 = p_ma[9]
        theta2 = p_ma[10]
        ma_hat1 = np.mat([[np.sin(alpha1) * np.cos(theta1)], [np.sin(alpha1) * np.sin(theta1)], [np.cos(alpha1)]])
        ma_hat2 = np.mat([[np.sin(alpha2) * np.cos(theta2)], [np.sin(alpha2) * np.sin(theta2)], [np.cos(alpha2)]])
        force1, moment1 = force_moment(pca1, ma_hat1 * self.ma_norm1, self.mc1)
        force2,moment2 =  force_moment(pca2, ma_hat2 * self.ma_norm2, self.mc1)

        force_c1 = force1+force2
        moment_c1 = moment1 +moment2


        # 下面两个减去的是胶囊的初始位姿
        force_new_c1 = (force_c1 - self.force)*1000
        moment_new_c1 = (moment_c1 - self.moment)*1000

        return [force_new_c1[0, 0], force_new_c1[1, 0], force_new_c1[2, 0], moment_new_c1[0, 0], moment_new_c1[1, 0], moment_new_c1[2, 0], 0, 0, 0, 0, 0, 0]

    def back_numerical_2(self, X0, force=np.array([[0], [0], [0.5e-3]]),
                       moment=np.array([[0], [0], [0]])):
        """
        求力和力矩逆解的函数（fsolve）
        :param X0: 迭代初始值
        :return:
        """
        self.force = force
        self.moment = moment
        # X0 =X0=[0.2, 0, 0.1, 0, 0, 0, 0.2, 0, 0.1, 0, 0, 0] # 迭代初始值

        result = root(self.__back_function_2, X0,tol = 1e-20)
        result = list(result.x)
        alpha1 = result[3]
        theta1 = result[4]
        result[3] = np.sin(alpha1) * np.cos(theta1)
        result[4] = np.sin(alpha1) * np.sin(theta1)
        result[5] = np.cos(alpha1)
        alpha2 = result[9]
        theta2 = result[10]
        result[9] = np.sin(alpha2) * np.cos(theta2)
        result[10] = np.sin(alpha2) * np.sin(theta2)
        result[11] = np.cos(alpha2)

        pa1 = np.array([[result[0]], [result[1]], [result[2]]])
        ma_hat1 = np.array([[result[3]], [result[4]], [result[5]]])
        pa2 = np.array([[result[6]], [result[7]], [result[8]]])
        ma_hat2 = np.array([[result[9]], [result[10]], [result[11]]])
        return pa1,ma_hat1,pa2,ma_hat2


class double_ma_solve_single:
    def __init__(self, mc_norm, ma1_norm, ma2_norm):
        self.force = None
        self.direction = None
        self.ma1_norm = ma1_norm
        self.mc_norm = mc_norm
        self.ma2_norm = ma2_norm

    def force_hat_field_jacob_for_2_a(self, pc, pa1, ma1_hat, mc_hat, pa2, ma2_hat):
        """
        考虑两个驱动磁源时求解雅可比矩阵，返回为5*13
        """
        X = np.mat(np.concatenate((pa1, pa2, calculate_angles(ma1_hat), calculate_angles(ma2_hat), pc), axis=0))
        h = 0.00000001
        grad = np.mat(np.zeros([5, 13]))

        for idx in range(X.size):
            tmp_val = X[idx, 0]

            # 前向差分
            X[idx, 0] = tmp_val + h
            results_plus = self.compute_forces_and_fields(X, mc_hat)

            # 后向差分
            X[idx, 0] = tmp_val - h
            results_minus = self.compute_forces_and_fields(X, mc_hat)

            # 恢复原始值
            X[idx, 0] = tmp_val

            # 计算梯度
            grad[0:3, idx] = (results_plus['force1'] - results_minus['force1']) / (2 * h)
            grad[3:5, idx] = (calculate_angles(results_plus['feild1']) - calculate_angles(results_minus['feild1'])) / (2 * h)

        return np.mat(grad)

    def compute_forces_and_fields(self, X, mc_hat):
        """
        计算力和力场
        """
        pa1c1 = X[10:13, 0] - X[0:3, 0]
        pa2c1 = X[10:13, 0] - X[3:6, 0]

        ma1 = calculate_axis(X[6:8, 0]) * self.ma1_norm
        ma2 = calculate_axis(X[8:10, 0]) * self.ma2_norm
        mc1 = self.mc_norm * mc_hat

        # 计算 force1
        fo1, m1 = force_moment(pa1c1, ma1, mc1)
        fo1_, m1_ = force_moment(pa2c1, ma2, mc1)
        c, fe1 = force_hat_field(pa1c1, ma1, mc1)
        c, fe1_ = force_hat_field(pa2c1, ma2, mc1)
        f1o = fo1 + fo1_
        f1e = fe1 + fe1_

        return {
            'force1': f1o,
            'feild1': f1e,
        }

    def compute_jacobian_forces( self,pa1, pa2, ma1_hat, ma2_hat, mc1_hat, mc2_hat, pc1, pc2):
        """
        计算 force1 和 force2 关于 pa1, pa2, ma1, ma2 的雅可比矩阵。
        """
        h = 0.00000001
        grad = np.mat(np.zeros([6, 16]))  # 6个输出（force1和force2的x,y,z），16个输入（pa1, pa2, ma1, ma2,pc1,pc2）

        X = np.mat(np.concatenate((pa1, pa2, calculate_angles(ma1_hat), calculate_angles(ma2_hat), pc1, pc2), axis=0))

        for idx in range(16):  # 对于每个输入变量
            tmp_val = X[idx, 0]

            # 前向差分
            X[idx, 0] = tmp_val + h
            results_plus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 后向差分
            X[idx, 0] = tmp_val - h
            results_minus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 恢复原始值
            X[idx, 0] = tmp_val

            # 计算梯度
            grad[0:3, idx] = (results_plus['force1'] - results_minus['force1']) / (2 * h)
            grad[3:6, idx] = (results_plus['force2'] - results_minus['force2']) / (2 * h)

        return grad

    def compute_jacobian_fields(self, pa1, pa2, ma1_hat, ma2_hat, mc1_hat, mc2_hat, pc1, pc2):
        """
        计算 force1 和 force2 关于 pa1, pa2, ma1, ma2 的雅可比矩阵。
        """
        h = 0.00000001
        grad = np.mat(np.zeros([4, 16]))  # 6个输出（force1和force2的x,y,z），16个输入（pa1, pa2, ma1, ma2,pc1,pc2）

        X = np.mat(np.concatenate((pa1, pa2, calculate_angles(ma1_hat), calculate_angles(ma2_hat), pc1, pc2), axis=0))

        for idx in range(16):  # 对于每个输入变量
            tmp_val = X[idx, 0]

            # 前向差分
            X[idx, 0] = tmp_val + h
            results_plus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 后向差分
            X[idx, 0] = tmp_val - h
            results_minus = self.compute_forces_and_fields(X, mc1_hat, mc2_hat)

            # 恢复原始值
            X[idx, 0] = tmp_val

            # 计算梯度
            grad[0:2, idx] = (calculate_angles(results_plus['feild1']) - calculate_angles(results_minus['feild1']))/ (2 * h)
            grad[2:4, idx] = (calculate_angles(results_plus['feild2']) - calculate_angles(results_minus['feild2']))/ (2 * h)

        return grad

def objective_function(delta_q, H, h_d):
    return np.linalg.norm(H * delta_q - h_d)

def constraint1(delta_q, F, f_d):
    print("约束",F * delta_q - f_d)
    return F * delta_q - f_d

def constraint2(delta_q, W, r):
    return np.linalg.norm(W * delta_q) - r

def gradient(delta_q, H, h_d):
    h = 1e-4
    grad = np.zeros_like(delta_q, dtype=float)
    for i in range(delta_q.size):
        delta_q_plus = delta_q.copy()
        delta_q_plus[i, 0] += h
        delta_q_minus = delta_q.copy()
        delta_q_minus[i, 0] -= h
        grad[i, 0] = (objective_function(delta_q_plus, H, h_d) - objective_function(delta_q_minus, H, h_d)) / (2 * h)
    return grad

def penalty(delta_q, F, f_d, W, r):
    return 1000*(np.linalg.norm(constraint1(delta_q, F, f_d))) + np.maximum(0, constraint2(delta_q, W, r))**2

def optimize(H, h_d, F, f_d, W, r, epsilon=1e-6, max_iter=5000, alpha=1e-7):
    delta_q = np.mat(np.zeros([12, 1]))
    k = 0
    alpha0 = alpha
    while k < max_iter:
        grad = gradient(delta_q, H, h_d)
        penalty_value = penalty(delta_q, F, f_d, W, r)
        delta_q_new = delta_q - alpha * grad

        # 动态调整学习率
        if penalty_value > 1e-7:
            alpha *= 0.9
        else:
            alpha = alpha0

        print('迭代次数：', k, '约束1：', np.linalg.norm(constraint1(delta_q_new, F, f_d)))

        if np.linalg.norm(delta_q_new - delta_q) < epsilon:
            break

        delta_q = delta_q_new
        k += 1
    print('迭代次数：', k, '约束1：', np.linalg.norm(constraint1(delta_q_new, F, f_d)))

    return delta_q



test = 2



# if test == 2:
#     t=0
#     v1_last =0
#     v2_last = 0
#     pc1 = np.array([[0.1], [0.1], [0]])
#     pc2 = np.array([[0.3], [0.1], [0]])
#     ePc1 = pc1
#     ePc2 = pc2
#     ma1_norm = 19.5
#     ma2_norm = 19.5
#     mc1_norm = 0.1015
#     mc2_norm = 0.1015
#     mc1_hat = np.array([[0], [0], [1]])
#     mc2_hat = np.array([[0], [0], [1]])
#     mc1 = mc1_norm * mc1_hat
#     mc2 = mc2_norm * mc2_hat
#     emc1_hat = mc1_hat
#     emc2_hat = mc2_hat
#     F1_0,F2_0 = np.array([[0], [0], [0]]),np.array([[0], [0], [0]])
#     M1_0, M2_0 = np.array([[0], [0]]),np.array([ [0], [0]])
#     wsp1 = np.array([[0.4], [0.2], [1]])
#     wsp2 = np.array([[0], [0], [0]])
#     SR1 = [0.03, 0.02, 0.02, 0.02, 0.02]
#     SR2 = [0.03, 0.02, 0.02, 0.02, 0.02]
#     T1 = np.array([
#         [-0.98267274, -0.18532605, -0.00292079, -0.52890682979],
#         [0.17926366, -0.95429625, 0.23913014, 0.14211470775],
#         [-0.04710434, 0.23446308, 0.97098314, -0.37837477746],
#         [0.0, 0.0, 0.0, 1.0]
#     ])
#
#     T2 = np.array([
#         [0.99917579, 0.01909051, 0.03582295, 0.96150574779],
#         [-0.020559, 0.99894423, 0.0410825, 0.14211470775],
#         [-0.03500085, -0.04178511, 0.99851337, -0.33066147363],
#         [0.0, 0.0, 0.0, 1.0]
#     ])
#     # print(11111111111)
#     # print(T_matrix_to_pose(T1))
#     # print(T_matrix_to_pose(T2))
#     # print(11111111111)
#     a = np.array([[0], [-0.42500], [-0.39225], [0], [0], [0]])
#     alpha = np.array([[np.pi / 2], [0], [0], [np.pi / 2], [-np.pi / 2], [0]])
#     d= np.array([[0.089159], [0], [0], [0.10915], [0.09465], [0.08230]])
#     T_bias = np.mat([[1, 0, 0 ,0], [0, 1, 0, 0], [0, 0, 1, 0.085], [0, 0, 0, 1]])
#     Manipulator1 = kinematics.Manipulator2(T1 , a,d,alpha,T_bias )
#     Manipulator2 = kinematics.Manipulator2(T2, a, d, alpha, T_bias)
#     Manipulator3 = kinematics.double_Manipulator2(T1,T2, a, d, alpha, T_bias)
#     # Path = Dual_planning.Path_test(wsp1, wsp2, SR1, SR2, T1, T2, a, d, alpha, T_bias)
#     JI = np.mat([[1,0,0,0,0,0,0,0,0,0],
#                  [0,1,0,0,0,0,0,0,0,0],
#                  [0,0,1,0,0,0,0,0,0,0],
#                  [0,0,0,1,0,0,0,0,0,0],
#                  [0,0,0,0,1,0,0,0,0,0],
#                  [0,0,0,0,0,1,0,0,0,0],
#                  [0,0,0,0,0,0,1,0,0,0],
#                  [0,0,0,0,0,0,0,1,0,0],
#                  [0,0,0,0,0,0,0,0,1,0],
#                  [0,0,0,0,0,0,0,0,0,1],
#                  [0,0,0,0,0,0,0,0,0,0],
#                  [0,0,0,0,0,0,0,0,0,0],
#                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                  [0,0,0,0,0,0,0,0,0,0],
#                  [0,0,0,0,0,0,0,0,0,0]])
#     pc1_point = []
#     pc2_point = []
#     pa1_point = []
#     pa2_point = []
#     epc1_point = []
#     epc2_point = []
#     Rlist = []
#     dq1_list =[]
#     dq2_list = []
#     Tlist = []
#
#     initial_solution_1 = Back_Solve_force_moment_2(ma1_norm, ma2_norm, mc1, mc2, pc1, pc2)
#     pa1, ma_hat1, pa2, ma_hat2 = initial_solution_1.back_numerical_2(X0=[0.1, 0.1, 0.2, 0, 0, 0, 0.3, 0.1, 0.2, 0, 0, 0])
#     pa1_ = pa1
#     pa2_ = pa2
#     theta1 = Manipulator1.back_numerical(pa1,ma_hat1,[-1, -0.5, 0.5, 0.3, 0.2, 1])
#     theta2 = Manipulator2.back_numerical(pa2, ma_hat2,  [-1, -0.5, 0.5, 0.3, 0.2, 1])
#     while 1:
#          if t<20:
#              dt = 0.1
#
#              ePc1 = ePc1 + np.array([[-0.001*dt], [0*dt], [00*dt]])
#              ePc2 = ePc2 + np.array([[0.001*dt], [0*dt], [0]])
#              Kp1 = 0.01
#              Kp2 = 0.01
#              Ki = 0.000000000000000000
#              kd = 0.00000000000000000
#          else:
#              break
#          # print(pa1,pa2,pc1,pc2,ma1_norm*ma_hat1,ma2_norm*ma_hat2,mc1_hat*mc1_norm,mc2_hat*mc2_norm)
#          force1,feild1,force2,feild2 = calculate_force_now(pa1,pa2,pc1,pc2,ma1_norm*ma_hat1,ma2_norm*ma_hat2,mc1_hat*mc1_norm,mc2_hat*mc2_norm)
#          print("force1",force1,"feild1",feild1,"force2",force2,"feild2",feild2)
#          mc1_hat = feild1 / (np.linalg.norm(feild1))
#          mc2_hat = feild2 / (np.linalg.norm(feild2))
#          position = Position_Control.position_controller(ePc1,ePc2,emc1_hat,emc2_hat,force1,feild1,force2,feild2,2.89e-3,dt)
#          v1_now, v2_now, x1_now, x2_now =position.physics(v1_last,v2_last,pc1,pc2)
#          print('222222222222222',x1_now)
#          dpc1 = x1_now - pc1
#          dpc2 = x2_now - pc2
#          v1_last, v2_last, pc1, pc2 = v1_now, v2_now, x1_now, x2_now
#
#          F1, F2, M1, M2=position.pid_controller(pc1,pc2,mc1_hat,mc2_hat,Kp1,Kp2,Ki,kd)
#          df1 = F1 - F1_0
#          df2 = F2 - F2_0
#          dh1 = M1
#          dh2 = M2
#          F1_0 , F2_0 , M1_0 , M2_0 = F1 , F2, M1, M2
#          solution2 = double_ma_solve(mc1_norm,mc2_norm, ma1_norm, ma2_norm)
#          Jf = solution2.force_hat_field_jacob_for_2_a(pc1, pc2, pa1, ma_hat1, mc1_hat, mc2_hat, pa2, ma_hat2)
#
#
#          JfI = Jf*JI
#          rank = np.linalg.matrix_rank(Jf)
#          Rlist.append(rank)
#          # iJf = regularized_inverse(JfI, 10e-5)
#          iJf = np.linalg.pinv(JfI)
#          expect = np.mat(np.concatenate((df1, df2, dh1, dh2), axis=0))
#          dd = expect - Jf*np.mat(np.concatenate((np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]),dpc1,dpc2)))
#
#
#          solutions = iJf * dd
#          dpa1,dpa2,dm1,dm2 = split_solution(solutions)
#          pa1_= pa1_ +dpa1
#          pa2_ = pa2_ +dpa2
#          JA1 = Manipulator1.fkine_jacob(theta1)
#          JA2 = Manipulator2.fkine_jacob(theta2)
#
#          # C1=Path.collision_test_ws(theta1, theta2)
#          # C2=Path.collision_test_manipulator(theta1, theta2)
#          # """奇异算法应对测试"""
#          # dh = np.mat(np.concatenate((dh1, dh2), axis=0))
#          # df = np.mat(np.concatenate((df1, df2), axis=0))
#          # J1 = solution2.compute_jacobian_forces(pa1, pa2, ma_hat1, ma_hat2, mc1_hat, mc2_hat, pc1, pc2)
#          # J2 = solution2.compute_jacobian_fields(pa1, pa2, ma_hat1, ma_hat2, mc1_hat, mc2_hat, pc1, pc2)
#          # J1I = J1*JI
#          # J2I = J2*JI
#          # ddf = df - J1 * np.concatenate((np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]),dpc1,dpc2))
#          # ddh = dh - J2 * np.concatenate((np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]),dpc1,dpc2))
#          # JA = Manipulator3.fkine_2_jacob(theta1,theta2)
#          # J_force_Q = J1I*JA
#          # J_field_Q = J2I*JA
#          # W=0.1
#          # r = 0.1
#          # delta_q_opt = optimize(J_field_Q, ddh, J_force_Q, ddf, W, r)
#          # print("Optimized delta_q:", delta_q_opt)
#
#
#
#
#          dq1 = np.linalg.pinv(JA1)*np.concatenate((dpa1,dm1),axis = 0)
#          dq2 = np.linalg.pinv(JA2)*np.concatenate((dpa2,dm2),axis = 0)
#          # if t>10 and t<11 :
#          #     print("i love you")
#          #     dq1 = np.array([[delta_q_opt[0,0]],[delta_q_opt[1,0]],[delta_q_opt[2,0]],[delta_q_opt[3,0]],[delta_q_opt[4,0]],[delta_q_opt[5,0]]])
#          #     dq2 = np.array([[delta_q_opt[6,0]],[delta_q_opt[7,0]],[delta_q_opt[8,0]],[delta_q_opt[9,0]],[delta_q_opt[10,0]],[delta_q_opt[11,0]]])
#          # print("dq1", dq1, "dq2", dq2)
#          # print("1",J_force_Q*np.concatenate((dq1,dq2),axis = 0))
#          # print("2",dd)
#          theta1 = theta1 + dq1
#          theta2 = theta2 + dq2
#          pa1,ma_hat1 = Manipulator1.fkine(theta1)
#          print('1111111111111111111111111111111111')
#          pa2,ma_hat2 = Manipulator2.fkine(theta2)
#          Tlist.append(t)
#
#
#          pc1_point.append(pc1)
#          pc2_point.append(pc2)
#          pa1_point.append(pa1)
#          pa2_point.append(pa2)
#          epc1_point.append(ePc1)
#          epc2_point.append(ePc2)
#          dq1_list.append(float(max(dq1)))
#          dq2_list.append(float(max(dq2)))
#
#          t = t+dt
#          print("t=",t,"ePc1=",ePc1,"pc1=",pc1,"ePc2=",ePc2,"pc2=",pc2,"pa1=",pa1,"pa1_",pa1_,"pa2",pa2,"pa2_",pa2_)
#          print(theta1)
#
#     # view.view_3D_lines(pa1_point,pa2_point,pc1_point,pc2_point,epc1_point,epc2_point)
#     # view.view_2d_err(dq1_list , dq2_list, Tlist)
#     # pc2x, pc2y, pc2z = view.exchange(epc1_point)
#     # view.view_3d(Rlist,pc2x,pc2y,pc2z)


# dt = 0.1
# Kp1 = 0.01
# Kp2 = 0.01
# Ki = 0.00000
# kd = 0.00000
#
# JI = np.mat([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#              [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#              [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
#              [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
#              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
#              [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
#              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
#              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
#              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
#              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
#
# dpc1 = self.x1_now - self.pc1
# dpc2 = self.x2_now - self.pc2
# self.v1_last, self.v2_last, self.pc1, self.pc2 = self.v1_now, self.v2_now, self.x1_now, self.x2_now
#
# F1, F2, M1, M2 = self.position.pid_controller(self.pc1, self.pc2, self.mc1_hat, self.mc2_hat, Kp1, Kp2, Ki, kd)
# df1 = F1 - self.F1_0
# df2 = F2 - self.F2_0
# dh1 = M1 - self.M1_0
# dh2 = M2 - self.M2_0
# self.F1_0, self.F2_0, self.M1_0, self.M2_0 = F1, F2, M1, M2
# solution2 = double_ma_solve(self.mc1_norm, self.mc2_norm, self.ma1_norm, self.ma2_norm)
# Jf = solution2.force_hat_field_jacob_for_2_a(self.pc1, self.pc2, self.pa1, self.ma_hat1, self.mc1_hat, self.mc2_hat,
#                                              self.pa2, self.ma_hat2)
#
# JfI = Jf * JI
# rank = np.linalg.matrix_rank(Jf)
# iJf = np.linalg.pinv(JfI)
# expect = np.mat(np.concatenate((df1, df2, dh1, dh2), axis=0))
# dd = expect - Jf * np.mat(
#     np.concatenate((np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]]), dpc1, dpc2)))
#
# solutions = iJf * dd
# dpa1, dpa2, dm1, dm2 = split_solution(solutions)
# # pa1_ = pa1_ + dpa1
# # pa2_ = pa2_ + dpa2
# # JA1 = Manipulator1.fkine_jacob(theta1)
# # JA2 = Manipulator2.fkine_jacob(theta2)
# JA1 = self.scene.magnetic_source[0]['robot'].fkine_jacob(self.scene.magnetic_source[0]['theta'])
# JA2 = self.scene.magnetic_source[1]['robot'].fkine_jacob(self.scene.magnetic_source[1]['theta'])
#
# dq1 = np.linalg.pinv(JA1) * np.concatenate((dpa1, dm1), axis=0)
# dq2 = np.linalg.pinv(JA2) * np.concatenate((dpa2, dm2), axis=0)
# self.scene.magnetic_source[0]['theta'] += dq1
# self.scene.magnetic_source[1]['theta'] += dq2
# self.pa1, self.ma_hat1 = self.scene.magnetic_source[0]['robot'].fkine(self.scene.magnetic_source[0]['theta'])
# self.pa2, self.ma_hat2 = self.scene.magnetic_source[1]['robot'].fkine(self.scene.magnetic_source[1]['theta'])
#
# pose_list = self.scene.magnetic_source[0]['robot'].fkine_all_link(self.scene.magnetic_source[0]['theta'])
# for i in range(len(self.scene.magnetic_source[0]['link_pose_list'])):
#     self.scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]
# pose_list2 = self.scene.magnetic_source[1]['robot'].fkine_all_link(self.scene.magnetic_source[1]['theta'])
# for i in range(len(self.scene.magnetic_source[1]['link_pose_list'])):
#     self.scene.magnetic_source[1]['link_pose_list'][i].position[0][:] = pose_list2[i]
#
# self.ePc1 = self.ePc1 + np.array([[-0.00 * dt], [0 * dt], [-0.001 * dt]])
# # print(self.ePc1)
# self.ePc2 = self.ePc2 + np.array([[0.00 * dt], [0 * dt], [-0.001 * dt]])
#
# # mc1_hat = self.pose_estimate.h_hat_pose[0]
# # mc2_hat = self.pose_estimate.h_hat_pose[1]
# print("force1", self.pc1, "force2", self.pc2)
# self.force1, self.feild1, self.force2, self.feild2 = calculate_force_now(self.pa1, self.pa2, self.pc1, self.pc2,
#                                                                          self.ma1_norm * self.ma_hat1,
#                                                                          self.ma2_norm * self.ma_hat2,
#                                                                          self.mc1_hat * self.mc1_norm,
#                                                                          self.mc2_hat * self.mc2_norm)
# # print("force1", self.force1, "feild1", self.feild1, "force2", self.force2, "feild2", self.feild2)
#
# self.position = position_controller(self.ePc1, self.ePc2, self.emc1_hat, self.emc2_hat, self.force1, self.feild1,
#                                     self.force2, self.feild2, 2.89e-3, 0.1)
# self.v1_now, self.v2_now, self.x1_now, self.x2_now = self.position.physics(self.v1_last, self.v2_last, self.pc1,
#                                                                            self.pc2)







