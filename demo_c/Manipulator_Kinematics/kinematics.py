import numpy as np
from math import sin, cos
from scipy.optimize import fsolve
from Pose_Transform.pose_transform import *


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

class Manipulator2:
    def __init__(self, base_pose, a, d, alpha, T_bias):
        """
        :param base_matrix: 基坐标系关于世界坐标系的齐次变换矩阵 np.array/np.mat 4*4
        :param a: 连杆长度 array n*1
        :param d: 连杆偏移 array n*1
        :param alpha: 连杆扭转角 array n*1
        :param bias_length: 偏差（永磁铁长度的一半） float
        """
        self.base_matrix = np.mat(pose_to_T_matrix(base_pose))
        self.a = np.mat(a)
        self.d = np.mat(d)
        self.alpha = np.mat(alpha)
        self.T_bias = T_bias
        self.p = None
        self.m_hat = None

    # def set_theta(self, theta):
    #     """
    #     输入theta值，设置机械臂的关节角度为theta
    #     :param theta: 关节角度 np.array n*1
    #     :return: 无返回值
    #     """
    #     self.theta = theta

    def fkine(self, theta):
        """
        整个机械臂的前向运动学,关节角度theta映射到末端永磁铁的位置和磁矩（默认永磁铁磁矩和末端坐标系的z轴平行）
        :return: 齐次变化矩阵 np.mat 4*4
        """
        T_matrix = self.base_matrix
        for i in range(theta.size):
            T_matrix = T_matrix * T_param(theta[i, 0], self.a[i, 0], self.d[i, 0], self.alpha[i, 0])

        T_matrix = T_matrix * self.T_bias

        p = T_matrix[0:3, 3]
        m_hat = T_matrix[0:3, 0:3] * np.mat([[1], [0], [0]])

        return p, m_hat

    def fkine_all_link(self, theta):
        """
        求前向运动学过程中各个连杆的位置和姿态四元数
        :return: 二维list n*7
        """
        link_list = list()
        T_matrix = self.base_matrix
        # print(T_matrix)
        # print(type(T_matrix))
        # print(T_matrix_to_pose(T_matrix))
        link_list.append(T_matrix_to_pose(T_matrix))

        for i in range(theta.size):
            T_matrix = T_matrix * T_param(theta[i, 0], self.a[i, 0], self.d[i, 0], self.alpha[i, 0])
            link_list.append(T_matrix_to_pose(T_matrix))

        T_matrix = T_matrix * self.T_bias
        link_list.append(T_matrix_to_pose(T_matrix))

        return link_list

    def fkine_jacob(self, theta):
        """
        求机械臂关节到末端位置和磁矩的雅可比矩阵
        :param theta: 关节角度 np.array n*1 (theta里面数据需为浮点数，不然可能会出问题)
        :return: 雅可比矩阵 np.array 6*n
        """
        X = np.mat(theta)
        h = 1e-4
        grad = np.mat(np.zeros([6, 6]))
        # print(X)
        for idx in range(X.size):
            tmp_val = float(X[idx, 0])
            # print(tmp_val + h)
            X[idx, 0] = tmp_val + h
            # print(float(X[idx, 0]))
            # print(f"第{idx+1}个的X+h为{X[idx, 0]}")
            p1, m_hat1 = self.fkine(X)
            X[idx, 0] = tmp_val - h
            # print(f"第{idx+1}个的X-h为{X[idx, 0]}")
            p2, m_hat2 = self.fkine(X)

            grad[:3, idx] = (p1 - p2) / (2 * h)
            grad[3:6, idx] = (m_hat1 - m_hat2) / (2 * h)
            X[idx, 0] = tmp_val
            # print(f"第{idx+1}个的X为{X[idx, 0]}")
        return grad

    def __back_function(self, theta):
        """
        逆解函数(私有函数)
        :param theta: 关节角度 np.array n*1
        :return: 待解列表 == 0
        """
        theta = np.array([[theta[0]], [theta[1]], [theta[2]], [theta[3]], [theta[4]], [theta[5]]])
        p, m_hat = self.fkine(theta)
        p_new = p - self.p
        m_hat_new = m_hat - self.m_hat

        return [p_new[0, 0], p_new[1, 0], p_new[2, 0], m_hat_new[0, 0], m_hat_new[1, 0], m_hat_new[2, 0]]

    def back_numerical(self, p, m_hat, X0):
        """
        求机械臂逆运动学解的函数（fsolve）
        :param p: 末端永磁铁的位置(中心位置) np.array 3*1
        :param m_hat: 末端永磁铁的磁矩方向(单位向量) np.array 3*1
        :param X0: 迭代初始值 一级列表list length==6
        :return: 逆解值 一级列表list length==6
        """
        self.p = p
        self.m_hat = m_hat
        result = list(fsolve(self.__back_function, X0))
        # 转成 np.array n*1 类型
        result_theta = np.array([result]).transpose()

        return result_theta

if __name__ == "__main__":
    T0 = np.array([[1, 0, 0, -0.4], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    a = np.array([[0], [-0.42500], [-0.39225], [0], [0], [0]])
    d = np.array([[0.089159], [0], [0], [0.10915], [0.09465], [0.08230]])
    alpha = np.array([[np.pi / 2], [0], [0], [np.pi / 2], [-np.pi / 2], [0]])
    bias = 0.03175 / 2
    aa = Manipulator(T0, a, d, alpha, bias)
    theta = np.array([[0.5], [1], [1.5], [2.0], [3.1], [0.0]])
    aa.set_theta(theta)
    p = np.array([[0.12], [0], [0.32]])
    m_hat = np.array([[0], [0], [-1]])
    X0 = [1, -1, -1, 1, 1, 1]
    # print(aa.back_numerical(p, m_hat, X0))

    print(aa.fkine(theta))
    print(aa.fkine_jacob(theta))

    # tmp = aa.Tmatrix_to_pose(T0)
    # print(tmp)
    # print((type(tmp)))
    # print(tmp.shape)

    # print(aa.fkine_all_link())
    # print(len(aa.fkine_all_link()))
    # print(type(aa.fkine_all_link(theta)))
    # print(len(aa.fkine_all_link(theta)))
    # print(type(aa.fkine_all_link(theta)[0]))
