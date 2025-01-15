import numpy as np
import math
from scipy import linalg
from scipy.spatial.transform import Rotation as R
from scipy.linalg import pinv


def moment_to_angle(m):
    """
    磁矩转换成角度
    :param m: 磁矩 np.array 3*1
    :return: 角度phi和theta float
    """
    phi = math.atan2(math.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2), m[2, 0])
    theta = math.atan2(m[1, 0], m[0, 0])
    return phi, theta


def angle_to_moment(phi, theta):
    """
    :param phi: float
    :param theta: float
    :return: 磁矩 np.array 3*1
    """
    return np.array([[math.sin(phi) * math.cos(theta)], [math.sin(phi) * math.sin(theta)], [math.cos(phi)]])


def moment_to_rmatrix(moment):

    phi, theta = moment_to_angle(moment)
    angle, axis = angle_to_axis_radian(phi, theta)
    rot_matrix = axis_radian_to_rmatrix(angle, axis)
    return rot_matrix


def angle_to_axis_radian(phi, theta):
    """
    这里是假设磁矩沿着x轴方向，然后得到对应的角度-轴线表示法
    :param phi: float
    :param theta: float
    :return: 角度angle float和 轴线axis np.array 3*1
    """

    axis = np.cross([1, 0, 0], [math.sin(phi) * math.cos(theta), math.sin(phi) * math.sin(theta), math.cos(phi)])
    angle = math.acos(
        np.dot([1, 0, 0], [math.sin(phi) * math.cos(theta), math.sin(phi) * math.sin(theta), math.cos(phi)]))
    return angle, np.array(axis)


def axis_radian_to_rmatrix(angle, axis):
    """
    角度和轴线转换成旋转矩阵
    :param angle: float
    :param axis: np.array 3*1
    :return: 旋转矩阵 np.array 3*3
    """

    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * angle))

    return np.array(rot_matrix)


def rmatrix_to_quat(rot_matrix):
    """
    旋转矩阵
    :param rot_matrix: 旋转矩阵 np.array 3*3
    :return: 四元数 list len==4
    """
    rot_matrix = np.mat(rot_matrix)
    r = R.from_matrix(rot_matrix)
    quat = r.as_quat()
    return list(quat)


def quat_to_rmatrix(quat):
    """
    四元数得到旋转矩阵
    :param quat: 四元数 list len==4
    :return: 旋转矩阵 np.array 3*3
    """
    r = R.from_quat(quat)
    return np.array(r.as_matrix())


def rot_to_euler(R, degree_mode=1):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    # degree_mode=1:【输出】是角度制，否则弧度制
    if degree_mode == 1:
        roll = np.rad2deg(roll)
        pitch = np.rad2deg(pitch)
        yaw = np.rad2deg(yaw)

    euler = np.array([roll, pitch, yaw])
    return euler


def euler_to_rot(euler, degree_mode=1):
    roll, pitch, yaw = euler
    # degree_mode=1:【输入】是角度制，否则弧度制
    if degree_mode == 1:
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R_y = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    R_z = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


def euler2_to_rot(euler, degree_mode=1):
    roll, pitch, yaw = euler
    # degree_mode=1:【输入】是角度制，否则弧度制
    if degree_mode == 1:
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R_y = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    R_z = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = np.dot(R_x, np.dot(R_y, R_z))
    return R


def euler_to_quat(euler):
    rot_matrix = euler_to_rot(euler)
    quat = rmatrix_to_quat(rot_matrix)
    return quat


def position_moment_to_T_matrix(position, moment):
    """
    位置和磁矩得到对应的齐次变换矩阵
    :param position: 位置 np.array 3*1
    :param moment: 磁矩 np.array 3*1
    :return: 齐次变换矩阵 np.array 4*4
    """
    T = np.zeros((4, 4))
    T[0:3, 3:4] = position
    T[3, 3] = 1

    phi, theta = moment_to_angle(moment)
    angle, axis = angle_to_axis_radian(phi, theta)
    rot_matrix = axis_radian_to_rmatrix(angle, axis)

    T[0:3, 0:3] = rot_matrix

    return T


def position_moment_to_pose(position, moment):
    """
    位置和磁矩得到对应的四元数
    :param position: 位置 np.array 3*1
    :param moment: 磁矩 np.array 3*1
    :return: 四元数表示的位姿 list len==7
    """
    pose = list()
    pose.extend(list(position.transpose()[0]))

    phi, theta = moment_to_angle(moment)
    angle, axis = angle_to_axis_radian(phi, theta)
    rot_matrix = axis_radian_to_rmatrix(angle, axis)
    quat = rmatrix_to_quat(rot_matrix)
    pose.extend(quat)

    return pose


def T_matrix_to_pose(T_matrix):
    """
    齐次变换矩阵得到四元数表示的位姿
    :param T_matrix: 齐次变换矩阵 np.array 4*4
    :return: 四元数表示的位姿 list len==7
    """

    pose = list()
    T_matrix = np.array(T_matrix)
    position = T_matrix[0:3, 3:4]

    pose.extend(list(position.transpose()[0]))

    rot_matrix = T_matrix[0:3, 0:3]
    quat = rmatrix_to_quat(rot_matrix)
    pose.extend(quat)

    return pose


def pose_to_position_moment(pose):
    """
    四元数表示的位姿得到位置和磁矩
    :param pose: list len==7
    :return: 位置 np.array 3*1 磁矩 np.array 3*1
    """
    position = np.array(np.mat(pose[0:3]).transpose())

    quat = pose[3:7]
    rot_matrix = quat_to_rmatrix(quat)
    rot_matrix = np.mat(rot_matrix)
    moment = np.array(rot_matrix * np.mat([[1], [0], [0]]))

    return position, moment


def pose_to_position_moment_z(pose):
    """
    四元数表示的位姿得到位置和磁矩
    :param pose: list len==7
    :return: 位置 np.array 3*1 磁矩 np.array 3*1
    """
    position = np.array(np.mat(pose[0:3]).transpose())

    quat = pose[3:7]
    rot_matrix = quat_to_rmatrix(quat)
    rot_matrix = np.mat(rot_matrix)
    moment = np.array(rot_matrix * np.mat([[0], [0], [-1]]))

    return position, moment


def pose_to_T_matrix(pose):
    """
    四元数表示的位姿得到齐次变换矩阵
    :param pose: list len==7
    :return: 齐次变换矩阵 np.array 4*4
    """
    T = np.zeros((4, 4))
    position = np.array(np.mat(pose[0:3]).transpose())

    quat = pose[3:7]
    rot_matrix = quat_to_rmatrix(quat)

    T[0:3, 3:4] = position
    T[0:3, 0:3] = rot_matrix
    T[3, 3] = 1

    return T


def vector_to_angles(vector):
    """
    向量转球坐标系极角、方位角
    :param vector: np.array len==3
    :return: 极角，方位角（角度制）
    """
    r = np.linalg.norm(vector)
    polar_angle = np.arccos(vector[2] / r)
    # 计算方位角
    azimuthal_angle = np.arctan2(vector[1], vector[0])

    return np.degrees(polar_angle)[0], np.degrees(azimuthal_angle)[0]


if __name__ == "__main__":
    # position = np.array([[1], [2], [3]])
    # moment = np.array([[0], [0], [15]])
    # t = position_moment_to_T_matrix(position, moment)
    # print(t)

    # a = np.array([[1], [2], [3]])
    # b = list(a.transpose()[0])
    # c = list()
    # c.extend(b)
    # print(c)
    # print(type(c))

    # position = np.array([[1], [2], [3]])
    # moment = np.array([[0], [0], [-15]])
    # t = position_moment_to_pose(position, moment)
    # print(t)
    # print(type(t))

    # t = np.array([[1, 0, 0, 2], [0, 1, 0, 3], [0, 0, 1, 5], [0, 0, 0, 1]])
    # p = T_matrix_to_pose(t)
    # print(p)
    # print(type(p))
    #
    # a = list()
    # a.append(p)
    # print(a)
    # # t = pose_to_T_matrix(p)
    # # print(t)
    # # print(type(t))
    #
    # aa = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    # qq = rot_to_euler(aa)
    # print(qq)
    #
    # bb = euler_to_rot([0, 0, 0])
    # print(bb)
    # moment = bb[0:3, 0:1]
    # print(moment)
    #
    # m1 = np.array([[0], [-1], [0]])
    # angle, axis = angle_to_axis_radian(moment_to_angle(m1)[0], moment_to_angle(m1)[1])
    # r = axis_radian_to_rmatrix(angle, axis)
    # e = rot_to_euler(r)
    # print(e)
    #
    # a = rmatrix_to_quat(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]))
    # print(a)


    # 磁矩到四元数
    # m = np.array([[0], [1], [0]])
    # phi, theta = moment_to_angle(m)
    # print(phi)
    # print(theta)
    # axis, radian = angle_to_axis_radian(phi, theta)
    # r = axis_radian_to_rmatrix(axis, radian)
    # quat = rmatrix_to_quat(r)
    # print(quat)

    # # phi, theta到四元数
    # axis, radian = angle_to_axis_radian(1.5707963267948966, 0.001)
    # r = axis_radian_to_rmatrix(axis, radian)
    # quat = rmatrix_to_quat(r)
    # print(quat)

    # phi, theta到向量
    array = angle_to_moment(120*3.14/180, 90*3.14/180)
    print(array.T)

    # # 欧拉角到四元数
    # a = euler_to_rot((180.0, 0.0, 0.0))
    # print(a)
    # b = rmatrix_to_quat(a)
    # print(b)

    # 欧拉角转四元数
    # q = euler_to_rot([0,180,-90])
    # print(q)


