import pandas as pd
from scipy.spatial.transform import Rotation as R
import scipy.io
import numpy as np
import csv


def readOptitrack_swich_Nov(fileName):
    """
    数据清洗的函数
    :param fileName:
    :return:
    """
    # 读取OptiTrack数据
    optiTrack = pd.read_csv(fileName, header=5)

    # print(type(optiTrack))
    optiTrack = optiTrack.iloc[:, [2, 3, 4, 5, 6, 7, 8, 34, 35, 36, 37, 38, 39, 40]]

    optiTrack.columns = ['Magnet_q1', 'Magnet_q2', 'Magnet_q3', 'Magnet_q0', 'Magnet_X', 'Magnet_Y', 'Magnet_Z',
                         'Sensor_q1', 'Sensor_q2', 'Sensor_q3', 'Sensor_q0', 'Sensor_X', 'Sensor_Y', 'Sensor_Z']

    optiTrack.interpolate(method='linear', inplace=True)

    optiTrack.loc[0] = optiTrack.loc[1]

    return optiTrack


def process_optitrack_switch(fileName):
    """
    数据转化的函数
    :param optiTrack:
    :return:
    """
    optiTrack = readOptitrack_swich_Nov(fileName)

    R_AC = np.zeros((optiTrack.shape[0], 3, 3))
    P_AC = np.zeros((3, optiTrack.shape[0]))

    R_BC = np.zeros((optiTrack.shape[0], 3, 3))
    P_BC = np.zeros((3, optiTrack.shape[0]))
    for i in range(optiTrack.shape[0]):
        # R_AC 磁铁相对于相机的相对姿态
        quaternion_mag = [optiTrack.loc[i, 'Magnet_q1'], optiTrack.loc[i, 'Magnet_q2'], optiTrack.loc[i, 'Magnet_q3'],
                          optiTrack.loc[i, 'Magnet_q0']]
        # 创建 Rotation 对象
        rotation = R.from_quat(quaternion_mag)
        # 将四元数转换为旋转矩阵
        rotation_matrix = rotation.as_matrix()
        R_AC[i, :, :] = rotation_matrix

        # P_AC 磁铁相对于相机的相对位置
        P_AC[:, i:i + 1] = np.array(
            [[optiTrack.loc[i, 'Magnet_X']], [optiTrack.loc[i, 'Magnet_Y']], [optiTrack.loc[i, 'Magnet_Z']]])

        # R_BC 磁传感器相对于相机的相对姿态
        quaternion_sensor = [optiTrack.loc[i, 'Sensor_q1'], optiTrack.loc[i, 'Sensor_q2'],
                             optiTrack.loc[i, 'Sensor_q3'], optiTrack.loc[i, 'Sensor_q0']]
        # 创建 Rotation 对象
        rotation = R.from_quat(quaternion_sensor)
        # print(i)
        # 将四元数转换为旋转矩阵
        rotation_matrix = rotation.as_matrix()
        R_BC[i, :, :] = rotation_matrix

        # P_BC 磁传感器相对于相机的相对位置
        P_BC[:, i:i + 1] = np.array(
            [[optiTrack.loc[i, 'Sensor_X']], [optiTrack.loc[i, 'Sensor_Y']], [optiTrack.loc[i, 'Sensor_Z']]])

    rotation = np.zeros((optiTrack.shape[0], 3, 3))
    pos = np.zeros((3, optiTrack.shape[0]))
    # 消除相机坐标系，得到传感器关于磁铁的相对位姿
    for i in range(optiTrack.shape[0]):
        rotation[i, :, :] = R_AC[i, :, :].transpose().dot(R_BC[i, :, :])
        pos[:, i] = R_AC[i, :, :].transpose().dot(P_BC[:, i] - P_AC[:, i])
    return rotation, pos


def readSensor_data(fileName):
    mag_data = scipy.io.loadmat(fileName)
    mag_data = mag_data["D"][:, 6:9].transpose()
    mag_data[:, 0] = mag_data[:, 3]
    mag_data[:, 1] = mag_data[:, 3]
    mag_data[:, 2] = mag_data[:, 3]
    return mag_data


def cal_moment_swtich(fileName_pose, fileName_mat):
    # 进行拟合的数据量
    num = 10000

    rotation, pos = process_optitrack_switch(fileName_pose)
    mag_data = readSensor_data(fileName_mat)

    miu = np.pi * 4e-7
    k = miu / (4 * np.pi)
    B = None
    T_matrix = None
    for i in range(num):
        x = pos[0, i]
        y = pos[1, i]
        z = pos[2, i]
        r = np.linalg.norm(pos[:, i])
        T = np.array([[3 * x ** 2 - r ** 2, 3 * x * y, 3 * x * z], [3 * x * y, 3 * y ** 2 - r ** 2, 3 * y * z],
                      [3 * x * z, 3 * y * z, 3 * z ** 2 - r ** 2]]) * (k / r ** 5)

        # 记录的磁场数据是毫高斯
        if i == 0:
            B = mag_data[:, i:i + 1] * 1e-7
            # print(B)
            T_matrix = rotation[i, :, :].transpose().dot(T)
        else:
            B = np.vstack((B, mag_data[:, i:i + 1] * 1e-7))
            T_matrix = np.vstack((T_matrix, rotation[i, :, :].transpose().dot(T)))

    # print(T_matrix.shape)
    # print(B.shape)
    M = np.linalg.pinv(T_matrix).dot(B)

    return M


# Take 2023-07-28 02.21.44 PM_017(Z=180mm).csv
# Take 2023-07-28 02.21.44 PM_018(Z=185mm).csv
# Take 2023-07-28 02.21.44 PM_019(Z=190mm).csv
# Take 2023-07-28 02.21.44 PM_021(Z=195mm).csv
# Take 2023-07-28 02.21.44 PM_022(Z=200mm).csv
# Take 2023-07-28 02.21.44 PM_023(Z=205mm).csv
# Take 2023-07-28 02.21.44 PM_024(Z=210mm).csv
# Take 2023-07-31 02.23.10 PM_001(Z=215mm).csv
# Take 2023-07-31 02.23.10 PM_002(Z=220mm).csv
# Take 2023-07-31 02.23.10 PM_003(Z=225mm).csv
# Take 2023-07-31 02.23.10 PM_004(Z=230mm).csv
# Take 2023-07-31 02.23.10 PM_005(Z=235mm).csv
# Take 2023-07-31 02.23.10 PM_006(Z=240mm).csv
# Take 2023-07-31 02.23.10 PM_007(Z=245mm).csv
# Take 2023-07-31 02.23.10 PM_008(Z=250mm).csv
# Take 2023-07-31 02.23.10 PM_009(Z=255mm).csv
# Take 2023-07-31 02.23.10 PM_010(Z=260mm).csv
# Take 2023-07-31 02.23.10 PM_011(Z=265mm).csv
# Take 2023-07-31 02.23.10 PM_012(Z=270mm).csv
# Take 2023-07-31 02.23.10 PM_015(Z=275mm).csv
# Take 2023-08-01 02.46.20 PM(Z=280mm).csv

# 原来的函数，注释掉了
# def write_train_data(fileName_pose, fileName_mat):
#     train_num = 20480
#     test_num = 3072
#
#     rotation, pos = process_optitrack_switch(fileName_pose)
#     mag_data = readSensor_data(fileName_mat)
#
#     # 最终写到csv文件里的训练数据初始化
#     mag_world_train = np.zeros((3, train_num))
#     pos_train = np.zeros((3, train_num))
#
#     # 最终写到csv文件里的测试数据初始化
#     mag_world_test = np.zeros((3, test_num))
#     pos_test = np.zeros((3, test_num))
#
#     for i in range(train_num):
#     下面这里可能有点问题
#         mag_world_train[:, i] = rotation[i, :, :].dot(mag_data[:, i])
#     pos_train = pos[:, 0:train_num]
#
#     for i in range(test_num):
#         mag_world_test[:, i] = rotation[train_num + i, :, :].dot(mag_data[:, train_num + i])
#     pos_test = pos[:, train_num:train_num + test_num]
#
#     # print(pos)
#     # print(pos.shape)
#     # print(mag_world)
#     # print(mag_world.shape)
#     # 打开文件，准备写入
#
#     pos_train_df = pd.DataFrame(pos_train.T)
#     pos_train_df.to_csv("train_x.csv", index=False, header=False)
#
#     mag_world_train_df = pd.DataFrame(mag_world_train.T * 1e-7)
#     mag_world_train_df.to_csv("train_y.csv", index=False, header=False)
#
#     pos_test_df = pd.DataFrame(pos_test.T)
#     pos_test_df.to_csv("test_x.csv", index=False, header=False)
#
#     mag_world_test_df = pd.DataFrame(mag_world_test.T * 1e-7)
#     mag_world_test_df.to_csv("test_y.csv", index=False, header=False)

# def write_train_data():
#     train_ratio = 0.8
#
#     # 最终写到csv文件里的训练数据初始化
#     mag_world_train = np.zeros((3, 0))
#     pos_train = np.zeros((3, 0))
#
#     # 最终写到csv文件里的测试数据初始化
#     mag_world_test = np.zeros((3, 0))
#     pos_test = np.zeros((3, 0))
#
#     for i in range(2):
#         fileName_csv = "./Data/Z=" + str(255 + 5 * i) + "mm.csv"
#         fileName_mat = "./Data/Z=" + str(255 + 5 * i) + "mm.mat"
#
#         rotation, pos = process_optitrack_switch(fileName_csv)
#         mag_data = readSensor_data(fileName_mat)
#
#         nums = min(pos.shape[1], mag_data.shape[1])
#
#         train_num = int(train_ratio * nums)
#         test_num = nums - train_num
#
#         for j in range(train_num):
#             mag_world_train = np.hstack((mag_world_train, rotation[j, :, :].dot(mag_data[:, j:j + 1])))
#         pos_train = np.hstack((pos_train, pos[:, 0:train_num]))
#
#         for k in range(test_num):
#             mag_world_test = np.hstack(
#                 (mag_world_test, rotation[train_num + k, :, :].dot(mag_data[:, train_num + k:train_num + k + 1])))
#         pos_test = np.hstack((pos_test, pos[:, train_num:train_num + test_num]))
#
#         # # 打开文件，准备写入
#         # print(i)
#         pos_train_df = pd.DataFrame(pos_train.T)
#         pos_train_df.to_csv("train_x.csv", index=False, header=False)
#
#         mag_world_train_df = pd.DataFrame(mag_world_train.T * 1e-7)
#         mag_world_train_df.to_csv("train_y.csv", index=False, header=False)
#
#         pos_test_df = pd.DataFrame(pos_test.T)
#         pos_test_df.to_csv("test_x.csv", index=False, header=False)
#
#         mag_world_test_df = pd.DataFrame(mag_world_test.T * 1e-7)
#         mag_world_test_df.to_csv("test_y.csv", index=False, header=False)

def write_train_data():
    train_ratio = 0.8

    # 最终写到csv文件里的训练数据初始化
    mag_world_train = np.zeros((3, 0))
    pos_train = np.zeros((3, 0))

    # 最终写到csv文件里的测试数据初始化
    mag_world_test = np.zeros((3, 0))
    pos_test = np.zeros((3, 0))

    for i in range(2):
        fileName_csv = "./Data/Z=" + str(255 + 5 * i) + "mm.csv"
        fileName_mat = "./Data/Z=" + str(255 + 5 * i) + "mm.mat"

        rotation, pos = process_optitrack_switch(fileName_csv)
        mag_data = readSensor_data(fileName_mat)

        nums = min(pos.shape[1], mag_data.shape[1])

        train_num = int(train_ratio * nums)
        test_num = nums - train_num

        for j in range(train_num):
            mag_world_train = np.hstack((mag_world_train, rotation[j, :, :].dot(mag_data[:, j:j + 1])))
        pos_train = np.hstack((pos_train, pos[:, 0:train_num]))

        for k in range(test_num):
            mag_world_test = np.hstack(
                (mag_world_test, rotation[train_num + k, :, :].dot(mag_data[:, train_num + k:train_num + k + 1])))
        pos_test = np.hstack((pos_test, pos[:, train_num:train_num + test_num]))

        data1 = np.hstack((pos_train.T, mag_world_train.T*1e-7))
        data2 = np.hstack((pos_test.T, mag_world_test.T * 1e-7))
        data = np.vstack((data1, data2))
        # # 打开文件，准备写入
        # print(i)

        mag_world_test_df = pd.DataFrame(data)
        mag_world_test_df.to_csv("data.csv", index=False, header=False)

if __name__ == "__main__":
    # M = cal_moment_swtich("./Data/Take 2023-07-28 02.21.44 PM_018(Z=185mm).csv",
    #                       "./Data/Take 2023-07-28 02.21.44 PM_018(Z=185mm).mat")
    # print(M)

    # write_train_data("Data/Take 2023-07-28 02.21.44 PM_018(Z=185mm).csv", "Data/Take 2023-07-28 02.21.44 PM_018(Z=185mm).mat")

    write_train_data()
