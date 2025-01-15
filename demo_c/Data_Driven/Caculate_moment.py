import pandas as pd
import numpy as np
import json


# 分文件存储数据
# def cal_moment_swtich(inputs, outputs):
#     """
#     # 用训练数据来拟合磁矩
#     @param inputs: 目标位置相对于磁铁的位置
#     @param outputs: 目标位置的磁场
#     @return: 标定结果（磁矩）
#     """
#     # 进行拟合的数据量，看训练集里面有多少数据
#
#     pos = np.array(pd.read_csv(inputs, header=None)).transpose()
#     mag_data = np.array(pd.read_csv(outputs, header=None)).transpose()
#
#     num = pos.shape[1]
#     # print(num)
#     miu = np.pi * 4e-7
#     k = miu / (4 * np.pi)
#     B = None
#     T_matrix = None
#     for i in range(num):
#         x = pos[0, i]
#         y = pos[1, i]
#         z = pos[2, i]
#         r = np.linalg.norm(pos[:, i])
#         T = np.array([[3 * x ** 2 - r ** 2, 3 * x * y, 3 * x * z], [3 * x * y, 3 * y ** 2 - r ** 2, 3 * y * z],
#                       [3 * x * z, 3 * y * z, 3 * z ** 2 - r ** 2]]) * (k / r ** 5)
#
#         # 记录的磁场数据是毫高斯
#         if i == 0:
#             B = mag_data[:, i:i + 1]
#             T_matrix = T
#         else:
#             B = np.vstack((B, mag_data[:, i:i + 1]))
#             T_matrix = np.vstack((T_matrix, T))
#         # if i%2000 == 0:
#         #     print(i)
#
#     M = np.linalg.pinv(T_matrix).dot(B)
#
#     return M

def cal_moment_swtich(data):
    """
    # 用训练数据来拟合磁矩
    @param inputs: 目标位置相对于磁铁的位置
    @param outputs: 目标位置的磁场
    @return: 标定结果（磁矩）
    """
    # 进行拟合的数据量，看训练集里面有多少数据

    data = np.array(pd.read_csv(data, header=None)).transpose()

    num = data.shape[1]
    print(num)
    miu = np.pi * 4e-7
    k = miu / (4 * np.pi)
    B = None
    T_matrix = None
    for i in range(num):
        x = data[0, i]
        y = data[1, i]
        z = data[2, i]
        r = np.linalg.norm(data[0:3, i])
        T = np.array([[3 * x ** 2 - r ** 2, 3 * x * y, 3 * x * z], [3 * x * y, 3 * y ** 2 - r ** 2, 3 * y * z],
                      [3 * x * z, 3 * y * z, 3 * z ** 2 - r ** 2]]) * (k / r ** 5)

        # 记录的磁场数据是毫高斯
        if i == 0:
            B = data[3:6, i:i + 1]
            T_matrix = T
        else:
            B = np.vstack((B, data[3:6, i:i + 1]))
            T_matrix = np.vstack((T_matrix, T))
        if i%2000 == 0:
            print(i)

    M = np.linalg.pinv(T_matrix).dot(B)

    return M


if __name__ == "__main__":
    # M = cal_moment_swtich("./train_x.csv", "./train_y.csv")
    f = open("..\\..\\demo_c\\GUI_interface\\ceshi.txt", "r", encoding="UTF-8")
    # f = open("C:\\Users\\10145\\Desktop\\realgui_2\\demo_c\\GUI_interface\\ceshi.txt", "r", encoding="UTF-8")
    json_str = f.read()
    initial_info = json.loads(json_str)
    # print(initial_info[4][0]["calibration_input"])
    M = cal_moment_swtich(initial_info[4][0]["calibration_input"])
    print(M)
    # M = cal_moment_swtich("./data.csv")
    # print(M)
    with open("..\\..\\demo_c\\Data_Driven\\moment.txt", 'w') as f:
        # print("--------------------------------")
        f.write(str(np.linalg.norm(M)))
