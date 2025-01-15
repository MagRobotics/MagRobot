import numpy as np
from Pose_Transform import pose_transform
import random

def rotation_maxwell(D, r_matrix):
    r = np.mat(r_matrix)
    r1 = np.mat(np.linalg.inv(r_matrix))
    # r1 = np.mat(r_matrix.transpose())
    # print(r)
    # print(r1)
    return r * np.mat(D) * r1


def get_result_matrix(D_list):
    result = np.array([[], [], [], [], []])
    i = 0
    while (i < len(D_list)):
        temp = np.array([[D_list[i][0, 0]], [D_list[i][0, 1]], [D_list[i][0, 2]], [D_list[i][1, 1]], [D_list[i][1, 2]]])
        result = np.hstack((result, temp))
        i = i + 1
    return result


if __name__ == "__main__":
    D = np.array([[2, 0, 0], [0, -1, 0], [0, 0, -1]])
    euler = np.array([[0, 0, 0], [45, 45, 90], [0, 90, 0]])

    r_matrix = pose_transform.euler2_to_rot(euler[0])
    # print(r_matrix)
    D1 = rotation_maxwell(D, r_matrix)
    print(D1)

    r_matrix = pose_transform.euler2_to_rot(euler[1])
    # print(r_matrix)
    D2 = rotation_maxwell(D, r_matrix)
    print(D2)

    r_matrix = pose_transform.euler2_to_rot(euler[2])
    D3 = rotation_maxwell(D, r_matrix)
    print(D3)

    D_list = [D1, D2, D3]
    A = get_result_matrix(D_list)
    # print(A)
    print(np.linalg.matrix_rank(A))
    # result = [0, 0, 0]
    sum_ = 0
    num = 3000
    data = []
    for i in range(num):
        a = random.uniform(0.0, np.pi)
        b = random.uniform(0.0, 2*np.pi)
        m = pose_transform.angle_to_moment(a, b)

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

        matrix = np.mat(M)*np.mat(A)

        det = abs(np.linalg.det(matrix))
        sum_ = sum_ + det
        data.append(det)

    data = sorted(data)
    print(data)
    print(sum_/num)


# if __name__ == "__main__":
#     D = np.array([[-0.0051, 0, 0], [0, -0.0051, 0], [0, 0, 0.0103]])
#     euler = np.array([[45, 0, 0], [0, 0, 0], [0, 0, 0]])
#
#     r_matrix = pose_transform.euler2_to_rot(euler[0])
#     # print(r_matrix)
#     D1 = rotation_maxwell(D, r_matrix)
#     print(D1)
#
#     r_matrix = pose_transform.euler2_to_rot(euler[1])
#     # print(r_matrix)
#     D2 = rotation_maxwell(D, r_matrix)
#     print(D2)
#
#     r_matrix = pose_transform.euler2_to_rot(euler[2])
#     D3 = rotation_maxwell(D, r_matrix)
#     print(D3)
#
#     D_list = [D1, D2, D3]
#     A = get_result_matrix(D_list)
#     # print(A)
#     print(np.linalg.matrix_rank(A))
#     # result = [0, 0, 0]
#     sum_ = 0
#     num = 1000
#     data = []
#     for i in range(num):
#         a = random.uniform(0.0, np.pi)
#         b = random.uniform(0.0, 2*np.pi)
#         m = pose_transform.angle_to_moment(a, b)
#
#         M = np.zeros((3, 5))
#         M[0, 0] = m[0, 0]
#         M[1, 0] = 0
#         M[2, 0] = -m[2, 0]
#
#         M[0, 1] = m[1, 0]
#         M[1, 1] = m[0, 0]
#         M[2, 1] = 0
#
#         M[0, 2] = m[2, 0]
#         M[1, 2] = 0
#         M[2, 2] = m[0, 0]
#
#         M[0, 3] = 0
#         M[1, 3] = m[1, 0]
#         M[2, 3] = -m[2, 0]
#
#         M[0, 4] = 0
#         M[1, 4] = m[2, 0]
#         M[2, 4] = m[1, 0]
#
#         matrix = np.mat(M)*np.mat(A)
#
#         det = abs(np.linalg.det(matrix))
#         sum_ = sum_ + det
#         data.append(det)
#
#     data = sorted(data)
#     print(data)
#     print(sum_/num)
