import numpy as np
import time


def field(pa, ma, pc, R):
    p = pc - pa
    k = 4 * np.pi * 1e-7
    ma = np.mat(ma)
    p = np.mat(p)
    # print("aa")
    # 磁源在某点产生的磁场强度
    field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma
    # print("bb")
    field = R.transpose() * field
    # print("cc")
    return field


def field_jacob(pa, ma, pc, R):
    X = np.mat(np.concatenate((pa, ma), axis=0))
    h = 0.00001
    grad = np.mat(np.zeros([3, 6]))

    for idx in range(X.size):
        tmp_val = X[idx, 0]

        X[idx, 0] = tmp_val + h
        pa = X[0:3, 0:1]
        ma = X[3:6, 0:1]
        field1 = field(pa, ma, pc, R)

        X[idx, 0] = tmp_val - h
        pa = X[0:3, 0:1]
        ma = X[3:6, 0:1]
        field2 = field(pa, ma, pc, R)

        grad[:3, idx] = (field1 - field2) / (2 * h)
        X[idx, 0] = tmp_val
    return np.mat(grad)


def gradient(alpha, pc, R, pa, ma, pa_real, ma_real, sensitivity):
    """
    :param alpha: list 1*4
    :param pc: np.array 3*4
    :param R: np.array 4*3*3 4代表四个传感器
    :param pa:
    :param ma:
    :param pa_real:
    :param ma_real:
    :return:
    """
    b_real = np.zeros((3, len(alpha)))
    for i in range(len(alpha)):
        b_real[:, i:i + 1] = field(pa_real, ma_real, pc[:, i:i + 1], R[i, :, :]) + np.random.normal(0, sensitivity[i], (3, 1))
    # print(b_real)
    # print(len(alpha))

    b = np.zeros((3, len(alpha)))
    for i in range(len(alpha)):
        b[:, i:i + 1] = field(pa, ma, pc[:, i:i + 1], R[i, :, :])

    loss = 0
    for i in range(len(alpha)):
        loss += np.linalg.norm(b[:, i:i + 1] - b_real[:, i:i + 1]) ** 2 * alpha[i]

    jacob = np.zeros((len(alpha), 3, 6))
    for i in range(len(alpha)):
        jacob[i, :, :] = field_jacob(pa, ma, pc[:, i:i + 1], R[i, :, :])

    gradient = np.zeros((1, 6))
    for i in range(len(alpha)):
        gradient += 2 * (b[:, i:i + 1] - b_real[:, i:i + 1]).transpose().dot(jacob[i, :, :]) * alpha[i]

    gradient = gradient.transpose()

    return gradient, loss


def LM_method(alpha, pc, R, pa, ma, pa_real, ma_real, u):
    """
    :param alpha: list 1*4
    :param pc: np.array 3*4
    :param R: np.array 4*3*3 4代表四个传感器
    :param pa:
    :param ma:
    :param pa_real:
    :param ma_real:
    :return:
    """
    b_real = np.zeros((3, 4))
    for i in range(4):
        b_real[:, i:i + 1] = field(pa_real, ma_real, pc[:, i:i + 1], R[i, :, :]) + np.random.normal(0, 1e-6, (3, 1))
    # print(b_real)

    b = np.zeros((3, 4))
    for i in range(4):
        b[:, i:i + 1] = field(pa, ma, pc[:, i:i + 1], R[i, :, :])

    loss = 0
    for i in range(4):
        loss += np.linalg.norm(b[:, i:i + 1] - b_real[:, i:i + 1]) ** 2 * alpha[i]

    jacob = np.zeros((4, 3, 6))
    for i in range(4):
        jacob[i, :, :] = field_jacob(pa, ma, pc[:, i:i + 1], R[i, :, :])

    gradient = np.zeros((1, 6))
    for i in range(4):
        gradient += 2 * (b[:, i:i + 1] - b_real[:, i:i + 1]).transpose().dot(jacob[i, :, :]) * alpha[i]

    gradient_trans = gradient.transpose()

    result = np.linalg.inv(gradient_trans.dot(gradient) + u * np.eye(6)).dot(gradient_trans) * loss

    return result, loss


def initial_u(alpha, pc, R, pa, ma, pa_real, ma_real):
    """
    :param alpha: list 1*4
    :param pc: np.array 3*4
    :param R: np.array 4*3*3 4代表四个传感器
    :param pa:
    :param ma:
    :param pa_real:
    :param ma_real:
    :return:
    """
    b_real = np.zeros((3, 4))
    for i in range(4):
        b_real[:, i:i + 1] = field(pa_real, ma_real, pc[:, i:i + 1], R[i, :, :]) + np.random.normal(0, 1e-6, (3, 1))
    # print(b_real)

    b = np.zeros((3, 4))
    for i in range(4):
        b[:, i:i + 1] = field(pa, ma, pc[:, i:i + 1], R[i, :, :])

    jacob = np.zeros((4, 3, 6))
    for i in range(4):
        jacob[i, :, :] = field_jacob(pa, ma, pc[:, i:i + 1], R[i, :, :])

    gradient = np.zeros((1, 6))
    for i in range(4):
        gradient += 2 * (b[:, i:i + 1] - b_real[:, i:i + 1]).transpose().dot(jacob[i, :, :]) * alpha[i]

    gradient_trans = gradient.transpose()

    A = gradient_trans.dot(gradient)
    max_num = -1e100
    for i in range(6):
        if A[i, i] > max_num:
            max_num = A[i, i]
    return max_num

#LM残差处理
def field_LM(params, pc, R):
    # print(pc)
    b = np.zeros(3*len(R))
    params = np.array([[params[0]], [params[1]], [params[2]], [params[3]], [params[4]], [params[5]]])
    for i in range(len(R)):
        # print(params.size)
        p = pc[:, i:i + 1] - np.array([[params[0, 0]], [params[1, 0]], [params[2, 0]]])
        # print(params[0:3])
        k = 4 * np.pi * 1e-7
        ma = np.mat(params[3:6, :])
        p = np.mat(p)
        # print("aa")
        # 磁源在某点产生的磁场强度
        field = k / (4 * np.pi * pow(np.linalg.norm(p), 5)) * (3 * (p * p.T) - pow(np.linalg.norm(p), 2) * np.eye(3)) * ma
        # print("bb")
        field = R[i, :, :].transpose() * field
        # print("cc")
        b[3*i:3*(i+1)] = np.array([field[0, 0], field[1, 0], field[2, 0]])
    return b

def field_residual_LM(params, pc, R, y_observed):
    return field_LM(params, pc, R).T - y_observed
#LM残差处理


if __name__ == '__main__':
    # pc = np.array([[0.025, 0.025, -0.025, -0.025], [0.025, -0.025, 0.025, -0.025], [0, 0, 0, 0]])
    pc = np.array([[-0.2526, -0.2, -0.2526, -0.1474], [-0.026, -0.0, 0.026, -0.026], [0.926, 0.926, 0.926, 0.926]])
    R = np.array(
        [[[1, 0, 0], [0, 1, 0], [0, 0, 1]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
         [[1, 0, 0], [0, 1, 0], [0, 0, 1]]])
    # pa_real = np.array([[0], [0], [0.05]])
    # ma_real = np.array([[0], [0], [0.126]])
    pa_real = np.array([[-0.2], [0], [1.1]])
    ma_real = np.array([[0], [0], [-0.126]])
    alpha = [0.25, 0.25, 0.25, 0.25]

    # pa_0 = np.array([[0], [0], [0.0505]])
    # ma_0 = np.array([[0], [0], [0.126]])
    pa_0 = np.array([[-0.2], [0], [1.1]])
    ma_0 = np.array([[0], [0], [-0.126]])
    X = np.mat(np.concatenate((pa_0, ma_0), axis=0))
    # initial = np.array([[0, 0, 0.08, 0, 0, 10]])
    # loss = float('inf')
    loss = 1e100
    beta = 3000000
    # loss_stop = 5.2170205182317816e-09
    loss_step = 2.0036296597783228e-10000

    start_time = time.time()
    n = 0

    # method = "GD"

    method = "GD"
    u = initial_u(alpha, pc, R, pa_0, ma_0, pa_real, ma_real)
    # u = 0.01
    loss_last = 1e100
    grad_last = None
    flag = 1

    while loss > loss_step:
        pa = X[0:3, 0]
        ma = X[3:6, 0]
        if int((time.time() - start_time) / 0.01) > n:
            pa_real[2, 0] = pa_real[2, 0] - 0.0001
            pa_real[0, 0] = pa_real[0, 0]
            print("------------------------------------------")
            print(pa_real)
            print(ma_real)
            print("------------------------------------------")
            n = int((time.time() - start_time) / 0.01)

        if method == "GD":
            grad, loss = gradient(alpha, pc, R, pa, ma, pa_real, ma_real, [1e-10,1e-10,1e-10,1e-10])
            # print(grad)
            X = X - grad * beta
        elif method == "LM":
            grad, loss = LM_method(alpha, pc, R, pa, ma, pa_real, ma_real, u)
            if loss > loss_last:
                # 如果优化目标变大，则拒绝本次迭代，重新调整u的值
                if flag == 1:
                    X = X + grad_last
                u *= 1.1
                flag = 0
            else:
                X = X - grad
                loss_last = loss
                grad_last = grad
                flag = 1
        print("***************")
        print(X)
        print(loss)
        print("ssssssssssssssss")
