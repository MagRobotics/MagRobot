import numpy as np
from scipy.optimize import leastsq
from Pose_Transform.pose_transform import *
from State_Estimation.Extended_Kalman_Filter import *
from State_Estimation.Optimization_Method import *
from State_Estimation.estimate_custom import *
import time


def Positioning_EKF(sensor, instrument_pos, mc_norm, pos_filter, P, EKF_Q, EKF_R):
    # sen0 = np.array([[sensor[0]["pose_sensor"][0], sensor[0]["pose_sensor"][1], sensor[0]["pose_sensor"][2]], [0, 0, 0], [1, 0, 0]])
    # sen1 = np.array([[sensor[1]["pose_sensor"][0], sensor[1]["pose_sensor"][1], sensor[1]["pose_sensor"][2]], [0, 0, 0], [1, 0, 0]])
    # sen2 = np.array([[sensor[2]["pose_sensor"][0], sensor[2]["pose_sensor"][1], sensor[2]["pose_sensor"][2]], [0, 0, 0], [1, 0, 0]])
    # sen3 = np.array([[sensor[3]["pose_sensor"][0], sensor[3]["pose_sensor"][1], sensor[3]["pose_sensor"][2]], [0, 0, 0], [1, 0, 0]])

    Pj = instrument_pos[0:3]
    quat2 = instrument_pos[3:7]
    r2 = R.from_quat(quat2)
    mj1 = r2.apply([1, 0., 0.]) * mc_norm  # 保证是单位矩阵
    pc = np.array([Pj[0], Pj[1], Pj[2], mj1[0], mj1[1], mj1[2]])
    # print(pc)
    # z = Getz(pc, sen0, sen1, sen2, sen3, sensor)
    z = Getz(pc, sensor)
    x_new, P[:, :] = KalmanFilter(z, pos_filter, P, sensor, EKF_Q, EKF_R)
    pos_filter[:, :] = x_new.astype(np.float64)
    # print(pos_filter)
    moment = np.array([[pos_filter[6, 0]], [pos_filter[7, 0]], [pos_filter[8, 0]]])
    axis, angle = angle_to_axis_radian(moment_to_angle(m=moment)[0], moment_to_angle(m=moment)[1])
    r1 = axis_radian_to_rmatrix(axis, angle)
    qua1 = rmatrix_to_quat(r1)


def Position_Optimization_gradient(sensor, instrument_pos, mc_norm, pos_gra, alpha, beta, gra_num):
    pa_real = np.array([[instrument_pos[0]], [instrument_pos[1]], [instrument_pos[2]]])
    quat = instrument_pos[3:7]
    r = R.from_quat(quat)
    mj1 = r.apply([1, 0., 0.]) * mc_norm  # 保证是单位矩阵
    ma_real = np.array([[mj1[0]], [mj1[1]], [mj1[2]]])
    pc = np.zeros((3, len(sensor)))
    R_sensor = np.zeros((len(sensor), 3, 3))
    sensitivity = np.array([])

    for i in range(len(sensor)):
        pc[:, i] = sensor[i]["pose_sensor"][0:3]
        quat2 = sensor[i]["pose_sensor"][3:7]
        R_sensor[i] = R.from_quat(quat2).as_matrix()
        sensitivity = np.append(sensitivity, sensor[i]["sensitivity"])

    for _ in range(gra_num):
        pa = pos_gra[0:3]
        ma = pos_gra[3:6]
        grad, loss = gradient(alpha, pc, R_sensor, pa, ma, pa_real, ma_real, sensitivity)
        pos_gra[:, :] = pos_gra - grad * beta
    #     print(loss)
    # print(pa_real)
    # print(pos_gra)


def Position_Optimization_LM(sensor, instrument_pos, mc_norm, pos_LM, LM_num):
    pa_real = np.array([[instrument_pos[0]], [instrument_pos[1]], [instrument_pos[2]]])
    quat = instrument_pos[3:7]
    r = R.from_quat(quat)
    mj1 = r.apply([1, 0., 0.]) * mc_norm  # 保证是单位矩阵
    ma_real = np.array([[mj1[0]], [mj1[1]], [mj1[2]]])
    pc = np.zeros((3, len(sensor)))
    R_sensor = np.zeros((len(sensor), 3, 3))
    sensitivity = np.array([])

    for i in range(len(sensor)):
        pc[:, i] = sensor[i]["pose_sensor"][0:3]
        quat2 = sensor[i]["pose_sensor"][3:7]
        R_sensor[i] = R.from_quat(quat2).as_matrix()
        sensitivity = np.append(sensitivity, sensor[i]["sensitivity"])
    true_params = np.array([pa_real[0, 0], pa_real[1, 0], pa_real[2, 0], ma_real[0, 0], ma_real[1, 0], ma_real[2, 0]])
    noise = np.zeros(3 * len(sensor))
    for i in range(len(sensor)):
        noise[3 * i:3 * (i + 1)] = np.random.normal(0, sensitivity[i], 3)

    field_sensor = field_LM(true_params, pc, R_sensor) + noise

    initial_guess = np.array([pos_LM[0, 0], pos_LM[1, 0], pos_LM[2, 0], pos_LM[3, 0], pos_LM[4, 0], pos_LM[5, 0]])

    fit_params = leastsq(field_residual_LM, initial_guess, args=(pc, R_sensor, field_sensor), maxfev=LM_num)

    pos_LM[:, :] = np.array([[fit_params[0][0]], [fit_params[0][1]], [fit_params[0][2]], [fit_params[0][3]], [fit_params[0][4]], [fit_params[0][5]]])
    # print(true_params)
    # print(pos_LM)


def Position_Custom(sensor, instrument_pos, mc_norm):

    p, h = custom(sensor, instrument_pos, mc_norm)

    return p, h