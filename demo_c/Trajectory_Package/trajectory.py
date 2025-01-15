import numpy as np
import math


def angle_to_moment(phi, theta):
    """
    :param phi:
    :param theta:
    :return: 返回磁矩的列向量（mat形式）
    """
    return np.array([[math.sin(phi) * math.cos(theta)], [math.sin(phi) * math.sin(theta)], [math.cos(phi)]])


def soft_trajectory(s, v, acc, time):
    t_constant = (s - v ** 2 / acc) / v
    if time <= v / acc:
        return 0.5 * acc * time ** 2
    if v / acc < time < v / acc + t_constant:
        return v ** 2 / (2 * acc) + v * (time - v / acc)
    if v / acc + t_constant <= time <= v / acc * 2 + t_constant:
        return v ** 2 / (2 * acc) + v * t_constant + v * (time - v / acc - t_constant) - 0.5 * acc * (
                time - v / acc - t_constant) ** 2


def polynomial_trajectory(theta_f, tf, time, theta_0=0., theta_0_dot=0., theta_0_dotdot=0., theta_f_dot=0.,
                          theta_f_dotdot=0.):
    a0 = theta_0
    a1 = theta_0_dot
    a2 = theta_0_dotdot / 2
    a3 = (20 * theta_f - 20 * theta_0 - (8 * theta_f_dot + 12 * theta_0_dot) * tf - (
            3 * theta_0_dotdot - theta_f_dotdot) * tf ** 2) / (2 * tf ** 3)
    a4 = (30 * theta_0 - 30 * theta_f + (14 * theta_f_dot + 16 * theta_0_dot) * tf + (
            3 * theta_0_dotdot - 2 * theta_f_dotdot) * tf ** 2) / (2 * tf ** 4)
    a5 = (12 * theta_f - 12 * theta_0 - (6 * theta_f_dot + 6 * theta_0_dot) * tf - (
            theta_0_dotdot - theta_f_dotdot) * tf ** 2) / (2 * tf ** 5)
    return a0 + a1 * time + a2 * time ** 2 + a3 * time ** 3 + a4 * time ** 4 + a5 * time ** 5


def polynomial_three(theta_f, tf, time, theta_0=0, theta_0_dot=0, theta_f_dot=0):
    a0 = theta_0
    a1 = 0
    a2 = 3 * (theta_f - theta_0) / (tf ** 2)
    a3 = -2 * (theta_f - theta_0) / (tf ** 3)
    return a0 + a1 * time + a2 * time ** 2 + a3 * time ** 3


def linear(theta_f, tf, time):
    a1 = theta_f / tf
    return a1 * time


if __name__ == "__main__":
    pass

