import numpy as np


def custom(sensor, instrument_pos, mc_norm):

    p = np.array([[0], [0], [0]])
    h = np.array([[0], [0], [1]])

    return p, h