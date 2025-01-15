import numpy as np

class Sofa_Interface:
    def set_force_moment(self, child, force, moment):
        """
        设置child的力和力矩
        :param child:
        :param force: 力 np.array 3*1
        :param moment: 力矩 np.array 3*1
        :return: 无返回值
        """
        # child.forces[0][:] = [0,0,0,0,0,0]
        child.forces[0][:] = [force[0, 0], force[1, 0], force[2, 0], moment[0, 0], moment[1, 0], moment[2, 0]]

    def get_velocity(self, child):
        """
        获得child的速度
        :param child:
        :return: 速度 np.array 3*1
        """
        return np.array([[child[0]], [child[1]], [child[2]], [0], [0], [0]])
