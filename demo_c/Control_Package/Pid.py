
class Pid:
    def __init__(self, kp=1, ki=0, kd=0, dt=0.01, coefficient=1):
        """
        :param kp: 比例系数 浮点型
        :param ki: 积分系数 浮点型
        :param kd: 微分系数 浮点型
        :param dt: 离散时间常数 浮点型
        :param coefficient: 将比例-积分-微分系数都乘以一个常数 浮点型
        """
        self.__error_last = 0
        self.__error_acc = 0
        self.__error_der = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.coefficient = coefficient

    def pid(self, error):
        """
        每个离散步长得到的误差，都传入到pid这个函数里
        :param error: 误差值 array类型/浮点类型
        :return: 返回pid计算后得到的结果 array类型/浮点类型
        """
        self.__error_acc = self.__error_acc + error * self.dt
        self.__error_der = (error - self.__error_last) / self.dt
        self.__error_last = error

        f = (self.kp * error + self.ki * self.__error_acc + self.kd * self.__error_der) * self.coefficient
        return f
