import numpy as np
import math
# def calculate_angles(m_hat):
#
#     r = math.sqrt(m_hat[0,0] ** 2 + m_hat[1,0] ** 2 + m_hat[2,0] ** 2)
#     if r ==0:
#         return np.array([[0],[0]])
#     else:
#         alpha = math.acos(m_hat[2,0] / r)  # 俯仰角
#         theta = math.atan2(m_hat[1,0], m_hat[0,0])
#         X=np.array([[alpha],[theta]])
#
#     # 方位角，需要根据象限调整
#         return X
def calculate_angles(m):
    """
    磁矩转换成角度
    :param m: 磁矩 np.array 3*1
    :return: 角度phi和theta float
    """
    phi = math.atan2(math.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2), m[2, 0])
    theta = math.atan2(m[1, 0], m[0, 0])
    return np.array([[phi],[theta]])

class position_controller:
    def __init__(self,ePc1,ePc2,emc1,emc2,force1,feild1,force2,feild2,m,dt):
        self.ePc1 = ePc1
        self.ePc2 = ePc2
        self.emc1 = emc1
        self.emc2 = emc2
        self.dt = dt
        self.force1 = force1
        self.feild1 = feild1
        self.force2 = force2
        self.feild2 = feild2
        self.m = m
        self.ix1 = 0
        self.ix2 = 0
        self.im1 = 0
        self.im2 = 0
        self.lx1 = 0
        self.lx2 = 0
        self.lm1 = 0
        self.lm2 = 0

    def physics(self,v1_last,v2_last,Pc1,Pc2):
        a1 = (self.force1 - 0.02*v1_last + np.array([[0],[0],[-0.0005]]))/self.m
        a2 = (self.force2 - 0.02 * v2_last +np.array([[0],[0],[-0.0005]]) ) / self.m
        v1_now = v1_last +a1*self.dt
        v2_now = v2_last +a2*self.dt
        x1_now = Pc1 +v1_last*self.dt
        x2_now = Pc2 +v2_last*self.dt
        return v1_now,v2_now,x1_now,x2_now



    def pid_controller(self,Pc1,Pc2,mc1,mc2,Kp1,Kp2,Ki,kd, Kpm):
        xerr1 =self.ePc1- Pc1
        xerr2 =self.ePc2 -Pc2
        merr1 = calculate_angles(self.emc1 )- calculate_angles(mc1)
        merr2 = calculate_angles(self.emc2 )- calculate_angles(mc2)
        ea1 = calculate_angles(self.emc1)
        ea2 = calculate_angles(self.emc2)
        # Kpm = [[0],[0.0]]
        # print(3333333333333333333)
        # print(calculate_angles(mc1))
        # print(mc1)
        if abs(ea1[0,0] - 3.1415) <=0.00001 or abs(ea1[0,0])<0.001:
            print(11111111111)
            Kpm1 = [[Kpm],[0]]
        else:
            Kpm1 = Kpm


        p_termx1 = Kp1 * xerr1
        p_termx2 = Kp2 * xerr2
        p_termm1 = Kpm1 * merr1
        p_termm2 = Kpm1 * merr2
        self.ix1 += xerr1*self.dt
        self.ix2 += xerr2 * self.dt
        self.im1 += merr1 * self.dt
        self.im2 += merr2 * self.dt
        i_termx1 = Ki * self.ix1
        i_termx2 = Ki * self.ix2
        i_termm1 = 0 * self.im1
        i_termm2 = 0 * self.im2
        d_termx1 = (xerr1 - self.lx1) / self.dt*kd
        d_termx2 = (xerr2 - self.lx2) / self.dt * kd
        d_termm1 = (merr1 - self.lm1) / self.dt * 0
        d_termm2 = (merr2 - self.lm2) / self.dt * 0

        F1 =p_termx1+i_termx1+d_termx1
        F2 =p_termx2+i_termx2+d_termx2
        M1 =p_termm1+i_termm1+d_termm1
        M2 =p_termm2+i_termm2+d_termm2
        self.lx1 = xerr1
        self.lx2 = xerr2
        self.lm1 = merr1
        self.lm2 = merr2
        return F1,F2,M1,M2



class position_controller_single:
    def __init__(self,ePc1,emc1,force1,feild1,m,dt):
        self.ePc1 = ePc1
        self.emc1 = emc1
        self.dt = dt
        self.force1 = force1
        self.feild1 = feild1
        self.m = m
        self.ix1 = 0
        self.ix2 = 0
        self.im1 = 0
        self.im2 = 0
        self.lx1 = 0
        self.lx2 = 0
        self.lm1 = 0
        self.lm2 = 0

    def pid_controller(self,Pc1,mc1,Kp1,Ki,kd, Kpm):
        xerr1 =self.ePc1 - Pc1
        merr1 = calculate_angles(self.emc1 )- calculate_angles(mc1)
        # Kpm = [[0],[0.0]]
        ea1 = calculate_angles(self.emc1)
        if abs(ea1[0,0] - 3.1415) <=0.01 or abs(ea1[0,0])<0.001:
            Kpm1 = [[Kpm],[0]]
        else:
            Kpm1 = Kpm

        p_termx1 = Kp1 * xerr1
        p_termm1 = Kpm * merr1
        self.ix1 += xerr1*self.dt
        self.im1 += merr1 * self.dt
        i_termx1 = Ki * self.ix1
        i_termm1 = 0 * self.im1
        d_termx1 = (xerr1 - self.lx1) / self.dt*kd
        d_termm1 = (merr1 - self.lm1) / self.dt * 0

        F1 =p_termx1+i_termx1+d_termx1
        M1 =p_termm1+i_termm1+d_termm1
        self.lx1 = xerr1
        self.lm1 = merr1
        return F1, M1


print(calculate_angles(np.array([[0], [0], [1]])))