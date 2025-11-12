import Sofa
from Module_Package.animate_module import *
from solve22 import *
# dt = 0.03


class MagController(Sofa.Core.Controller):

    def __init__(self, root_node, info, scene, path, pose_estimate, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.scene = scene
        self.root_node = root_node
        self.i = 0
        self.path = path
        self.pose_estimate = pose_estimate
        self.info = info
        self.draw = list()

        self.time_list = []
        self.force_desired_list = []
        self.field_desired_list = []
        self.force_real_list = []
        self.field_real_list = []
        self.current_list = []
        self.flag = [0]

        self.data_t = []
        self.data_p_real = []
        self.data_p_desired = []
        self.angle_list = []
        self.angle_desired_list = []

        self.sofa_interface = Sofa_Interface()


        self.ePc1 = np.array([[0.10],[0.1],[0.065]])
        self.ePc2 = np.array([[0.25],[0.1],[0.065]])
        self.emc1_hat = np.array([[0], [0], [-1]])
        self.emc2_hat = np.array([[0], [0], [1]])

        self.F1_0, self.F2_0 = np.array([[0], [0], [0]]), np.array([[0], [0], [0]])
        self.M1_0, self.M2_0 = np.array([[0], [0]]), np.array([[0], [0]])

        self.force1, self.feild1, self.force2, self.feild2 = np.array([[0], [0], [0]]), np.array([[0], [0], [0]]), np.array([[0], [0], [0]]), np.array([[0], [0], [0]])
        self.pc1, self.pc2 = np.array([[0.10],[0.1],[0.065]]), np.array([[0.25],[0.1],[0.065]])
        self.v1_last = np.array([[0.0], [0.0], [0]])
        self.v2_last = np.array([[0.0], [0.0], [0]])

        self.pa1, self.ma_hat1 = self.scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
        self.pa2, self.ma_hat2 = self.scene.magnetic_source[1]['robot'].fkine(scene.magnetic_source[1]['theta'])

        self.x1_now = np.array([[0.10],[0.1],[0.065]])
        self.x2_now = np.array([[0.25],[0.1],[0.065]])
        self.v1_now = np.array([[0.0], [0.0], [0]])
        self.v2_now = np.array([[0.0], [0.0], [0]])


        self.ma1_norm = 19.5
        self.ma2_norm = 19.5
        self.mc1_norm = 0.1015
        self.mc2_norm = 0.1015
        self.mc1_hat = np.array([[0.0], [0.0], [-1]])
        self.mc2_hat = np.array([[0.0], [0.0], [1]])
        self.force1, self.feild1, moment1, self.force2, self.feild2, moment2 = calculate_force_now(self.pa1, self.pa2, self.pc1, self.pc2,
                                                                                 self.ma1_norm * self.ma_hat1,
                                                                                 self.ma2_norm * self.ma_hat2,
                                                                                 self.mc1_hat * self.mc1_norm, self.mc2_hat * self.mc2_norm)
        self.position = position_controller(self.ePc1, self.ePc2, self.emc1_hat, self.emc2_hat, self.force1, self.feild1,
                                       self.force2, self.feild2, 2.89e-3, 0.1)

        self.t = 0.0

        self.position_x1 = []
        self.position_y1 = []
        self.position_z1 = []
        self.position_m1 = []
        self.position_n1 = []
        self.position_p1 = []
        self.plan_x1 = []
        self.plan_y1 = []
        self.plan_z1 = []
        self.plan_m1 = []
        self.plan_n1 = []
        self.plan_p1 = []

        self.position_x2 = []
        self.position_y2 = []
        self.position_z2 = []
        self.position_m2 = []
        self.position_n2 = []
        self.position_p2 = []
        self.plan_x2 = []
        self.plan_y2 = []
        self.plan_z2 = []
        self.plan_m2 = []
        self.plan_n2 = []
        self.plan_p2 = []


        # self.ePc1 = np.array([[0.3], [0.1], [0]])
        # self.emc1_hat = np.array([[0], [0], [1]])
        #
        # self.F1_0 = np.array([[0], [0], [0]])
        # self.M1_0 = np.array([[0], [0]])
        #
        # self.force1, self.feild1 = np.array([[0], [0], [0]]), np.array([[0], [0], [0]])
        # self.pc1 = np.array([[0.3], [0.1], [0]])
        # self.v1_last = np.array([[0.0], [0.0], [0]])
        #
        # self.pa1, self.ma_hat1 = self.scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
        # self.pa2, self.ma_hat2 = self.scene.magnetic_source[1]['robot'].fkine(scene.magnetic_source[1]['theta'])
        #
        # self.x1_now = np.array([[0.3], [0.1], [0]])
        # self.v1_now = np.array([[0.0], [0.0], [0]])
        #
        #
        # self.ma1_norm = 19.5
        # self.ma2_norm = 19.5
        # self.mc1_norm = 0.1015
        # self.mc1_hat = np.array([[0.0], [0.0], [1]])
        #
        # self.force1, self.feild1, moment = calculate_force_now_single(self.pa1, self.pa2, self.pc1,
        #                                                                          self.ma1_norm * self.ma_hat1,
        #                                                                          self.ma2_norm * self.ma_hat2,
        #                                                                          self.mc1_hat * self.mc1_norm)
        # print('2222222222222')
        # print(self.scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta']))
        # # print("pa1:", self.pa1)
        # # print("ma_hat1:", self.ma_hat1)
        # # print("pa2:", self.pa2)
        # # print("ma_hat2:", self.ma_hat2)
        # self.position = position_controller_single(self.ePc1, self.emc1_hat, self.force1, self.feild1, 2.89e-3, 0.1)



    def onAnimateBeginEvent(self, event):
        # self.i += self.scene.dt*100
        # # print("self.i:", self.i)
        # # print("test:", self.path.p_desired)
        # # print("idtest:", id(self.path.p_desired))
        #
        # if self.info[1] == []:
        #     Create_Animate_Location(self.scene, self.i, self.path, self.pose_estimate)
        #
        # # 丝杆+电磁铁控制胶囊
        # elif 'lead_screw_electromagnet' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule":
        #     Create_Animate_Capsule_Lead_Screw_Electromagnet_OpenLoop(self.scene, self.i, self.path, self.pose_estimate, self.pid)
        #
        # # 机械臂操控胶囊
        # elif 'robot' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule":
        #     Create_Animate_Capsule_OpenLoop_double_new(self.scene, self.i, self.path, self.pose_estimate, self.pid1, self.pid2)
        #
        # # # 机械臂操控胶囊
        # # elif 'robot' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule":
        # #     # Create_Animate_Capsule_CloseLoop(self.scene, self.i, self.path, self.pose_estimate, self.pid, self.pid1)
        # #     Create_Animate_Capsule_OpenLoop(self.scene, self.i, self.path, self.pose_estimate, self.pid)
        #
        # #电磁铁操控胶囊
        # elif 'electromagnet' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule":
        #     # Create_Animate_Electromagnet_CloseLoop(self.scene, self.path,  self.pose_estimate, self.pid, self.pid1)
        #     Create_Animate_Electromagnet_OpenLoop(self.scene, self.i, self.path, self.pose_estimate, self.pid, self.pid1)
        #
        # # 机械臂操控导丝
        # elif 'robot' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "wire":
        #     Create_Animate_wire(self.scene, self.path, self.pose_estimate)
        #
        # # hm线圈操控胶囊
        # elif 'Helmholtz' in self.scene.magnetic_source[0] and 'Maxwell' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule":
        #     # Create_Animate_Helmholtz_Maxwell_testnewidea(self.scene, self.path, self.pose_estimate, self.pid, self.i)
        #     Create_Animate_Helmholtz_Maxwell3(self.scene, self.path, self.pose_estimate, self.pid, self.i, self.flag, self.data_t, self.data_p_real, self.data_p_desired, self.angle_list, self.angle_desired_list)
        #
        # # hm线圈操控软磁胶囊
        # elif 'Helmholtz' in self.scene.magnetic_source[0] and 'Maxwell' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "softrobot":
        #     Create_Animate_Soft_Robot_full(self.scene, self.path, self.i, self.pid, self.draw)
        #
        # # 电磁铁操控软磁胶囊
        # elif 'electromagnet' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "softrobot":
        #     # #软磁胶囊三个轴的长度不同
        #     # Create_Animate_eight_Soft_Robot_full(self.scene, self.path, self.i, self.pid, self.draw, self.force_desired_list, self.field_desired_list, self.force_real_list, self.field_real_list, self.current_list, self.time_list, self.flag)
        #
        #     # 软磁球
        #     Create_Animate_eight_SoftBall(self.scene, self.path, self.i, self.pid, self.draw, self.force_desired_list, self.field_desired_list, self.force_real_list, self.field_real_list, self.current_list, self.time_list, self.flag)


        # dt = 0.1
        # Kp1 = 0.01
        # Kp2 = 0.01
        # Ki = 0.00
        # kd = 0.00
        # ma1_norm = 19.5
        # ma2_norm = 19.5
        # mc1_norm = 0.1015
        # mc2_norm = 0.1015
        # JI = np.mat([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        #
        #
        # self.ePc1 = self.ePc1 + np.array([[-0.00 * dt], [0 * dt], [-0.001 * dt]])
        # print(self.ePc1)
        # self.ePc2 = self.ePc2 + np.array([[0.00 * dt], [0 * dt], [-0.001 * dt]])
        #
        # mc1_hat = self.pose_estimate.h_hat_pose[0]
        # mc2_hat = self.pose_estimate.h_hat_pose[1]
        # position = position_controller(self.ePc1, self.ePc2, self.emc1_hat, self.emc2_hat, self.force1, self.feild1, self.force2, self.feild2, 2.89e-3, 0.1)
        # x1_now, x2_now = self.pose_estimate.p_pose[0], self.pose_estimate.p_pose[1]
        # print('11111',x1_now)
        # print('11111',x2_now)
        # dpc1 = x1_now - self.pc1
        # dpc2 = x2_now - self.pc2
        # pc1, pc2 = x1_now, x2_now
        #
        # F1, F2, M1, M2 = position.pid_controller(pc1, pc2, mc1_hat, mc2_hat, Kp1, Kp2, Ki, kd)
        # df1 = F1 - self.F1_0
        # df2 = F2 - self.F2_0
        # dh1 = M1 - self.M1_0
        # dh2 = M2 - self.M2_0
        # self.F1_0, self.F2_0, self.M1_0, self.M2_0 = F1, F2, M1, M2
        # solution2 = double_ma_solve(mc1_norm, mc2_norm, ma1_norm, ma2_norm)
        # Jf = solution2.force_hat_field_jacob_for_2_a(pc1, pc2, self.pa1, self.ma_hat1, mc1_hat, mc2_hat, self.pa2, self.ma_hat2)
        #
        # JfI = Jf * JI
        # rank = np.linalg.matrix_rank(Jf)
        # iJf = np.linalg.pinv(JfI)
        # expect = np.mat(np.concatenate((df1, df2, dh1, dh2), axis=0))
        # dd = expect - Jf * np.mat(
        #     np.concatenate((np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]]), dpc1, dpc2)))
        #
        # solutions = iJf * dd
        # dpa1, dpa2, dm1, dm2 = split_solution(solutions)
        # # pa1_ = pa1_ + dpa1
        # # pa2_ = pa2_ + dpa2
        # # JA1 = Manipulator1.fkine_jacob(theta1)
        # # JA2 = Manipulator2.fkine_jacob(theta2)
        # JA1 = self.scene.magnetic_source[0]['robot'].fkine_jacob(self.scene.magnetic_source[0]['theta'])
        # JA2 = self.scene.magnetic_source[1]['robot'].fkine_jacob(self.scene.magnetic_source[1]['theta'])
        #
        # dq1 = np.linalg.pinv(JA1) * np.concatenate((dpa1, dm1), axis=0)
        # dq2 = np.linalg.pinv(JA2) * np.concatenate((dpa2, dm2), axis=0)
        # self.scene.magnetic_source[0]['theta'] += dq1
        # self.scene.magnetic_source[1]['theta'] += dq2
        # self.pa1, self.ma_hat1 = self.scene.magnetic_source[0]['robot'].fkine(self.scene.magnetic_source[0]['theta'])
        # self.pa2, self.ma_hat2 = self.scene.magnetic_source[1]['robot'].fkine(self.scene.magnetic_source[1]['theta'])
        #
        # pose_list = self.scene.magnetic_source[0]['robot'].fkine_all_link(self.scene.magnetic_source[0]['theta'])
        # for i in range(len(self.scene.magnetic_source[0]['link_pose_list'])):
        #     self.scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]
        # pose_list2 = self.scene.magnetic_source[1]['robot'].fkine_all_link(self.scene.magnetic_source[1]['theta'])
        # for i in range(len(self.scene.magnetic_source[1]['link_pose_list'])):
        #     self.scene.magnetic_source[1]['link_pose_list'][i].position[0][:] = pose_list2[i]
        #
        # self.force1, self.feild1, self.force2, self.feild2 = calculate_force_now(self.pa1, self.pa2, pc1, pc2, ma1_norm * self.ma_hat1, ma2_norm * self.ma_hat2,
        #                                                      mc1_hat * mc1_norm, mc2_hat * mc2_norm)
        # # print('ma_hat1', self.ma_hat1, 'ma_hat2', self.ma_hat2, 'mc_hat1', mc1_hat, 'mc_hat2', mc2_hat)
        # print("force1", self.force1, "feild1", self.feild1, "force2", self.force2, "feild2", self.feild2)
        # v_pos1 = np.array([self.scene.instrument[0]['velocity_active'][0:3]]).transpose()
        # v_pos2 = np.array([self.scene.instrument[1]['velocity_active'][0:3]]).transpose()
        # self.sofa_interface.set_force_moment(self.scene.instrument[0]['force_torque_capsule'], self.force1 - 0.02*v_pos1 + np.array([[0],[0],[-0.0005]]), np.array([[0], [0], [0]]))
        # self.sofa_interface.set_force_moment(self.scene.instrument[1]['force_torque_capsule'], self.force2 - 0.02*v_pos2 + np.array([[0],[0],[-0.0005]]), np.array([[0], [0], [0]]))



        # dt = 0.1
        # Kp1 = 0.01
        # Kp2 = 0.01
        # Ki = 0.00000
        # kd = 0.00000
        # ma1_norm = 19.5
        # ma2_norm = 19.5
        # mc1_norm = 0.1015
        # mc2_norm = 0.1015
        # JI = np.mat([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        #
        #
        # self.ePc1 = self.ePc1 + np.array([[-0.00 * dt], [0 * dt], [-0.001 * dt]])
        # # print(self.ePc1)
        # self.ePc2 = self.ePc2 + np.array([[0.00 * dt], [0 * dt], [-0.001 * dt]])
        #
        # mc1_hat = self.pose_estimate.h_hat_pose[0]
        # mc2_hat = self.pose_estimate.h_hat_pose[1]
        # # print(self.pa1, self.pa2, self.pc1, self.pc2,ma1_norm * self.ma_hat1,ma2_norm * self.ma_hat2,mc1_hat * mc1_norm, mc2_hat * mc2_norm)
        # self.force1, self.feild1, self.force2, self.feild2 = calculate_force_now(self.pa1, self.pa2, self.pc1, self.pc2,
        #                                                                          ma1_norm * self.ma_hat1,
        #                                                                          ma2_norm * self.ma_hat2,
        #                                                                          mc1_hat * mc1_norm, mc2_hat * mc2_norm)
        # print("force1", self.force1, "feild1", self.feild1, "force2", self.force2, "feild2", self.feild2)
        #
        # position = position_controller(self.ePc1, self.ePc2, self.emc1_hat, self.emc2_hat, self.force1, self.feild1, self.force2, self.feild2, 2.89e-3, 0.1)
        # v1_now, v2_now, x1_now, x2_now = position.physics(self.v1_last, self.v2_last, self.pc1, self.pc2)
        #
        # dpc1 = x1_now - self.pc1
        # dpc2 = x2_now - self.pc2
        # self.v1_last, self.v2_last, self.pc1, self.pc2 = v1_now, v2_now, x1_now, x2_now
        #
        # F1, F2, M1, M2 = position.pid_controller(self.pc1, self.pc2, mc1_hat, mc2_hat, Kp1, Kp2, Ki, kd)
        # df1 = F1 - self.F1_0
        # df2 = F2 - self.F2_0
        # dh1 = M1 - self.M1_0
        # dh2 = M2 - self.M2_0
        # self.F1_0, self.F2_0, self.M1_0, self.M2_0 = F1, F2, M1, M2
        # solution2 = double_ma_solve(mc1_norm, mc2_norm, ma1_norm, ma2_norm)
        # Jf = solution2.force_hat_field_jacob_for_2_a(self.pc1, self.pc2, self.pa1, self.ma_hat1, mc1_hat, mc2_hat, self.pa2, self.ma_hat2)
        #
        # JfI = Jf * JI
        # rank = np.linalg.matrix_rank(Jf)
        # iJf = np.linalg.pinv(JfI)
        # expect = np.mat(np.concatenate((df1, df2, dh1, dh2), axis=0))
        # dd = expect - Jf * np.mat(
        #     np.concatenate((np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]]), dpc1, dpc2)))
        #
        # solutions = iJf * dd
        # dpa1, dpa2, dm1, dm2 = split_solution(solutions)
        # # pa1_ = pa1_ + dpa1
        # # pa2_ = pa2_ + dpa2
        # # JA1 = Manipulator1.fkine_jacob(theta1)
        # # JA2 = Manipulator2.fkine_jacob(theta2)
        # JA1 = self.scene.magnetic_source[0]['robot'].fkine_jacob(self.scene.magnetic_source[0]['theta'])
        # JA2 = self.scene.magnetic_source[1]['robot'].fkine_jacob(self.scene.magnetic_source[1]['theta'])
        #
        # dq1 = np.linalg.pinv(JA1) * np.concatenate((dpa1, dm1), axis=0)
        # dq2 = np.linalg.pinv(JA2) * np.concatenate((dpa2, dm2), axis=0)
        # self.scene.magnetic_source[0]['theta'] += dq1
        # self.scene.magnetic_source[1]['theta'] += dq2
        # self.pa1, self.ma_hat1 = self.scene.magnetic_source[0]['robot'].fkine(self.scene.magnetic_source[0]['theta'])
        # self.pa2, self.ma_hat2 = self.scene.magnetic_source[1]['robot'].fkine(self.scene.magnetic_source[1]['theta'])
        #
        # pose_list = self.scene.magnetic_source[0]['robot'].fkine_all_link(self.scene.magnetic_source[0]['theta'])
        # for i in range(len(self.scene.magnetic_source[0]['link_pose_list'])):
        #     self.scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]
        # pose_list2 = self.scene.magnetic_source[1]['robot'].fkine_all_link(self.scene.magnetic_source[1]['theta'])
        # for i in range(len(self.scene.magnetic_source[1]['link_pose_list'])):
        #     self.scene.magnetic_source[1]['link_pose_list'][i].position[0][:] = pose_list2[i]



        self.t += 0.01

        dt = 0.04
        Kp1 = 0.1
        Kp2 = 0.1
        Ki = 0.1
        kd = 0.00

        JI = np.mat([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])



        # print('11111',self.x1_now)
        # print('11111',self.x2_now)
        self.x1_now = self.pose_estimate.p_pose[0]
        self.x2_now = self.pose_estimate.p_pose[1]
        self.mc1_hat = self.pose_estimate.h_hat_pose[0]
        self.mc2_hat = self.pose_estimate.h_hat_pose[1]
        dpc1 = self.x1_now - self.pc1
        dpc2 = self.x2_now - self.pc2
        self.pc1, self.pc2 = self.x1_now, self.x2_now

        F1, F2, M1, M2 = self.position.pid_controller(self.pc1, self.pc2, self.mc1_hat, self.mc2_hat, Kp1, Kp2, Ki, kd, 0.7)
        df1 = F1 - self.F1_0
        df2 = F2 - self.F2_0
        dh1 = M1
        dh2 = M2
        self.F1_0, self.F2_0, self.M1_0, self.M2_0 = F1, F2, M1, M2
        solution2 = double_ma_solve(self.mc1_norm, self.mc2_norm, self.ma1_norm, self.ma2_norm)
        Jf = solution2.force_hat_field_jacob_for_2_a(self.pc1, self.pc2, self.pa1, self.ma_hat1, self.mc1_hat, self.mc2_hat, self.pa2, self.ma_hat2)

        JfI = Jf * JI
        rank = np.linalg.matrix_rank(Jf)
        iJf = np.linalg.pinv(JfI)
        expect = np.mat(np.concatenate((df1, df2, dh1, dh2), axis=0))
        dd = expect - Jf * np.mat(
            np.concatenate((np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]]), dpc1, dpc2)))

        solutions = iJf * dd
        dpa1, dpa2, dm1, dm2 = split_solution(solutions)
        # pa1_ = pa1_ + dpa1
        # pa2_ = pa2_ + dpa2
        # JA1 = Manipulator1.fkine_jacob(theta1)
        # JA2 = Manipulator2.fkine_jacob(theta2)
        JA1 = self.scene.magnetic_source[0]['robot'].fkine_jacob(self.scene.magnetic_source[0]['theta'])
        JA2 = self.scene.magnetic_source[1]['robot'].fkine_jacob(self.scene.magnetic_source[1]['theta'])

        dq1 = np.linalg.pinv(JA1) * np.concatenate((dpa1, dm1), axis=0)
        dq2 = np.linalg.pinv(JA2) * np.concatenate((dpa2, dm2), axis=0)
        self.scene.magnetic_source[0]['theta'] += dq1
        self.scene.magnetic_source[1]['theta'] += dq2
        self.pa1, self.ma_hat1 = self.scene.magnetic_source[0]['robot'].fkine(self.scene.magnetic_source[0]['theta'])
        self.pa2, self.ma_hat2 = self.scene.magnetic_source[1]['robot'].fkine(self.scene.magnetic_source[1]['theta'])

        pose_list = self.scene.magnetic_source[0]['robot'].fkine_all_link(self.scene.magnetic_source[0]['theta'])
        for i in range(len(self.scene.magnetic_source[0]['link_pose_list'])):
            self.scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]
        pose_list2 = self.scene.magnetic_source[1]['robot'].fkine_all_link(self.scene.magnetic_source[1]['theta'])
        for i in range(len(self.scene.magnetic_source[1]['link_pose_list'])):
            self.scene.magnetic_source[1]['link_pose_list'][i].position[0][:] = pose_list2[i]



        # self.ePc1 = self.ePc1 + np.array([[-0.0003 * dt], [0 * dt], [-0.0003 * dt]])
        # # print(self.ePc1)
        # self.ePc2 = self.ePc2 + np.array([[0.0003 * dt], [0 * dt], [-0.0003 * dt]])
        # self.emc1_hat = (self.emc1_hat + np.array([[0.003 * dt], [0 * dt], [-0.00 * dt]]))/np.linalg.norm(self.emc1_hat + np.array([[0.003 * dt], [0 * dt], [-0.00 * dt]]))
        # print(self.emc1_hat)
        # self.emc2_hat = (self.emc2_hat + np.array([[0.00 * dt], [0 * dt], [-0.00 * dt]]))/np.linalg.norm(self.emc2_hat + np.array([[0.00 * dt], [0 * dt], [-0.00 * dt]]))

        # if  0 < self.t < 1:
        #     self.ePc1 = self.ePc1 + np.array([[-0.0003 * dt], [0 * dt], [-0.000 * dt]])
        #     self.ePc2 = self.ePc2 + np.array([[-0.0003 * dt], [0 * dt], [-0.000 * dt]])
        #
        # if  1 < self.t < 11:
        #     self.ePc1 = self.ePc1 + np.array([[-0.0005 * dt], [0 * dt], [-0.000 * dt]])
        #     self.ePc2 = self.ePc2 + np.array([[-0.0005 * dt], [0 * dt], [-0.000 * dt]])
        #     # print(self.ePc2)
        # if 11 < self.t < 21:
        #     self.ePc1 = self.ePc1 + np.array([[0.000 * dt], [0 * dt], [-0.0005 * dt]])
        #     self.ePc2 = self.ePc2 + np.array([[0.000 * dt], [0 * dt], [-0.0005 * dt]])
        #     # if 9 < self.t < 13:
        #     #     self.emc1_hat = (self.emc1_hat + np.array([[0.003 * dt], [0 * dt], [-0.00 * dt]]))/np.linalg.norm(self.emc1_hat + np.array([[0.003 * dt], [0 * dt], [-0.00 * dt]]))
        # if 21 < self.t < 31:
        #     self.ePc1 = self.ePc1 + np.array([[0.0005 * dt], [0 * dt], [-0.000 * dt]])
        #     self.ePc2 = self.ePc2 + np.array([[0.0005 * dt], [0 * dt], [-0.000 * dt]])
        # if 31 < self.t < 41:
        #     self.ePc1 = self.ePc1 + np.array([[-0.000 * dt], [0 * dt], [0.0005 * dt]])
        #     self.ePc2 = self.ePc2 + np.array([[-0.000 * dt], [0 * dt], [0.0005 * dt]])

        if  0 < self.t < 1:
            self.ePc1 = self.ePc1 + np.array([[-0.0003 * dt], [0 * dt], [-0.000 * dt]])
            self.ePc2 = self.ePc2 + np.array([[-0.0003 * dt], [0 * dt], [-0.000 * dt]])

        if  1 < self.t < 11:
            self.ePc1 = self.ePc1 + np.array([[-0.0005 * dt], [0 * dt], [-0.000 * dt]])
            self.ePc2 = self.ePc2 + np.array([[-0.0005 * dt], [0 * dt], [-0.000 * dt]])
            # print(self.ePc2)
        if 11 < self.t < 21:
            # self.ePc1 = self.ePc1 + np.array([[0.000 * dt], [-0.0005 * dt], [0 * dt]])
            # self.ePc2 = self.ePc2 + np.array([[0.000 * dt], [-0.0005 * dt], [0 * dt]])
            self.ePc1 = self.ePc1 + np.array([[0.000 * dt], [0 * dt], [-0.0005 * dt]])
            self.ePc2 = self.ePc2 + np.array([[0.000 * dt], [0 * dt], [-0.0005 * dt]])
            # if 9 < self.t < 13:
            #     self.emc1_hat = (self.emc1_hat + np.array([[0.003 * dt], [0 * dt], [-0.00 * dt]]))/np.linalg.norm(self.emc1_hat + np.array([[0.003 * dt], [0 * dt], [-0.00 * dt]]))
        if 21 < self.t < 22:
            self.ePc1 = self.ePc1 + np.array([[0.000 * dt], [0 * dt], [-0.000 * dt]])
            self.ePc2 = self.ePc2 + np.array([[0.000 * dt], [0 * dt], [-0.000 * dt]])
        if 22 < self.t < 31:
            self.ePc1 = self.ePc1 + np.array([[0.0005 * dt], [0 * dt], [-0.000 * dt]])
            self.ePc2 = self.ePc2 + np.array([[0.0005 * dt], [0 * dt], [-0.000 * dt]])
        if 31 < self.t < 41:
            # self.ePc1 = self.ePc1 + np.array([[0.000 * dt], [0.0005 * dt], [0 * dt]])
            # self.ePc2 = self.ePc2 + np.array([[0.000 * dt], [0.0005 * dt], [0 * dt]])
            self.ePc1 = self.ePc1 + np.array([[0.000 * dt], [0 * dt], [0.0005 * dt]])
            self.ePc2 = self.ePc2 + np.array([[0.000 * dt], [0 * dt], [0.0005 * dt]])

        if self.scene.root_node.time.value > 1:
            position1, moment1 = pose_to_position_moment(self.scene.instrument[0]["position_active"])
            self.position_x1.append(position1[0, 0])
            self.position_y1.append(position1[1, 0])
            self.position_z1.append(position1[2, 0])
            # self.position_m1.append(moment1[0, 0])
            # self.position_n1.append(moment1[1, 0])
            # self.position_p1.append(moment1[2, 0])
            p_desired1, h_hat_desired1 = self.ePc1, self.emc1_hat
            self.plan_x1.append(p_desired1[0, 0])
            self.plan_y1.append(p_desired1[1, 0])
            self.plan_z1.append(p_desired1[2, 0])
            # self.plan_m1.append(h_hat_desired1[0, 0])
            # self.plan_n1.append(h_hat_desired1[1, 0])
            # self.plan_p1.append(h_hat_desired1[2, 0])

            position2, moment2 = pose_to_position_moment(self.scene.instrument[1]["position_active"])
            self.position_x2.append(position2[0, 0])
            self.position_y2.append(position2[1, 0])
            self.position_z2.append(position2[2, 0])
            # self.position_m2.append(moment2[0, 0])
            # self.position_n2.append(moment2[1, 0])
            # self.position_p2.append(moment2[2, 0])
            p_desired2, h_hat_desired2 = self.ePc2, self.emc2_hat
            self.plan_x2.append(p_desired2[0, 0])
            self.plan_y2.append(p_desired2[1, 0])
            self.plan_z2.append(p_desired2[2, 0])
            # self.plan_m2.append(h_hat_desired2[0, 0])
            # self.plan_n2.append(h_hat_desired2[1, 0])
            # self.plan_p2.append(h_hat_desired2[2, 0])

        if self.scene.root_node.time.value > 1:
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_x.txt", 'w') as f:
                f.write(str(self.position_x1))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_y.txt", 'w') as f:
                f.write(str(self.position_y1))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_z.txt", 'w') as f:
                f.write(str(self.position_z1))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_m.txt", 'w') as f:
            #     f.write(str(self.position_m1))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_n.txt", 'w') as f:
            #     f.write(str(self.position_n1))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_p.txt", 'w') as f:
            #     f.write(str(self.position_p1))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_x.txt", 'w') as f:
                f.write(str(self.plan_x1))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_y.txt", 'w') as f:
                f.write(str(self.plan_y1))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_z.txt", 'w') as f:
                f.write(str(self.plan_z1))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_m.txt", 'w') as f:
            #     f.write(str(self.plan_m1))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_n.txt", 'w') as f:
            #     f.write(str(self.plan_n1))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_p.txt", 'w') as f:
            #     f.write(str(self.plan_p1))

            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_x2.txt", 'w') as f:
                f.write(str(self.position_x2))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_y2.txt", 'w') as f:
                f.write(str(self.position_y2))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_z2.txt", 'w') as f:
                f.write(str(self.position_z2))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_m2.txt", 'w') as f:
            #     f.write(str(self.position_m2))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_n2.txt", 'w') as f:
            #     f.write(str(self.position_n2))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\cunchu_p2.txt", 'w') as f:
            #     f.write(str(self.position_p2))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_x2.txt", 'w') as f:
                f.write(str(self.plan_x2))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_y2.txt", 'w') as f:
                f.write(str(self.plan_y2))
            with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_z2.txt", 'w') as f:
                f.write(str(self.plan_z2))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_m2.txt", 'w') as f:
            #     f.write(str(self.plan_m2))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_n2.txt", 'w') as f:
            #     f.write(str(self.plan_n2))
            # with open("C:\\Users\\10145\\Desktop\\double\\11.28\\post_processing\\plan_p2.txt", 'w') as f:
            #     f.write(str(self.plan_p2))

        mc1_hat = self.pose_estimate.h_hat_pose[0]
        mc2_hat = self.pose_estimate.h_hat_pose[1]


        self.force1, self.feild1, moment1, self.force2, self.feild2, moment2 = calculate_force_now(self.pa1, self.pa2, self.pc1, self.pc2, self.ma1_norm * self.ma_hat1, self.ma2_norm * self.ma_hat2,
                                                             self.mc1_hat * self.mc1_norm, self.mc2_hat * self.mc2_norm)
        self.position = position_controller(self.ePc1, self.ePc2, self.emc1_hat, self.emc2_hat, self.force1, self.feild1, self.force2, self.feild2, 2.89e-3, 0.1)
        # print('ma_hat1', self.ma_hat1, 'ma_hat2', self.ma_hat2, 'mc_hat1', mc1_hat, 'mc_hat2', mc2_hat)
        print("force1", self.force1, "feild1", self.feild1, "force2", self.force2, "feild2", self.feild2, 'moment', moment1)

        v_pos1 = np.array([self.scene.instrument[0]['velocity_active'][0:3]]).transpose()
        v_pos2 = np.array([self.scene.instrument[1]['velocity_active'][0:3]]).transpose()
        w_pos1 = np.array([self.scene.instrument[0]['velocity_active'][3:6]]).transpose()
        w_pos2 = np.array([self.scene.instrument[1]['velocity_active'][3:6]]).transpose()
        self.sofa_interface.set_force_moment(self.scene.instrument[0]['force_torque_capsule'], self.force1 - 0.02*v_pos1 + np.array([[0],[0],[-0.0005]]), moment1 - 0.00002*w_pos1)
        self.sofa_interface.set_force_moment(self.scene.instrument[1]['force_torque_capsule'], self.force2 - 0.02*v_pos2 + np.array([[0],[0],[-0.0005]]), moment2 - 0.00002*w_pos2)





        # dt = 0.1
        # Kp1 = 0.01
        # Ki = 0.00
        # kd = 0.00
        #
        # JI = np.mat([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        #
        #
        #
        # # print('11111',self.x1_now)
        # # print(calculate_force_now_single(self.pa1, self.pa2, self.pc1, self.ma_hat1 * self.ma1_norm, self.ma_hat2 * self.ma2_norm, self.mc1_hat * self.mc1_norm))
        # self.x1_now = self.pose_estimate.p_pose[0]
        # self.mc1_hat = self.pose_estimate.h_hat_pose[0]
        # dpc1 = self.x1_now - self.pc1
        # self.pc1 = self.x1_now
        #
        # F1, M1 = self.position.pid_controller(self.pc1, self.mc1_hat, Kp1, Ki, kd, 0.7)
        # df1 = F1 - self.F1_0
        # dh1 = M1
        #
        # self.F1_0, self.M1_0 = F1, M1
        # solution2 = double_ma_solve_single(self.mc1_norm, self.ma1_norm, self.ma2_norm)
        # Jf = solution2.force_hat_field_jacob_for_2_a(self.pc1, self.pa1, self.ma_hat1, self.mc1_hat, self.pa2, self.ma_hat2)
        #
        # JfI = Jf * JI
        # rank = np.linalg.matrix_rank(Jf)
        # iJf = np.linalg.pinv(JfI)
        # expect = np.mat(np.concatenate((df1, dh1), axis=0))
        # dd = expect - Jf * np.mat(
        #     np.concatenate((np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]]), dpc1)))
        #
        # solutions = iJf * dd
        # dpa1, dpa2, dm1, dm2 = split_solution(solutions)
        #
        # JA1 = self.scene.magnetic_source[0]['robot'].fkine_jacob(self.scene.magnetic_source[0]['theta'])
        # JA2 = self.scene.magnetic_source[1]['robot'].fkine_jacob(self.scene.magnetic_source[1]['theta'])
        #
        #
        # dq1 = np.linalg.pinv(JA1) * np.concatenate((dpa1, dm1), axis=0)
        # dq2 = np.linalg.pinv(JA2) * np.concatenate((dpa2, dm2), axis=0)
        # self.scene.magnetic_source[0]['theta'] += dq1
        # self.scene.magnetic_source[1]['theta'] += dq2
        # self.pa1, self.ma_hat1 = self.scene.magnetic_source[0]['robot'].fkine(self.scene.magnetic_source[0]['theta'])
        # self.pa2, self.ma_hat2 = self.scene.magnetic_source[1]['robot'].fkine(self.scene.magnetic_source[1]['theta'])
        #
        # pose_list = self.scene.magnetic_source[0]['robot'].fkine_all_link(self.scene.magnetic_source[0]['theta'])
        # for i in range(len(self.scene.magnetic_source[0]['link_pose_list'])):
        #     self.scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]
        # pose_list2 = self.scene.magnetic_source[1]['robot'].fkine_all_link(self.scene.magnetic_source[1]['theta'])
        # for i in range(len(self.scene.magnetic_source[1]['link_pose_list'])):
        #     self.scene.magnetic_source[1]['link_pose_list'][i].position[0][:] = pose_list2[i]
        #
        #
        #
        # self.ePc1 = self.ePc1 + np.array([[-0.0001 * dt], [0 * dt], [-0.0001 * dt]])
        # self.emc1_hat = (self.emc1_hat + np.array([[0.01 * dt], [0 * dt], [-0.00 * dt]]))/np.linalg.norm(self.emc1_hat + np.array([[0.01 * dt], [0 * dt], [-0.00 * dt]]))
        # # print(self.ePc1)
        #
        # # mc1_hat = self.pose_estimate.h_hat_pose[0]
        # # mc2_hat = self.pose_estimate.h_hat_pose[1]
        #
        #
        # self.force1, self.feild1, moment = calculate_force_now_single(self.pa1, self.pa2, self.pc1, self.ma1_norm * self.ma_hat1, self.ma2_norm * self.ma_hat2,
        #                                                      self.mc1_hat * self.mc1_norm)
        # print(moment)
        # # print(self.moment)
        # # print(self.feild1)
        # self.position = position_controller_single(self.ePc1, self.emc1_hat, self.force1, self.feild1, 2.89e-3, 0.1)
        # # print('ma_hat1', self.ma_hat1, 'ma_hat2', self.ma_hat2, 'mc_hat1', mc1_hat, 'mc_hat2', mc2_hat)
        # # print("force1", self.force1, "feild1", self.feild1, "force2", self.force2, "feild2", self.feild2)
        #
        # v_pos1 = np.array([self.scene.instrument[0]['velocity_active'][0:3]]).transpose()
        # w_pos1 = np.array([self.scene.instrument[0]['velocity_active'][3:6]]).transpose()
        # self.sofa_interface.set_force_moment(self.scene.instrument[0]['force_torque_capsule'], self.force1 - 0.02*v_pos1 + np.array([[0],[0],[-0.0005]]), moment - 0.00002*w_pos1)


