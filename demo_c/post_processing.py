import Sofa
from Pose_Transform.pose_transform import *
import numpy as np
import os

class Post_processing(Sofa.Core.Controller):
    def __init__(self, scene, path, pose_estimate, mag_controller, environment, *args, **kwargs):

        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.scene = scene
        self.path = path
        self.pose_estimate = pose_estimate
        self.mag_controller = mag_controller
        self.env = environment
        # print("---------------------------------------------------")
    #真实的位姿
        self.position_x = [0]
        self.position_y = [0]
        self.position_z = [0]

        # self.position_m = [0]
        # self.position_n = [0]
        # self.position_p = [0]
        self.theta = []

    ##定位的位姿
        self.estimate_x = []
        self.estimate_y = []
        self.estimate_z = []

        self.angle_between_p_e = []

        # self.estimate_m = []
        # self.estimate_n = []
        # self.estimate_p = []
        # self.estimate_polar_angle = []
        # self.estimate_azimuthal_angle = []

    ##驱动的理想位姿
        self.plan_x = []
        self.plan_y = []
        self.plan_z = []

        self.angle_between_p_p = []

        # self.plan_m = []
        # self.plan_n = []
        # self.plan_p = []

    ##约束力
        self.magnetic_force = []
        self.constraintforce_collision = []
        self.constraintforce_friction = []

    ##电流
        self.list_of_currents = [[] for _ in range(self.scene.current_number)]
        # print('------------------')
        # print(self.list_of_currents)
        # print(self.scene.current_number)
        self.i = 0

    def onAnimateBeginEvent(self, event):
        self.i += 1
        position, moment = pose_to_position_moment(self.scene.instrument[0]["position_active"])
        self.position_x.append(round(position[0, 0], 6))
        self.position_y.append(round(position[1, 0], 6))
        self.position_z.append(round(position[2, 0], 6))
        # self.position_m.append(round(moment[0, 0], 6))
        # self.position_n.append(round(moment[1, 0], 6))
        # self.position_p.append(round(moment[2, 0], 6))
        self.theta.append(round(moment_to_angle(moment)[1], 6))

        p_estimate, h_hat_estimate = self.pose_estimate.p_pose, self.pose_estimate.h_hat_pose
        self.estimate_x.append(round(p_estimate[0, 0], 6))
        self.estimate_y.append(round(p_estimate[1, 0], 6))
        self.estimate_z.append(round(p_estimate[2, 0], 6))

        #虽然都是单位向量，但存在微小误差导致可能超出cos值域，所以需要除以模场长
        if np.dot(moment.T, h_hat_estimate)/(np.linalg.norm(moment) * np.linalg.norm(h_hat_estimate)) > 1:
            self.angle_between_p_e.append(0.0)
        else:
            self.angle_between_p_e.append(round(np.degrees(np.arccos(np.dot(moment.T, h_hat_estimate)/(np.linalg.norm(moment) * np.linalg.norm(h_hat_estimate))))[0, 0], 6))
        # print(round(np.degrees(np.arccos(np.dot(moment.T, h_hat_estimate)/(np.linalg.norm(moment) * np.linalg.norm(h_hat_estimate))))[0, 0], 6))
        # self.estimate_m.append(round(h_hat_estimate[0, 0], 6))
        # self.estimate_n.append(round(h_hat_estimate[1, 0], 6))
        # self.estimate_p.append(round(h_hat_estimate[2, 0], 6))

        # polar_angle_estimate, azimuthal_angle_estimate = vector_to_angles(np.array([h_hat_estimate[0], h_hat_estimate[1], h_hat_estimate[2]]))
        # print(azimuthal_angle_estimate)
        # self.estimate_polar_angle.append(round(polar_angle_estimate, 6))
        # self.estimate_azimuthal_angle.append(round(azimuthal_angle_estimate, 6))

        p_desired, h_hat_desired = self.path.p_desired, self.path.h_hat_desired
        self.plan_x.append(round(p_desired[0, 0], 6))
        self.plan_y.append(round(p_desired[1, 0], 6))
        self.plan_z.append(round(p_desired[2, 0], 6))

        #虽然都是单位向量，但存在微小误差导致可能超出cos值域，所以需要除以模场长
        if np.dot(moment.T, h_hat_desired)/(np.linalg.norm(moment) * np.linalg.norm(h_hat_desired)) > 1:
            self.angle_between_p_p.append(0.0)
        else:
            self.angle_between_p_p.append(round(np.degrees(np.arccos(np.dot(moment.T, h_hat_desired)/(np.linalg.norm(moment) * np.linalg.norm(h_hat_desired))))[0, 0], 6))
        # print(round(np.degrees(np.arccos(np.dot(moment.T, h_hat_desired)/(np.linalg.norm(moment) * np.linalg.norm(h_hat_desired))))[0, 0], 6))
        # self.plan_m.append(round(h_hat_desired[0, 0], 6))
        # self.plan_n.append(round(h_hat_desired[1, 0], 6))
        # self.plan_p.append(round(h_hat_desired[2, 0], 6))

        if np.size(self.env.constraints.constraintForces.value) == 0:
            self.constraintforce_collision.append(0.0)
            self.constraintforce_friction.append(0.0)
            if 'wire_Mechanical' in self.scene.instrument[0] or 'capsule_Mechanical' in self.scene.instrument[0]:
                self.scene.instrument[0]['CFF_visu'].forces[0][0:3] = [0.0, 0.0, 0.0]
                self.scene.instrument[0]['CFF_visu_2'].forces[0][0:3] = [0.0, 0.0, 0.0]
        else:
            cf_normal_sum = [0, 0, 0]
            cf_tangent_sum = [0, 0, 0]
            if 'wire_Mechanical' in self.scene.instrument[0]:
                lines = self.scene.instrument[0]['wire_Mechanical'].constraint.value.splitlines()
                point = self.scene.instrument[0]['num']
            elif 'capsule_Mechanical' in self.scene.instrument[0]:
                lines = self.scene.instrument[0]['capsule_Mechanical'].constraint.value.splitlines()
                point = 0.0

            for i in range(len(lines)):
                array = np.fromstring(lines[i], sep=' ')
                # print(array)
                mask = (array == point)  # 创建布尔掩码，标记与目标数相等的元素
                indices = np.where(mask)[0]  # 获取满足条件的元素的索引
                # print(indices)

                if len(indices) > 0:
                    if len(indices) > 1:
                        index = indices[1]
                    else:
                        index = indices[0]
                    trans = array[index + 1:index + 4]
                    if i % 3 == 0:
                        cf_normal_sum += self.env.constraints.constraintForces.value[i] * trans / self.scene.root_node.dt.value
                        # print(self.env.constraints.constraintForces.value[i] * trans / self.scene.root_node.dt.value)
                    else:
                        cf_tangent_sum += self.env.constraints.constraintForces.value[i] * trans / self.scene.root_node.dt.value
                else:
                    # print(i)
                    break
            if cf_normal_sum[0] != 0 or cf_tangent_sum[0] != 0:
                # 计算共线方向的投影向量
                # print(1)
                cf_normal_d = np.dot(cf_tangent_sum, cf_normal_sum) / np.dot(cf_normal_sum, cf_normal_sum) * cf_normal_sum

                # 计算垂直方向的投影向量
                cf_tangent_d = cf_tangent_sum - cf_normal_d
                # print(cf_normal_d)
                # print(cf_tangent_d)

                cf_normal_sum = cf_normal_sum + cf_normal_d
                cf_tangent_sum = cf_tangent_d

                # print(cf_normal_sum)
                # print(cf_tangent_sum)

                self.scene.instrument[0]['CFF_visu'].forces[0][0:3] = cf_normal_sum / 3000
                self.scene.instrument[0]['CFF_visu_2'].forces[0][0:3] = cf_tangent_sum / 3000
                self.constraintforce_collision.append(np.linalg.norm(cf_normal_sum))
                self.constraintforce_friction.append(np.linalg.norm(cf_tangent_sum))
                # print(cf_normal_sum)
            else:
                self.scene.instrument[0]['CFF_visu'].forces[0][0:3] = [0.0, 0.0, 0.0]
                self.scene.instrument[0]['CFF_visu_2'].forces[0][0:3] = [0.0, 0.0, 0.0]
                self.constraintforce_collision.append(0.0)
                self.constraintforce_friction.append(0.0)
                # print(111111111)

        if self.scene.root_node.time.value < 113:
            with open("..\\..\\demo_c\\post_processing\\cunchu_x.txt", 'w') as f:
                # print("--------------------------------")
                f.write(str(self.position_x[1:]))
            # with open("..\\..\\demo_c\\post_processing\\cunchu_x.txt", "r+b") as file:
            #     # 移动文件指针到"]"符号的前一个位置
            #     file.seek(-1, os.SEEK_END)
            with open("..\\..\\demo_c\\post_processing\\cunchu_y.txt", 'w') as f:
                f.write(str(self.position_y[1:]))
            with open("..\\..\\demo_c\\post_processing\\cunchu_z.txt", 'w') as f:
                f.write(str(self.position_z[1:]))
            # with open("..\\..\\demo_c\\post_processing\\cunchu_m.txt", 'w') as f:
            #     f.write(str(self.position_m[1:]))
            # with open("..\\..\\demo_c\\post_processing\\cunchu_n.txt", 'w') as f:
            #     f.write(str(self.position_n[1:]))
            # with open("..\\..\\demo_c\\post_processing\\cunchu_p.txt", 'w') as f:
            #     f.write(str(self.position_p[1:]))
            with open("..\\..\\demo_c\\post_processing\\cunchu_theta.txt", 'w') as f:
                f.write(str(self.theta))


            with open("..\\..\\demo_c\\post_processing\\estimate_x.txt", 'w') as f:
                f.write(str(self.estimate_x))
            with open("..\\..\\demo_c\\post_processing\\estimate_y.txt", 'w') as f:
                f.write(str(self.estimate_y))
            with open("..\\..\\demo_c\\post_processing\\estimate_z.txt", 'w') as f:
                f.write(str(self.estimate_z))
            with open("..\\..\\demo_c\\post_processing\\estimate_angle_error.txt", 'w') as f:
                f.write(str(self.angle_between_p_e))
            # with open("..\\..\\demo_c\\post_processing\\estimate_m.txt", 'w') as f:
            #     f.write(str(self.estimate_m))
            # with open("..\\..\\demo_c\\post_processing\\estimate_n.txt", 'w') as f:
            #     f.write(str(self.estimate_n))
            # with open("..\\..\\demo_c\\post_processing\\estimate_p.txt", 'w') as f:
            #     f.write(str(self.estimate_p))
            # with open("..\\..\\demo_c\\post_processing\\estimate_polar_angle.txt", 'w') as f:
            #     f.write(str(self.estimate_polar_angle))
            # with open("..\\..\\demo_c\\post_processing\\estimate_azimuthal_angle.txt", 'w') as f:
            #     f.write(str(self.estimate_azimuthal_angle))

            with open("..\\..\\demo_c\\post_processing\\plan_x.txt", 'w') as f:
                f.write(str(self.plan_x))
            with open("..\\..\\demo_c\\post_processing\\plan_y.txt", 'w') as f:
                f.write(str(self.plan_y))
            with open("..\\..\\demo_c\\post_processing\\plan_z.txt", 'w') as f:
                f.write(str(self.plan_z))
            with open("..\\..\\demo_c\\post_processing\\plan_angle_error.txt", 'w') as f:
                f.write(str(self.angle_between_p_p))
            # with open("..\\..\\demo_c\\post_processing\\plan_m.txt", 'w') as f:
            #     f.write(str(self.plan_m))
            # with open("..\\..\\demo_c\\post_processing\\plan_n.txt", 'w') as f:
            #     f.write(str(self.plan_n))
            # with open("..\\..\\demo_c\\post_processing\\plan_p.txt", 'w') as f:
            #     f.write(str(self.plan_p))

            if 'wire_Mechanical' in self.scene.instrument[0]:
                self.magnetic_force.append(np.linalg.norm(self.scene.instrument[0]['force_torque_wire'].forces[0][0:3]))

            with open("..\\..\\demo_c\\post_processing\\magnetic_force.txt", 'w') as f:
                f.write(str(self.magnetic_force))

            with open("..\\..\\demo_c\\post_processing\\constraintforce_collision.txt", 'w') as f:
                f.write(str(self.constraintforce_collision))
            with open("..\\..\\demo_c\\post_processing\\constraintforce_collision_realtime.txt", 'w') as f:
                f.write(str([self.scene.root_node.time.value, (self.constraintforce_collision[len(self.constraintforce_collision) - 1]), (self.constraintforce_friction[len(self.constraintforce_friction) - 1])]))
            with open("..\\..\\demo_c\\post_processing\\pose_realtime.txt", 'w') as f:
                r = R.from_quat([self.scene.instrument[0]["position_active"][3], self.scene.instrument[0]["position_active"][4], self.scene.instrument[0]["position_active"][5], self.scene.instrument[0]["position_active"][6]])
                euler_angles = r.as_euler('xyz', degrees=True)  # 返回角度制的欧拉角
                f.write(str([round(self.position_x[-1], 6), round(self.position_y[-1], 6), round(self.position_z[-1], 6), round(euler_angles[0], 6), round(euler_angles[1], 6), round(euler_angles[2], 6)]))
            with open("..\\..\\demo_c\\post_processing\\constraintforce_friction.txt", 'w') as f:
                f.write(str(self.constraintforce_friction))

            if len(self.scene.magnetic_source) != 0:

                for i in range(len(self.scene.current)):
                    # print(self.scene.current)
                    # print(self.list_of_currents[i])
                    # print(i)
                    self.list_of_currents[i].append(self.scene.current[i, 0])
                if 'electromagnet' in self.scene.magnetic_source[0]:
                    with open("..\\..\\demo_c\\post_processing\\elc_current_realtime.txt", 'w') as f:
                        f.write(str([self.scene.root_node.time.value] + self.scene.current.flatten().tolist()[0]))
                    with open("..\\..\\demo_c\\post_processing\\elc_current.txt", 'w') as f:
                        f.write(str(self.list_of_currents))
                # if 'Helmholtz' in self.scene.magnetic_source[0]:
                #     with open("..\\..\\demo_c\\post_processing\\elc_current_HM_realtime.txt", 'w') as f:
                #         f.write(str([self.scene.root_node.time.value] + self.scene.current.flatten().tolist()))
                #     with open("..\\..\\demo_c\\post_processing\\elc_current_HM.txt", 'w') as f:
                #         f.write(str(self.list_of_currents))
