import numpy as np

from Module_Package.scene_module import *
from Environment_Package.environment import *
from Magnetic_Engine.permanent_magnet import *
import math


class Scene:
    def __init__(self, root_node, info):
        self.dt = info[5][0]["step"]
        self.root_node = root_node

        self.instrument = []
        self.magnetic_source = []
        self.sensor = []

        self.show_pose = None
        self.pose = None
        self.current = None
        self.current_number = 0
        # 器械
        for i in range(len(info[2])):
            if info[2][i]["object"] == "capsule":
                # 下面是磁性胶囊的参数
                # v = 5.236e-7, a = 1.53e-8
                if not info[1]:
                    a = 6.12e-9
                elif info[1][i]["drive_yuan"] == "yidong_rob_mag":
                    a = 6.12e-5
                else:
                    # a = 6.12e-9
                    a = 3e-4
                # a = 6.12e-5
                # self.instrument.append({'pose_active': None, 'force_torque_capsule': None, 'pose': info[1][i]["pose"], 'moment': info[1][i]["moment"], 'totalMass': info[1][i]["capsule_mass"], 'volume': 5.236e-7, 'inertiaMatrix': [1.53e-8, 0., 0., 0., 1.53e-8, 0., 0., 0., 1.53e-8]})
                self.instrument.append({'position_active': None, 'velocity_active': None, 'force_torque_capsule': None, 'capsule_Mechanical': None, 'CFF_visu': None, 'CFF_visu_2': None,
                                        'pose': info[2][i]["pose"],
                                        "object": "capsule",
                                        'moment': info[2][i]["moment"], 'totalMass': info[2][i]["capsule_mass"],
                                        'flotage': info[2][i]["flotage"],
                                        'volume': 4.18879e-6,
                                        # 'inertiaMatrix': [a, 0., 0., 0., a, 0., 0., 0., a],
                                        'inertiaMatrix': info[2][i]["inertiaMatrix"]})
                self.instrument[i]['position_active'], self.instrument[i]['capsule_Mechanical'], self.instrument[i]['velocity_active'], self.instrument[i][
                    'force_torque_capsule'], self.instrument[i]['CFF_visu'], self.instrument[i]['CFF_visu_2'] = Create_Magnetic_sphere(
                    self.root_node, self.instrument[i]['pose'], self.instrument[i]['totalMass'],
                    self.instrument[i]['volume'], self.instrument[i]['inertiaMatrix'], info)
            if info[2][i]["object"] == "wire":

                self.instrument.append(
                    {'position_active': None, 'force_torque_wire': None, 'wire_Mechanical': None, 'CFF_visu': None, 'CFF_visu_2': None,
                     'moment': info[2][i]["moment"], "object": "wire",
                     'pose': info[2][i]["pose"], 'y_modulus_wire': info[2][i]["y_modulus_wire"], 'inner_diameter_wire': info[2][i]["inner_diameter_wire"], 'outer_diameter_wire': info[2][i]["outer_diameter_wire"], 'IRC': 0,
                     'num': math.ceil(info[2][i]["length"] / info[2][i]["length_density"][0]), 'length_density': info[2][i]["length_density"]})
                self.instrument[i]['position_active'], self.instrument[i][
                    'force_torque_wire'], self.instrument[i]['wire_Mechanical'], self.instrument[i]['CFF_visu'], self.instrument[i]['CFF_visu_2'], self.mec, self.instrument[i]['IRC'] = Create_Magnetic_wire(root_node=self.root_node,
                                                                                                     T_start_sim=self.instrument[i]['pose'], young_modulus_body=self.instrument[i]['y_modulus_wire'], outer_diam=self.instrument[i]['outer_diameter_wire'],
                                                                                                    young_modulus_tip=self.instrument[i]['y_modulus_wire'], num=self.instrument[i]['num'], length_density=self.instrument[i]['length_density'],
                                                                                                        inner_diam=self.instrument[i]['inner_diameter_wire'], info=info)

            if info[2][i]["object"] == "Manipulator_estimate":
                self.instrument.append(
                    {'robot': None, 'theta': None, 'link_pose_list': list(), 'position_active': None,
                     'base_pose': info[2][i]["manipulator_base_pose"],
                     'pose': info[2][i]["pose"],
                     "object": "Manipulator_estimate",
                     'moment': info[2][i]["moment"],
                     'visual': info[2][i]["visual"]})
                self.instrument[i]['position_active'] = Create_Magnetic_sphere_dingwei(self.root_node,
                                                                                       self.instrument[i]['pose'], info=info)

                self.instrument[i]['robot'], self.instrument[i]['theta'], self.instrument[i][
                    'link_pose_list'] = Create_Manipulator_dingwei(self.root_node,
                                                                   self.instrument[i]['base_pose'],
                                                                   self.instrument[i]['pose'],
                                                                   self.instrument[i]['visual'])

            #循迹脚本
            if info[2][i]["object"] == "trajectory_ball":
                self.instrument.append({'position_active': None,
                                        "object": "trajectory_ball",
                                        'pose': info[2][i]["pose"],
                                        'moment': info[2][i]["moment"]})
                self.instrument[i]['position_active'] = Create_Magnetic_sphere_dingwei(self.root_node,
                                                                                       self.instrument[i]['pose'], info)

            # if info[2][i]["object"] == "mag_still_beikong":
            #     self.instrument.append({'position_active': None, 'force_torque_capsule': None,
            #                             'pose': info[2][i]["pose"],
            #                             "object": "mag_still",
            #                             'moment': info[2][i]["moment"], 'totalMass': info[2][i]["mag_still_mass"],
            #                             'volume': 4.18879e-6,
            #                             'inertiaMatrix': info[2][i]["inertiaMatrix"]})
            #     self.instrument[i]['position_active'], self.instrument[i][
            #         'force_torque_mag_still'] = Create_Mag_Still_Active(self.root_node, self.instrument[i]['pose'], self.instrument[i]['totalMass'],
            #         self.instrument[i]['volume'], self.instrument[i]['inertiaMatrix'])

            # self.pose = pose_estimation(root_node, info[2][i]["pose"])
        # 磁源
        # -5e-4
        # force = np.array([[0], [0], [self.instrument[0]['totalMass'] * 9.8 - 0.0148]])
        for i in range(len(info[1])):
            if info[1][i]["drive_yuan"] == "yidong_rob_mag" and (not info[2]):
                base_pose = info[1][i]["manipulator_base_pose"]
                magnet_pose = base_pose[:]
                magnet_pose[0] = magnet_pose[0] + 0.0
                magnet_pose[1] = magnet_pose[1]
                magnet_pose[2] = magnet_pose[2] + 1.2
                self.magnetic_source.append({'robot': None, 'theta': None, 'link_pose_list': list(),
                                             'moment': info[1][i]["moment"],
                                             'base_pose': info[1][i]["manipulator_base_pose"],
                                             'magnet_pose': magnet_pose,
                                             'visual': info[1][i]["visual"]})
                self.magnetic_source[i]['robot'], self.magnetic_source[i]['theta'], self.magnetic_source[i][
                    'link_pose_list'] = Create_Manipulator(self.root_node, self.magnetic_source[i]['base_pose'],
                                                           self.magnetic_source[i]['magnet_pose'],
                                                           self.magnetic_source[i]['visual'],
                                                           X0=[1.0, -1.0, 1.0, 1.0, 1.0, 1.0]
                                                           )
            elif info[1][i]["drive_yuan"] == "yidong_rob_mag" and info[2][i]["object"] == "capsule":
                # 下面磁源的参数
                moment = info[1][i]["moment"]
                pc, mc = pose_to_position_moment(self.instrument[0]['pose'])
                mc = mc * self.instrument[0]['moment']
                b_pose = Back_Solve_force_moment(moment, mc, pc)
                magnet_pose = b_pose.back_numerical(
                    force=np.array([[0], [0], [self.instrument[0]['totalMass'] * 9.8 - self.instrument[0]["flotage"]]]))

                self.magnetic_source.append({'robot': None, 'theta': None, 'link_pose_list': list(),
                                             'moment': info[1][i]["moment"],
                                             'base_pose': info[1][i]["manipulator_base_pose"],
                                             'magnet_pose': magnet_pose,
                                             'visual': info[1][i]["visual"]})
                self.magnetic_source[i]['robot'], self.magnetic_source[i]['theta'], self.magnetic_source[i][
                    'link_pose_list'] = Create_Manipulator(self.root_node, self.magnetic_source[i]['base_pose'],
                                                           self.magnetic_source[i]['magnet_pose'],
                                                           self.magnetic_source[i]['visual'],
                                                           X0=[1.0, -1.0, -2.0, 1.0, 1.0, 1.0])

            elif info[1][i]["drive_yuan"] == "yidong_rob_mag" and info[2][i]["object"] == "wire":
                moment = info[1][i]["moment"]
                pc, mc_ = pose_to_position_moment(self.instrument[0]['pose'])
                b_pose = Back_Solve_field(moment, mc_, pc)
                # b_pose = Back_Solve_force_hat_field(moment, mc_, pc)
                if info[8]:
                    if info[8][0]["control_type"] == "wire_kaihuan":
                        # print(mc_)
                        magnet_pose = b_pose.back_numerical(field=mc_ * info[8][0]["field"])
                        # magnet_pose = b_pose.back_numerical(force_hat=mc_+np.array([[0], [0], [1]]), field=mc_ * info[8][0]["field"])

                        # magnet_pose = b_pose.back_numerical(force_hat=np.array([[0], [-0.707], [0.707]]), field=mc_ * info[8][0]["field"])
                        # print(mc_ * info[8][0]["field"])
                else:
                    magnet_pose = b_pose.back_numerical(field=mc_ * 0.001414)

                self.magnetic_source.append({'robot': None, 'theta': None, 'link_pose_list': list(),
                                             'moment': info[1][i]["moment"],
                                             'base_pose': info[1][i]["manipulator_base_pose"],
                                             'magnet_pose': magnet_pose,
                                             'visual': info[1][i]["visual"]})

                self.magnetic_source[i]['robot'], self.magnetic_source[i]['theta'], self.magnetic_source[i][
                    'link_pose_list'] = Create_Manipulator(self.root_node, self.magnetic_source[i]['base_pose'],
                                                           self.magnetic_source[i]['magnet_pose'],
                                                           self.magnetic_source[i]['visual'],
                                                           # X0=[-1.753613, -1.65546861, -1.5911363, 2.99254862, -1.44792851, 2.26469802]) #实验用关节角初始值
                                                           X0=[1.0, -1.0, 1.0, -1.0, 1.0, -1.0]) #肺部介入关节角初始迭代之

            if info[1][i]["drive_yuan"] == "mag_still_qudong":
                self.magnetic_source.append({'moment': info[1][i]["moment"],
                                             'magnet_pose': info[1][i]["pose"],
                                             'mag_still_qudong': None})
                self.magnetic_source[i]['magnet_pose'] = Create_Mag_Still_Visual(self.root_node, self.magnetic_source[i]['magnet_pose'])


                # 机械臂底座可视化
                # visu_object(self.root_node, path="model/box.stl",
                #             pose=[0.0, 1.1, 0.02, 0.0, 0.7071067811865475, 0.0, -0.7071067811865475], scale="0.0009",
                #             color=[0.6509803, 0.6509803, 0.6509803, 1])

                # visu_object(self.root_node, path="model/magnetic source/box.stl",
                #             pose=[self.magnetic_source[0]["base_pose"][0], self.magnetic_source[0]["base_pose"][1] + 0,
                #                   self.magnetic_source[0]["base_pose"][2] - 0.725, 0.0, 0.7071067811865475, 0.0,
                #                   -0.7071067811865475], scale="0.0009",
                #             color=[0.6509803, 0.6509803, 0.6509803, 1])

                # Create_Bu(self.root_node)

            if info[1][i]["drive_yuan"] == "still_Helmholtz_Maxwell":
            # if info[1][i]["drive_yuan"] == "still_Helmholtz_Maxwell" and info[2][i]["object"] == "capsule":
                # 电磁铁控制时，需要用到这个
                self.magnetic_source.append({'Helmholtz': None, 'Maxwell': None, 'tran_elc': info[1][i]["tran_elc"], 'radius': info[1][i]["radius"], 'turns': info[1][i]["turns"]})
                self.magnetic_source[i]['Helmholtz'], self.magnetic_source[i]['Maxwell'] = Create_Helmholtz_Maxwell(
                    self.root_node,
                    self.magnetic_source[i]['tran_elc'],
                    self.magnetic_source[i]['radius'],
                    self.magnetic_source[i]['turns'])
                self.show_pose = Create_Show(self.root_node)
                self.current_number = len(info[1][i]["turns"])

        list_radius = []
        list_turns = []
        list_trans = []
        for i in range(len(info[1])):
            if info[1][i]["drive_yuan"] == "still_elc":
                list_radius.append(info[1][i]["radius"])
                list_turns.append(info[1][i]["turns"])
                list_trans.append(info[1][i]["tran_elc"])

        for i in range(len(info[1])):
            if info[1][0]["drive_yuan"] == "still_elc":
                self.magnetic_source.append({'electromagnet': None, 'tran_elc': list_trans, 'radius': list_radius, 'turns': list_turns})
                self.magnetic_source[0]['electromagnet'] = Create_Electromagnet(self.root_node,
                                                                                self.magnetic_source[0]['tran_elc'],
                                                                                self.magnetic_source[0]['radius'],
                                                                                self.magnetic_source[0]['turns'])
            self.current_number = len(list_turns)

        # 传感器
        for i in range(len(info[3])):
            self.sensor.append({'pose_sensor': Create_Sensor2(self.root_node, info[3][i]["trans_sensor"]),
                                "sensitivity": info[3][i]["noise_sensor"]})
            # print(self.sensor[i]['pose_sensor'][0])

        # 环境
        for i in range(len(info[0])):
            if info[0] == []:
                pass
            elif info[0][i]["env"] == "rigid_active" or info[0][i]["env"] == "rigid_static":
                # 下面是环境的参数
                if info[0][i]["env"] == "rigid_active":
                    self.totalMass = info[0][i]["totalMass"]
                    self.inertiaMatrix = info[0][i]["inertiaMatrix"]
                    self.isAStaticObject = False
                else:
                    self.totalMass = 1
                    self.inertiaMatrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]
                    self.isAStaticObject = True
                self.path = info[0][i]["file_path"]
                self.pose_env = info[0][i]["trans_env"]
                self.color = info[0][i]["color"]
                self.scale = info[0][i]["scale"]
                self.flipnormals = info[0][i]["flipnormals"]

                rigid_object(self.root_node, path=self.path,
                             pose=self.pose_env, totalMass=[self.totalMass, 1, self.inertiaMatrix], scale=self.scale,
                             color=self.color, isAStaticObject=self.isAStaticObject, flipnormals=self.flipnormals)

            elif info[0][i]["env"] == "soft_active" or info[0][i]["env"] == "soft_static":
                if info[0][i]["env"] == "soft_active":
                    self.isAStaticObject = False
                    self.boxroi = None
                else:
                    self.isAStaticObject = True
                    self.boxroi = info[0][i]["indices"]
                self.filepath = info[0][i]["surfaceMeshFilePath"]
                self.file_volume_path = info[0][i]["volumeMeshFilePath"]
                self.trans_env = info[0][i]["trans_env"]
                self.scale = info[0][i]["scale"]
                self.color = info[0][i]["color"]
                self.totalMass = info[0][i]["mass"]
                self.poissonRatio = info[0][i]["poissonRatio"]
                self.youngModulus = info[0][i]["youngModulus"]

                soft_object(self.root_node, path=self.filepath, volume_path=self.file_volume_path, pose=self.trans_env,
                            totalMass=self.totalMass, poissonRatio=self.poissonRatio, youngModulus=self.youngModulus,
                            scale=self.scale, color=self.color, isAStaticObject=self.isAStaticObject, boxroi=self.boxroi)

            elif info[0][i]["env"] == "visu":
                self.path = info[0][i]["file_path"]
                self.pose_env = info[0][i]["trans_env"]
                self.scale = info[0][i]["scale"]
                self.color = info[0][i]["color"]

                visu_object(self.root_node, path=self.path, pose=self.pose_env, scale=self.scale, color=self.color)

            elif info[0][i]["env"] == "build-in":
                if info[0][i]["file_name"] == "tank":
                    Create_Tank(self.root_node, info[0][i]["trans_env"])
        # floor = Floor(self.root_node, name="Floor", translation=[0.0, 0.4, 0.3], rotation=[90.0, 0.0, 0.0], uniformScale=0.005, isAStaticObject=True)
