import numpy as np

from Pose_Transform.pose_transform import *
import Sofa
from Trajectory_Package.trajectory import *
import json
import keyboard


class KeyBoardController(Sofa.Core.Controller):
    def __init__(self, p_desired, h_hat_desired, dt, path_des_pos, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.p_desired = p_desired
        self.h_hat_desired = h_hat_desired
        # print(self.p_desired)
        self.i = 0
        self.dt = dt
        self.phi, self.theta = moment_to_angle(self.h_hat_desired)

        self.path_des_pos = path_des_pos

        self.record_label = 0

    def onKeypressedEvent(self, event):
        key0 = event['key']

        self.key(key0)

    def key(self, key):
        # print(ord(key))
        # 前：19 后：21 左：18 右: 20
        # print(self.p_desired)
        position_step = 0.02 * self.dt  # 0.05代表一直按着键盘，仿真1s时间移动0.05m
        # angle_step = 20 * np.pi/180*self.dt  # 15代表一直按着键盘，仿真1s时间转动15度
        angle_step = 10 * np.pi/180*self.dt  # 实验用10度

        if ord(key) == 20:
            self.p_desired[0, 0] = self.p_desired[0, 0] - position_step

        if ord(key) == 18:
            self.p_desired[0, 0] = self.p_desired[0, 0] + position_step

        if ord(key) == 19:
            self.p_desired[1, 0] = self.p_desired[1, 0] - position_step

        if ord(key) == 21:
            self.p_desired[1, 0] = self.p_desired[1, 0] + position_step

        # +：43 -：45
        if ord(key) == 43:
            self.p_desired[2, 0] = self.p_desired[2, 0] + position_step

        if ord(key) == 45:
            self.p_desired[2, 0] = self.p_desired[2, 0] - position_step

        # I:73  J:74  K:75  L:76
        if ord(key) == 73:
            self.phi -= angle_step
            self.h_hat_desired[0:3, 0:1] = angle_to_moment(self.phi, self.theta)
            self.record_label = 1

        if ord(key) == 75:
            self.phi += angle_step
            self.h_hat_desired[0:3, 0:1] = angle_to_moment(self.phi, self.theta)
            self.record_label = 2

        if ord(key) == 74:
            self.theta += angle_step
            self.h_hat_desired[0:3, 0:1] = angle_to_moment(self.phi, self.theta)
            self.record_label = 3

        if ord(key) == 76:
            self.theta -= angle_step
            self.h_hat_desired[0:3, 0:1] = angle_to_moment(self.phi, self.theta)
            self.record_label = 4

        # self.path_des_pos.position[0][:] = position_moment_to_pose(self.p_desired, self.h_hat_desired)
        self.path_des_pos.position[0][:] = position_moment_to_pose(np.array([[-0.05], [0.37], [1.0]]), self.h_hat_desired)

class YusheController(Sofa.Core.Controller):

    def __init__(self, p_desired, h_hat_desired, dt, info, scene, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.p_desired = p_desired
        self.h_hat_desired = h_hat_desired
        self.p_initial = p_desired.copy()
        self.h_hat_initial = h_hat_desired.copy()
        self.i = 0
        # self.j = 0
        self.dt = dt
        self.info = info
        self.scene = scene
        self.custom_pose = []
        if self.info[6][0]["type"] == "custom":
            # f = open("..\\..\\demo_c\\post_processing\\trajectory_output.txt", "r", encoding="UTF-8")
            f = open("post_processing/trajectory_output.txt", "r", encoding="UTF-8")
            json_str = f.read()
            self.custom_pose = json.loads(json_str)
            # print(self.custom_pose)

            p_start = np.array([[self.custom_pose[0][0]], [self.custom_pose[1][0]], [self.custom_pose[2][0]]])

            r = R.from_euler('xyz', [self.custom_pose[3][0], self.custom_pose[4][0], self.custom_pose[5][0]], degrees=True)
            # 将欧拉角转换为旋转矩阵
            rotation_matrix = r.as_matrix()
            h_start = np.array(rotation_matrix * np.mat([[1], [0], [0]]))

            self.scene.instrument[0]['position_active'][:] = position_moment_to_pose(p_start, h_start)

    def trajectory(self, time, type, velocity=0.004, p_initial=np.array([[0],[0],[0]]), h_hat_initial=np.array([[0],[0],[1]])):
        if type=="grid_path":
            # "三次多项式轨迹"
            t1 = 0.05 / velocity
            t2 = 0.02 / velocity
            T = int(time / (0.14 / velocity))
            x_bias = -0.04 * T
            t = time % (0.14 / velocity)
            if 0 <= t < t1:
                return np.array([[p_initial[0, 0] + x_bias], [p_initial[1, 0]], [p_initial[2, 0] - polynomial_three(0.05, t1, t)]]), h_hat_initial
            if t1 <= t < t1 + t2:
                return np.array([[p_initial[0, 0] + x_bias - polynomial_three(0.02, t2, (t - t1))], [p_initial[1, 0]], [p_initial[2, 0]-0.05]]), h_hat_initial
            if t1 + t2 <= t < 2 * t1 + t2:
                return np.array([[p_initial[0, 0]-0.02 + x_bias], [p_initial[1, 0]], [p_initial[2, 0]-0.05 + polynomial_three(0.05, t1, (t - t1 - t2))]]), h_hat_initial
            if 2 * t1 + t2 <= t < 2 * (t1 + t2):
                return np.array(
                    [[p_initial[0, 0]-0.02 + x_bias - polynomial_three(0.02, t2, (t - 2 * t1 - t2))], [p_initial[1, 0]], [p_initial[2, 0]]]), h_hat_initial

        if type == "spiral_path":
            # "带抛物线过渡"
            R = 0.04
            N = 0.04
            alpha = math.atan2(N, 2 * np.pi * R)
            if 0 <= time < 6:
                # 四秒时间加速
                L = soft_trajectory(0.1, velocity, velocity / 4, time)
            else:
                L = velocity * 4 + velocity * (time - 6)
            H = L * np.sin(alpha)
            W = L * np.cos(alpha)

            x = 0.04 * np.cos(W / R)
            y = 0.04 * np.sin(W / R)

            return np.array([[p_initial[0, 0]-0.04 + x], [p_initial[1, 0] + y], [p_initial[2, 0] + H]]), h_hat_initial

    def onAnimateBeginEvent(self, event):
        self.i += self.dt * 100
        # print(self.i)
        # self.j += 1

        if self.info[6][0]["type"] == "custom":

            r = R.from_euler('xyz', [self.custom_pose[3][int(self.i)], self.custom_pose[4][int(self.i)], self.custom_pose[5][int(self.i)]], degrees=True)
            # 将欧拉角转换为旋转矩阵
            rotation_matrix = r.as_matrix()

            self.p_desired[0:3], self.h_hat_desired[0:3] = np.array([[self.custom_pose[0][int(self.i)]], [self.custom_pose[1][int(self.i)]], [self.custom_pose[2][int(self.i)]]]), np.array(rotation_matrix * np.mat([[1], [0], [0]]))
            # self.p_desired[0:3], self.h_hat_desired[0:3] = np.array[[self.custom_pose[0][self.j]], [
            #     self.custom_pose[1][self.j]], [self.custom_pose[2][self.j]]], np.array[
            #                                                    [self.custom_pose[3][self.j]], [
            #                                                        self.custom_pose[4][self.j]], [
            #                                                        self.custom_pose[5][self.j]]]

        elif self.info[6][0]["type"] == "geshan":
            # 栅格轨迹
            self.p_desired[0:3], self.h_hat_desired[0:3] = self.trajectory(self.i / 100, type="grid_path", velocity=0.006,
                                                                      p_initial=self.p_initial,
                                                                      h_hat_initial=self.h_hat_initial)

        elif self.info[6][0]["type"] == "luoxuan":
            # 螺旋线
            self.p_desired[0:3], self.h_hat_desired[0:3] = self.trajectory(self.i / 100, type="spiral_path", velocity=0.03,
                                                                      p_initial=self.p_initial,
                                                                      h_hat_initial=self.h_hat_initial)

        # 圆弧轨迹
        # self.p_desired[0:3], self.h_hat_desired[0:3] = trajectory(self.i / 100, type="spiral_path", velocity=0.03, p_initial=self.p_initial, h_hat_initial=self.h_hat_initial)


class Planning(Sofa.Core.Controller):
    def __init__(self, root_node, p_desired, h_hat_desired, dt, info, scene, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root_node = root_node
        self.p_desired = p_desired
        self.h_hat_desired = h_hat_desired
        self.i = 1
        self.dt = dt
        self.info = info
        self.scene = scene
        self.planning_pose = []
        quat = self.info[6][0]["pose"][3:7]
        rot_matrix = quat_to_rmatrix(quat)
        rot_matrix = np.mat(rot_matrix)
        if self.info[6][0]["type"] == "centerline":
            f = open("post_processing/centerline.txt", "r", encoding="UTF-8")
            json_str = f.read()
            self.planning_pose = json.loads(json_str)

            for j in range(len(self.planning_pose[0])):
                pose_initial = np.mat([[self.planning_pose[0][j]], [self.planning_pose[1][j]], [self.planning_pose[2][j]]]) * self.info[6][0]["scale"]
                pose_trans = rot_matrix * pose_initial + np.mat([[self.info[6][0]["pose"][0]], [self.info[6][0]["pose"][1]], [self.info[6][0]["pose"][2]]])
                self.planning_pose[0][j] = pose_trans[0][0].item()
                self.planning_pose[1][j] = pose_trans[1][0].item()
                self.planning_pose[2][j] = pose_trans[2][0].item()

        elif self.info[6][0]["type"] == "centerline_avoid":
            f = open("post_processing/centerline_avoid.txt", "r", encoding="UTF-8")
            json_str = f.read()
            self.planning_pose = json.loads(json_str)

            for j in range(len(self.planning_pose[0])):
                pose_initial = np.mat([[self.planning_pose[0][j]], [self.planning_pose[1][j]], [self.planning_pose[2][j]]]) * self.info[6][0]["scale"]
                pose_trans = rot_matrix * pose_initial + np.mat([[self.info[6][0]["pose"][0]], [self.info[6][0]["pose"][1]], [self.info[6][0]["pose"][2]]])
                self.planning_pose[0][j] = pose_trans[0][0].item()
                self.planning_pose[1][j] = pose_trans[1][0].item()
                self.planning_pose[2][j] = pose_trans[2][0].item()

        #位姿初始化（与起点一致）
        p_start = np.array([[self.planning_pose[0][0]], [self.planning_pose[1][0]], [self.planning_pose[2][0]]])
        h_start = np.array([[self.planning_pose[0][1]-self.planning_pose[0][0]], [self.planning_pose[1][1]-self.planning_pose[1][0]], [self.planning_pose[2][1]-self.planning_pose[2][0]]])

        if self.info[2][0]["object"] == "capsule":
            self.scene.instrument[0]['position_active'][:] = position_moment_to_pose(p_start, h_start)

        if self.info[2][0]["object"] == "wire":

            self.scene.instrument[0]['IRC'].startingPos[0] = position_moment_to_pose(p_start, h_start)[0]
            self.scene.instrument[0]['IRC'].startingPos[1] = position_moment_to_pose(p_start, h_start)[1]
            self.scene.instrument[0]['IRC'].startingPos[2] = position_moment_to_pose(p_start, h_start)[2]
            self.scene.instrument[0]['IRC'].startingPos[3] = position_moment_to_pose(p_start, h_start)[3]
            self.scene.instrument[0]['IRC'].startingPos[4] = position_moment_to_pose(p_start, h_start)[4]
            self.scene.instrument[0]['IRC'].startingPos[5] = position_moment_to_pose(p_start, h_start)[5]
            self.scene.instrument[0]['IRC'].startingPos[6] = position_moment_to_pose(p_start, h_start)[6]

            restPos = []
            for pos in self.scene.instrument[0]['wire_Mechanical'].rest_position.value:
                restPos.append(position_moment_to_pose(p_start, h_start))
            self.scene.instrument[0]['wire_Mechanical'].rest_position.value = restPos

            self.centerline_length = 0.001
            self.single = 1

            # self.center_point_1 = self.root_node.addChild('center_point_1')
            # self.center_point_1_pos = self.center_point_1.addObject('MechanicalObject', name="center_point_1", template="Rigid3", position=[self.planning_pose[0][0], self.planning_pose[1][0], self.planning_pose[2][0], 0, 0, 0, 1], showObject=1, showObjectScale=0.01)

    def started(self):
        # if self.single == 1:
        #     print(1)
            keyboard.press('up')
            # self.single = 0

    def closed(self):
        keyboard.release('up')
        self.single = 1

    def onAnimateBeginEvent(self, event):
        if self.info[2][0]["object"] == "capsule":
            if self.i < len(self.planning_pose[0])-1:
                self.p_desired[0:3] = np.array([[self.planning_pose[0][int(self.i)]], [self.planning_pose[1][int(self.i)]], [self.planning_pose[2][int(self.i)]]])
                h_desired = np.array([[self.planning_pose[0][int(self.i)]-self.planning_pose[0][int(self.i)-1]], [self.planning_pose[1][int(self.i)]-self.planning_pose[1][int(self.i)-1]],
                                                    [self.planning_pose[2][int(self.i)]-self.planning_pose[2][int(self.i)-1]]])

                self.h_hat_desired[0:3] = h_desired / np.linalg.norm(h_desired)

                self.i += 1
            else:
                pass

        if self.info[2][0]["object"] == "wire":

            if self.i < len(self.planning_pose[0]) - 1:

                if self.scene.instrument[0]['IRC'].xtip[0] < self.centerline_length:
                    self.started()
                else:
                    self.closed()
                    self.i += 1

                    #无用
                    self.p_desired[0:3] = np.array(
                        [[self.planning_pose[0][int(self.i)]], [self.planning_pose[1][int(self.i)]],
                         [self.planning_pose[2][int(self.i)]]])

                    h_desired = np.array([[self.planning_pose[0][int(self.i)+1] - self.planning_pose[0][int(self.i) - 1]],
                                          [self.planning_pose[1][int(self.i)+1] - self.planning_pose[1][int(self.i) - 1]],
                                          [self.planning_pose[2][int(self.i)+1] - self.planning_pose[2][int(self.i) - 1]]])
                    self.h_hat_desired[0:3] = h_desired / np.linalg.norm(h_desired)

                    # self.center_point_1_pos.position[0][:] = position_moment_to_pose(self.p_desired, self.h_hat_desired)

                    self.centerline_length += np.linalg.norm(np.array(
                        [[self.planning_pose[0][int(self.i)] - self.planning_pose[0][int(self.i) - 1]],
                         [self.planning_pose[1][int(self.i)] - self.planning_pose[1][int(self.i) - 1]],
                         [self.planning_pose[2][int(self.i)] - self.planning_pose[2][int(self.i) - 1]]]))



class Geomagic(Sofa.Core.Controller):
    def __init__(self, p_desired, h_hat_desired, path_des_pos, scene, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.p_desired = p_desired
        self.h_hat_desired = h_hat_desired

        self.path_des_pos = path_des_pos

        self.scene = scene

        # self.scene.instrument[0]['position_active'][:] = self.path_des_pos.position[0][:]
        #-z方向为h_hat方向  并未将touch_pos赋值给物体初始位姿  需要用户自行配置（后面可以加上去）

    def onAnimateBeginEvent(self, event):
        self.p_desired[0:3], self.h_hat_desired[0:3] = pose_to_position_moment_z(self.path_des_pos.position[0][:])


