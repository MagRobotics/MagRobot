import Sofa
import keyboard
import json
import numpy as np
from Pose_Transform.pose_transform import *


class Operate_Reocrd(Sofa.Core.Controller):

    def __init__(self, root_node, scene, path, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.root_node = root_node
        self.scene = scene
        self.path = path
        self.length_record = 0
        self.data = []
        self.data_turn = []
        self.data_turn_b1 = []
        self.data_turn_b2 = []
        self.data_turn_b3 = []
        self.i = 0
        self.j = 0

        # self.h_hat_desired = self.path.key.h_hat_desired
        # self.phi, self.theta = moment_to_angle(self.h_hat_desired)
        # self.angle_step = 10 * np.pi/180*0.05  # 15代表一直按着键盘，仿真1s时间转动15度

        f = open("post_processing/data_record_bronchus.txt", "r", encoding="UTF-8")
        json_str = f.read()
        self.data_read = json.loads(json_str)

        f = open("post_processing/data_record_turn_b1.txt", "r", encoding="UTF-8")
        json_str = f.read()
        self.data_read_turn_b1 = json.loads(json_str)
        f = open("post_processing/data_record_turn_b2.txt", "r", encoding="UTF-8")
        json_str = f.read()
        self.data_read_turn_b2 = json.loads(json_str)
        f = open("post_processing/data_record_turn_b3.txt", "r", encoding="UTF-8")
        json_str = f.read()
        self.data_read_turn_b3 = json.loads(json_str)

        # f = open("post_processing/data_record_experiment.txt", "r", encoding="UTF-8")
        # json_str = f.read()
        # self.data_read = json.loads(json_str)
        #
        # f = open("post_processing/data_record_turn_experiment.txt", "r", encoding="UTF-8")
        # json_str = f.read()
        # self.data_read_turn = json.loads(json_str)

    def onAnimateBeginEvent(self, event):

        # 支气管
        # if 330.0 < self.root_node.time.value / self.root_node.findData('dt').value < 332.0:
        #     self.root_node.animate.value = 0
        #
        # if 545.0 < self.root_node.time.value / self.root_node.findData('dt').value < 546.0:
        #     self.root_node.animate.value = 0
        #
        # if 735.0 < self.root_node.time.value / self.root_node.findData('dt').value < 736.0:
        #     self.root_node.animate.value = 0
        #
        # if 955.0 < self.root_node.time.value / self.root_node.findData('dt').value < 956.0:
        #     self.root_node.animate.value = 0
        #
        # if 1130.0 < self.root_node.time.value / self.root_node.findData('dt').value < 1131.0:
        #     self.root_node.animate.value = 0
        #
        # # self.data.append(self.scene.instrument[0]['IRC'].xtip[0])
        # # with open("post_processing\\data_record_bronchus.txt", 'w') as f:
        # #     f.write(str(self.data))
        #
        # num = round((self.data_read[self.i]-self.length_record)/0.0007)
        # for _ in range(num):
        #     keyboard.press_and_release('ctrl+up')
        # self.length_record = self.data_read[self.i]
        #
        # #转向
        # # self.data_turn_b1.append(self.h_hat_desired[0, 0])
        # # with open("post_processing\\data_record_turn_b1.txt", 'w') as f:
        # #     f.write(str(self.data_turn_b1))
        # # self.data_turn_b2.append(self.h_hat_desired[1, 0])
        # # with open("post_processing\\data_record_turn_b2.txt", 'w') as f:
        # #     f.write(str(self.data_turn_b2))
        # # self.data_turn_b3.append(self.h_hat_desired[2, 0])
        # # with open("post_processing\\data_record_turn_b3.txt", 'w') as f:
        # #     f.write(str(self.data_turn_b3))
        #
        # self.h_hat_desired[0, 0] = self.data_read_turn_b1[self.i]
        # self.h_hat_desired[1, 0] = self.data_read_turn_b2[self.i]
        # self.h_hat_desired[2, 0] = self.data_read_turn_b3[self.i]
        #
        # self.i += 1



        # # #主动脉弓
        # if 309.0 < self.root_node.time.value / self.root_node.findData('dt').value < 310.0:
        #     self.root_node.animate.value = 0
        #
        # if 429.0 < self.root_node.time.value / self.root_node.findData('dt').value < 430.0:
        #     self.root_node.animate.value = 0
        #
        # if 499.0 < self.root_node.time.value / self.root_node.findData('dt').value < 500.0:
        #     self.root_node.animate.value = 0
        #
        # if 589.0 < self.root_node.time.value / self.root_node.findData('dt').value < 590.0:
        #     self.root_node.animate.value = 0
        #
        # if 669.0 < self.root_node.time.value / self.root_node.findData('dt').value < 670.0:
        #     self.root_node.animate.value = 0
        #
        # self.data.append(self.scene.instrument[0]['IRC'].xtip[0])
        # with open("post_processing\\data_record_arc.txt", 'w') as f:
        #     f.write(str(self.data))
        #
        # # num = round((self.data_read[self.i]-self.length_record)/0.0007)
        # # for _ in range(num):
        # #     keyboard.press_and_release('ctrl+up')
        # # self.length_record = self.data_read[self.i]
        #
        # self.i += 1



        # #主动脉弓
        if 1124 < self.root_node.time.value / self.root_node.findData('dt').value < 1125:
            self.root_node.animate.value = 0

        if 1480 < self.root_node.time.value / self.root_node.findData('dt').value < 1481:
            self.root_node.animate.value = 0

        if 1803 < self.root_node.time.value / self.root_node.findData('dt').value < 1804:
            self.root_node.animate.value = 0

        if 2050 < self.root_node.time.value / self.root_node.findData('dt').value < 2051:
            self.root_node.animate.value = 0

        if 2350 < self.root_node.time.value / self.root_node.findData('dt').value < 2351:
            self.root_node.animate.value = 0

        self.data.append(self.scene.instrument[0]['IRC'].xtip[0])
        with open("post_processing\\data_record_arc.txt", 'w') as f:
            f.write(str(self.data))

        # num = round((self.data_read[self.i]-self.length_record)/0.0007)
        # for _ in range(num):
        #     keyboard.press_and_release('ctrl+up')
        # self.length_record = self.data_read[self.i]

        self.i += 1




        # 消化道
        # if 130.0 < self.root_node.time.value / self.root_node.findData('dt').value < 131.0:
        #     self.root_node.animate.value = 0
        #
        # if 225.0 < self.root_node.time.value / self.root_node.findData('dt').value < 226.0:
        #     self.root_node.animate.value = 0
        #
        # if 329.0 < self.root_node.time.value / self.root_node.findData('dt').value < 330.0:
        #     self.root_node.animate.value = 0
        #
        # if 429.0 < self.root_node.time.value / self.root_node.findData('dt').value < 430.0:
        #     self.root_node.animate.value = 0
        #
        # if 525.0 < self.root_node.time.value / self.root_node.findData('dt').value < 526.0:
        #     self.root_node.animate.value = 0
        #
        # self.i += 1




        # 实验测试
        # # if 350.0 < self.root_node.time.value / self.root_node.findData('dt').value < 351.0:
        # #     self.root_node.animate.value = 0
        # #
        # # if 450.0 < self.root_node.time.value / self.root_node.findData('dt').value < 451.0:
        # #     self.root_node.animate.value = 0
        # #
        # # if 550.0 < self.root_node.time.value / self.root_node.findData('dt').value < 551.0:
        # #     self.root_node.animate.value = 0
        #
        # # self.data.append(round(self.scene.instrument[0]['IRC'].xtip[0], 6))
        # # with open("post_processing\\data_record_experiment.txt", 'w') as f:
        # #     f.write(str(self.data))
        #
        # num = round((self.data_read[self.i]-self.length_record)/0.0004)
        # if num > 0:
        #     for _ in range(num):
        #         keyboard.press_and_release('ctrl+up')
        # if num < 0:
        #     num = abs(num)
        #     for _ in range(num):
        #         keyboard.press_and_release('ctrl+down')
        # self.length_record = self.data_read[self.i]
        #
        # #转向
        #
        # # self.data_turn.append(self.path.key.record_label)
        # # with open("post_processing\\data_record_turn_experiment.txt", 'w') as f:
        # #     f.write(str(self.data_turn))
        # # self.path.key.record_label = 0
        #
        # # if self.data_read_turn[self.i] == 1:
        # #     keyboard.press_and_release('ctrl+i')
        # # if self.data_read_turn[self.i] == 2:
        # #     keyboard.press_and_release('ctrl+i')
        # # if self.data_read_turn[self.i] == 3:
        # #     # self.theta += self.angle_step
        # #     # self.h_hat_desired[0:3, 0:1] = angle_to_moment(self.phi, self.theta)
        # #     # print(self.j)
        # #     # self.j += 1
        # #     keyboard.press_and_release('ctrl+j')
        # # if self.data_read_turn[self.i] == 4:
        # #     # self.theta -= self.angle_step
        # #     # self.h_hat_desired[0:3, 0:1] = angle_to_moment(self.phi, self.theta)
        # #     # print(self.j)
        # #     # self.j -= 1
        # #     keyboard.press_and_release('ctrl+l')
        #
        # # self.data_turn.append(round(moment_to_angle(self.h_hat_desired)[1], 6))
        # # with open("post_processing\\data_record_turn_experiment.txt", 'w') as f:
        # #     f.write(str(self.data_turn))
        #
        # num = round((self.data_read_turn[self.i] - self.phi) / self.angle_step)
        # if num > 0:
        #     for _ in range(num):
        #         keyboard.press_and_release('ctrl+j')
        # if num < 0:
        #     num = abs(num)
        #     for _ in range(num):
        #         keyboard.press_and_release('ctrl+l')
        # self.phi = self.data_read_turn[self.i]
        #
        # self.i += 1
