import Sofa
from Module_Package.position_estimate_module import *
import shutil


class Position_Estimate(Sofa.Core.Controller):

    def __init__(self, info, scene, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.info = info

        self.p_pose = np.array([[0],[0],[0]])
        self.h_hat_pose = np.array([[0], [0], [0]])

        self.scene = scene

        self.P = np.zeros((9, 9))
        self.pos_filter = np.zeros((9, 1))

        self.EKF_Q = None
        self.EKF_R = None

        self.pos_gra = np.zeros((6, 1))

        self.gra_alpha = None
        self.gra_beta = None
        self.gra_loss = None
        self.gra_num = None

        self.pos_LM = np.zeros((6, 1))

        self.LM_num = None

        self.pos_custom = np.zeros((6, 1))

        p_initial, h_hat_initial = pose_to_position_moment(self.scene.instrument[0]['position_active'][:])

        if self.info[7][0]["algorithm_type"] == "EKF":
            self.pos_filter[0:3] = p_initial
            self.pos_filter[6:9] = np.mat(h_hat_initial * self.info[2][0]['moment'])

        elif self.info[7][0]["algorithm_type"] == "gradient":
            self.pos_gra[0:3] = p_initial
            self.pos_gra[3:6] = np.mat(h_hat_initial * self.info[2][0]['moment'])

        elif self.info[7][0]["algorithm_type"] == "LM":
            self.pos_LM[0:3] = p_initial
            self.pos_LM[3:6] = np.mat(h_hat_initial * self.info[2][0]['moment'])

        elif self.info[7][0]["algorithm_type"] == "custom":
            self.pos_custom[0:3] = p_initial
            self.pos_custom[3:6] = np.mat(h_hat_initial * self.info[2][0]['moment'])

            shutil.copy2(info[7][0]["file_path"], "State_Estimation/estimate_custom.py")

    def onAnimateBeginEvent(self, event):
        if self.info[7][0]["algorithm_type"] == "none":
            # 如果没有定位算法
            # print(self.scene.instrument[0]['position_active'][:])
            self.p_pose, self.h_hat_pose = pose_to_position_moment(
                self.scene.instrument[0]['position_active'][:])
            #bug

        elif self.info[7][0]["algorithm_type"] == "EKF":
            self.EKF_Q = self.info[7][0]["EKF_Q"]
            self.EKF_R = self.info[7][0]["EKF_R"]
            Positioning_EKF(self.scene.sensor, self.scene.instrument[0]['position_active'][:], self.scene.instrument[0]["moment"], self.pos_filter, self.P, self.EKF_Q, self.EKF_R)
            # print(self.scene.instrument[0]['position_active'][:])
            self.p_pose = self.pos_filter[0:3]
            self.h_hat_pose = self.pos_filter[6:9]/np.linalg.norm(self.pos_filter[6:9])
            # print(self.scene.instrument[0]['position_active'][:])
            # print(self.p_pose)

        elif self.info[7][0]["algorithm_type"] == "gradient":
            self.gra_alpha = self.info[7][0]["gra_alpha"]
            self.gra_beta = self.info[7][0]["gra_beta"]
            self.gra_num = self.info[7][0]["gra_num"]
            Position_Optimization_gradient(self.scene.sensor, self.scene.instrument[0]['position_active'][:], self.scene.instrument[0]["moment"], self.pos_gra, self.gra_alpha, self.gra_beta, self.gra_num)
            self.p_pose = self.pos_gra[0:3]
            self.h_hat_pose = self.pos_gra[3:6]/np.linalg.norm(self.pos_gra[3:6])
            # print(self.p_pose)

        elif self.info[7][0]["algorithm_type"] == "LM":
            self.LM_num = self.info[7][0]["LM_num"]
            Position_Optimization_LM(self.scene.sensor, self.scene.instrument[0]['position_active'][:], self.scene.instrument[0]["moment"], self.pos_LM, self.LM_num)
            self.p_pose = self.pos_LM[0:3]
            self.h_hat_pose = self.pos_LM[3:6]/np.linalg.norm(self.pos_LM[3:6])

        elif self.info[7][0]["algorithm_type"] == "custom":
            self.pos_custom[0:3], self.pos_custom[3:6] = Position_Custom(self.scene.sensor, self.scene.instrument[0]['position_active'][:], self.scene.instrument[0]["moment"])
            self.p_pose = self.pos_custom[0:3]
            self.h_hat_pose = self.pos_custom[3:6]/np.linalg.norm(self.pos_custom[3:6])

        # self.scene.pose.position[0][:] = position_moment_to_pose(np.array(self.p_pose), np.array(self.h_hat_pose))