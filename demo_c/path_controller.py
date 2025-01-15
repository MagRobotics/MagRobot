from Module_Package.path_controller_module import *


class Path:
    def __init__(self, root_node, info, scene):
        self.root_node = root_node
        self.p_desired = np.array([[0.0], [0.0], [0.0]])
        self.h_hat_desired = np.array([[0.0], [0.0], [0.0]])
        self.dt = info[5][0]["step"]
        self.key = None
        #为了方便永磁铁静止的情况
        if info[1]:
            if info[1][0]["drive_yuan"] == "mag_still_qudong":
                self.p_desired, self.h_hat_desired = pose_to_position_moment(info[1][0]["pose"])
                self.p_desired, self.h_hat_desired = np.array([[float(self.p_desired[0, 0])], [float(self.p_desired[1, 0])], [float(self.p_desired[2, 0])]]),\
                                                     np.array([[float(self.h_hat_desired[0, 0])], [float(self.h_hat_desired[1, 0])], [float(self.h_hat_desired[2, 0])]])
            else:
                for i in range(len(info[2])):
                    self.p_desired, self.h_hat_desired = pose_to_position_moment(info[2][i]["pose"])
                    self.p_desired, self.h_hat_desired = np.array(
                        [[float(self.p_desired[0, 0])], [float(self.p_desired[1, 0])], [float(self.p_desired[2, 0])]]), \
                                                         np.array([[float(self.h_hat_desired[0, 0])],
                                                                   [float(self.h_hat_desired[1, 0])],
                                                                   [float(self.h_hat_desired[2, 0])]])
        else:
            for i in range(len(info[2])):
                self.p_desired, self.h_hat_desired = pose_to_position_moment(info[2][i]["pose"])
                self.p_desired, self.h_hat_desired = np.array([[float(self.p_desired[0, 0])], [float(self.p_desired[1, 0])], [float(self.p_desired[2, 0])]]),\
                                                     np.array([[float(self.h_hat_desired[0, 0])], [float(self.h_hat_desired[1, 0])], [float(self.h_hat_desired[2, 0])]])

        if info[6][0]['path_type'] == 'key':
            self.path_des_pos = self.root_node.addObject('MechanicalObject', name="path_des_pos", template="Rigid3", position=[0, 0, 0, 0, 0, 0, 1], showObject=True, showObjectScale=0.005)
            self.key = KeyBoardController(self.p_desired, self.h_hat_desired, self.dt, self.path_des_pos)
            root_node.addObject(self.key)

        if info[6][0]['path_type'] == 'geomagic':
            self.root_node.addObject('GeomagicDriver', name="GeomagicDevice", scale=info[6][0]['scale'], drawDevice="1", drawDeviceFrame="1", positionBase=info[6][0]['pose'][0:3], orientationBase=info[6][0]['pose'][3:7])
            self.path_des_pos = self.root_node.addObject('MechanicalObject', name="path_des_pos", template="Rigid3", position="@GeomagicDevice.positionDevice", showObject=True, showObjectScale=0.01)
            geo = Geomagic(self.p_desired, self.h_hat_desired, self.path_des_pos, scene)
            root_node.addObject(geo)

        if info[6][0]['path_type'] == 'yushe':
            yushecontroller = YusheController(self.p_desired, self.h_hat_desired, self.dt, info, scene)
            root_node.addObject(yushecontroller)

        if info[6][0]['path_type'] == 'planning':
            planning = Planning(self.root_node, self.p_desired, self.h_hat_desired, self.dt, info, scene)
            root_node.addObject(planning)


