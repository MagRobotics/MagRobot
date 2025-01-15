import Sofa
from Module_Package.animate_module import *
from Control_Package.Pid import *
import shutil
# dt = 0.03


class MagController(Sofa.Core.Controller):

    def __init__(self, root_node, info, simulator, scene, path, pose_estimate, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.simulator = simulator
        self.scene = scene
        self.root_node = root_node
        self.i = 0
        self.path = path
        self.pose_estimate = pose_estimate
        self.info = info

        if info[8] == []:
            pass
        elif info[8][0]["control_type"] == "PID":
            # 第一个pid是位置的pid
            self.pid = Pid(kp=info[8][0]["kp"], ki=info[8][0]["ki"], kd=info[8][0]["kd"], dt=self.scene.dt, coefficient=1.53e-3)

            # 第二个pid是姿态的pid
            if self.info[2][0]["object"] == "capsule":
                self.pid1 = Pid(kp=info[8][0]["kp2"], ki=info[8][0]["ki2"], kd=info[8][0]["kd2"], dt=self.scene.dt, coefficient=1.53e-11)
            else:
                self.pid1 = Pid(kp=info[8][0]["kp2"], ki=info[8][0]["ki2"], kd=info[8][0]["kd2"], dt=self.scene.dt, coefficient=1.53e-5)

        elif info[8][0]["control_type"] == "wire_kaihuan":
            self.field_norm = info[8][0]["field"]

        elif info[8][0]["control_type"] == "custom":
            shutil.copy2(info[8][0]["file_path"], "Control_Package/controller_custom.py")

        self.sofa_interface = Sofa_Interface()

    def onAnimateBeginEvent(self, event):
        self.i += self.scene.dt*100

        if self.info[2][0]["object"] == "Manipulator_estimate":
            Create_Animate_Location(self.scene, self.i, self.path, self.pose_estimate)

        elif self.info[2][0]["object"] == "trajectory_ball":
            Create_trajectory(self.scene, self.i, self.path, self.pose_estimate)

        elif 'robot' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule" and self.info[8][0]["control_type"] == "PID":
            # Create_Animate_Capsule_CloseLoop(self.scene, self.i, self.path, self.pose_estimate, self.pid, self.pid1)
            Create_Animate_Capsule_OpenLoop(self.scene, self.i, self.path, self.pose_estimate, self.pid)

        elif 'electromagnet' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule" and self.info[8][0]["control_type"] == "PID":
            Create_Animate_Electromagnet_CloseLoop(self.scene, self.path,  self.pose_estimate, self.pid, self.pid1)
            # Create_Animate_Electromagnet_OpenLoop(self.scene, self.path, self.pose_estimate, self.pid, self.pid1)

        elif 'Helmholtz' in self.scene.magnetic_source[0] and 'Maxwell' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule" and self.info[8][0]["control_type"] == "PID":
            Create_Animate_Helmholtz_Maxwell_OpenLoop(self.scene, self.path, self.pose_estimate, self.pid)

        elif 'robot' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "wire" and self.info[8][0]["control_type"] == "wire_kaihuan":
            Create_Animate_wire_rob_2(self.scene, self.path, self.pose_estimate, self.field_norm)
            # Create_Animate_wire_rob(self.scene, self.path, self.pose_estimate, self.field_norm)

        elif 'electromagnet' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "wire" and self.info[8][0]["control_type"] == "wire_kaihuan":
            Create_Animate_wire_elc_field_ignore_or_force(self.scene, self.path, self.pose_estimate, self.field_norm)

        elif 'electromagnet' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "wire" and self.info[8][0]["control_type"] == "PID":
            Create_Animate_wire_elc_moment_ignore_or_force(self.scene, self.path, self.pose_estimate, self.pid, self.pid1)

        elif "mag_still_qudong" in self.scene.magnetic_source[0] and self.info[2] and self.info[8] != []:
            Create_Only_Mag_Still_OpenLoop(self.scene, self.path, self.pose_estimate)

        #custom#####################################################
        elif 'robot' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule" and self.info[8][0]["control_type"] == "custom":
            Create_Animate_Capsule_Custom(self.scene, self.i, self.path, self.pose_estimate)

        elif 'electromagnet' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule" and self.info[8][0]["control_type"] == "custom":
            Create_Animate_Electromagnet_Custom(self.scene, self.path, self.pose_estimate)

        elif 'Helmholtz' in self.scene.magnetic_source[0] and 'Maxwell' in self.scene.magnetic_source[0] and self.info[2][0]["object"] == "capsule" and self.info[8][0]["control_type"] == "custom":
            Create_Animate_Helmholtz_Maxwell_Custom(self.scene, self.path, self.pose_estimate)
        # custom#####################################################

        #内窥镜
        if self.info[5][0]['cam'] == 1:
            if self.info[2][0]["object"] == 'capsule':
                pos = self.scene.instrument[0]['position_active']
                self.simulator.cam.cameraRigid.value = [pos[0], pos[1], pos[2], 0.707*pos[3]-0.707*pos[5], 0.707*pos[4]+0.707*pos[6], 0.707*pos[3]+0.707*pos[5], -0.707*pos[4]+0.707*pos[6]]
                # self.simulator.light.position.value = [pos[0], pos[1], pos[2]]
            if self.info[2][0]["object"] == 'wire':
                pos = self.scene.instrument[0]['position_active']
                self.simulator.cam.cameraRigid.value = [pos[0], pos[1], pos[2], -pos[4], pos[3], pos[5], pos[6]]
            # self.simulator.cam.cameraRigid.value = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]]