import numpy as np

from Sofa_Interface.sofa_interface import *
from Magnetic_Engine.permanent_magnet import *
from Pose_Transform.pose_transform import *
from scipy.linalg import pinv

from Control_Package.controller_custom import *

sofa_interface = Sofa_Interface()

# PID参数由用户自己设定
# pid = Pid(kp=4, ki=0, kd=4, dt=0.03, coefficient=1.53e-3)
# # pid1 = Pid(kp=4, ki=0, kd=4, dt=0.03, coefficient=1.53e-5)
# pid1 = Pid(kp=4, ki=0, kd=4, dt=0.03, coefficient=1.53e-11)
flag = 0

pc_last = None

k = 0


def Create_Animate_Capsule_CloseLoop(scene, i, path, pose_estimate, pid, pid1):
    # 全自由度闭环控制
    # 机械臂驱动胶囊
    # 通过机械臂初始状态推出驱动磁铁的初始状态
    pa_, ma_hat_ = scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
    ma_norm = scene.magnetic_source[0]['moment']
    ma = ma_hat_ * scene.magnetic_source[0]['moment']

    # # 胶囊的位姿
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['pose_capsule'].position[0])
    mc = mc_hat * scene.instrument[0]['moment']

    # 模拟真实世界受到的力和力矩
    gravity_compensation = -5e-4
    force_cal, moment_cal = force_moment(pc - pa_, ma, mc, gravity_compensation)
    v_pos = np.array([scene.instrument[0]['velocity_active'][0:3]]).transpose()
    v_angle = np.array([scene.instrument[0]['velocity_active'][3:6]]).transpose()
    force = force_cal - v_pos * 1e-3
    moment = moment_cal - v_angle * 1e-10
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_capsule'], force_cal, moment_cal)

    # 用户定位得到的pc和姿态 和 认为受到的力和磁力矩
    p = pc - pa_

    force, moment = force_moment(p, ma, mc, gravity_compensation)

    # # 想要的位置和姿态
    p_desired = path.p_desired
    h_hat_desired = path.h_hat_desired

    # 计算误差
    err_p = p_desired - pc
    h_hat_desired = np.array(h_hat_desired)
    mc_hat = np.array(mc_hat)
    h_hat_desired_temp = h_hat_desired.transpose()[0]
    mc_hat_temp = mc_hat.transpose()[0]
    # err_moment = np.arccos(
    #     np.dot(h_hat_desired_temp, mc_hat_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)))
    if np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)) > 1:
        err_moment = 0
    else:
        err_moment = np.arccos(np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)))

    # 得到应该施加的力和力矩
    force_desired = pid.pid(err_p)
    moment_desired = np.cross(mc_hat_temp, h_hat_desired_temp) / np.linalg.norm(
        np.cross(mc_hat_temp, h_hat_desired_temp)) * pid1.pid(err_moment)
    moment_desired = np.mat(moment_desired).transpose()

    force_change = force_desired - force
    moment_change = moment_desired - moment

    f_m_change = np.mat(np.concatenate((force_change, moment_change), axis=0))

    jf_val = force_moment_jacob(p, ma, mc)
    jf_val = jf_val[0:6, 0:6]
    jf_val[0:6, 0:6] = jf_val[0:6, 0:6]
    jf_val[0:6, 3:6] = jf_val[0:6, 3:6] * ma_norm

    global pc_last
    if i == 3:
        pc_last = scene.instrument[0]["pose"]

    pc_now = scene.instrument[0]['position_active'][:]
    # print("yesssssssssssssssssssssssssssssssssssssssssssssssssssss")
    # pc_change = pc_now - pc_last
    pc_change = np.array([[pc_now[0] - pc_last[0]], [pc_now[1] - pc_last[1]], [pc_now[2] - pc_last[2]], [0], [0], [0]])
    # pc_change = np.mat(sofa_interface.get_velocity(scene.instrument[0]['pose_capsule'])) * scene.dt
    pc_last = pc_now
    # print(pc_last)
    # global flag
    # flag += 1
    # if flag == 1:
    #     scene.root_node.addObject('OglLabel', label=np.shape(jf_val), fontsize="10", color="contrast",
    #                               prefix="test: ", y="500")
    d_change = f_m_change - jf_val * pc_change

    ja_val = scene.magnetic_source[0]['robot'].fkine_jacob(scene.magnetic_source[0]['theta'])

    jfa = jf_val * np.mat([[-1, 0, 0, 0, 0, 0], [0, -1, 0, 0, 0, 0], [0, 0, -1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]]) * ja_val

    theta_change = np.mat(np.linalg.pinv(jfa)) * d_change

    scene.magnetic_source[0]['theta'] = scene.magnetic_source[0]['theta'] + np.array(theta_change)

    # 对磁导航方案的位姿进行更新
    pose_list = scene.magnetic_source[0]['robot'].fkine_all_link(scene.magnetic_source[0]['theta'])
    for i in range(len(scene.magnetic_source[0]['link_pose_list'])):
        scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]


def Create_Animate_Capsule_OpenLoop(scene, i, path, pose_estimate, pid):
    # # 通过机械臂初始状态推出驱动磁铁的初始状态
    pa_, ma_hat_ = scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
    ma_norm = scene.magnetic_source[0]['moment']
    ma = ma_hat_ * scene.magnetic_source[0]['moment']

    # 胶囊的位姿
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    gravity = scene.instrument[0]['totalMass'] * 9.8
    flotage = scene.instrument[0]['flotage']
    # print(pc)
    mc_norm = scene.instrument[0]['moment']
    mc = mc_hat * mc_norm
    # 模拟真实世界受到的力和力矩
    # gravity_compensation = -5e-4
    force, moment = force_moment(pc - pa_, ma, mc)
    # 物体受到浮力
    force = force + np.array([[0], [0], [flotage]])
    v_pos = np.array([scene.instrument[0]['velocity_active'][0:3]]).transpose()
    v_angle = np.array([scene.instrument[0]['velocity_active'][3:6]]).transpose()
    force = force - v_pos * 1e-3
    moment = moment - v_angle * 1e-10
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_capsule'], force, moment)

    # 用户定位得到的pc和认为受到的力和磁场方向
    pc = pc
    p = pc - pa_
    force, direction = force_direction(p, ma, mc_norm)
    force = force - np.array([[0], [0], [gravity - flotage]])

    p_desired = path.p_desired
    h_hat_desired = path.h_hat_desired

    # 计算误差
    err = p_desired - pc

    # 得到应该施加的力和磁场方向
    force_desired = pid.pid(err)
    force_change = force_desired - force
    h_hat_change = h_hat_desired - direction

    f_h_change = np.mat(np.concatenate((force_change, h_hat_change), axis=0))

    jf_val = force_direction_jacob(p, ma, mc_norm)
    jf_val[0:3, 0:6] = jf_val[0:3, 0:6]
    jf_val[0:6, 3:6] = jf_val[0:6, 3:6] * ma_norm

    # global pc_last
    # if i == 3:
    #     pc_last = scene.instrument[0]["pose"]
    #
    # pc_now = scene.instrument[0]['position_active'][:]

    # pc_change = np.array([[pc_now[0] - pc_last[0]], [pc_now[1] - pc_last[1]], [pc_now[2] - pc_last[2]], [0], [0], [0]])
    pc_change = np.mat(sofa_interface.get_velocity(scene.instrument[0]['velocity_active'])) * scene.dt
    # pc_last = pc_now

    d_change = f_h_change - jf_val * pc_change

    ja_val = scene.magnetic_source[0]['robot'].fkine_jacob(scene.magnetic_source[0]['theta'])

    jfa = jf_val * np.mat([[-1, 0, 0, 0, 0, 0], [0, -1, 0, 0, 0, 0], [0, 0, -1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]]) * ja_val

    theta_change = np.mat(np.linalg.pinv(jfa)) * d_change

    scene.magnetic_source[0]['theta'] = scene.magnetic_source[0]['theta'] + np.array(theta_change)

    # 对磁导航方案的位姿进行更新
    pose_list = scene.magnetic_source[0]['robot'].fkine_all_link(scene.magnetic_source[0]['theta'])
    for i in range(len(scene.magnetic_source[0]['link_pose_list'])):
        scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]


def Create_Animate_Electromagnet_CloseLoop(scene, path, pose_estimate, pid, pid1):
    # 位置和姿态都是闭环控制
    # 创建电磁铁对象
    e = scene.magnetic_source[0]['electromagnet']
    # 胶囊的位姿
    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['pose_active'])

    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    gravity = scene.instrument[0]['totalMass'] * 9.8
    flotage = scene.instrument[0]['flotage']

    mc = mc_hat * scene.instrument[0]['moment']
    # 定位得到的胶囊位姿
    # pc_location = pc + np.random.normal(0, 0.0002, (3, 1))
    pc_location = pc
    mc_location = mc

    # 想要的位置和姿态
    p_desired = path.p_desired
    h_hat_desired = path.h_hat_desired

    # 计算误差
    err_p = p_desired - pc_location
    h_hat_desired = np.array(h_hat_desired)
    mc_hat = np.array(mc_hat)
    h_hat_desired_temp = h_hat_desired.transpose()[0]
    # print(h_hat_desired_temp.shape)
    mc_hat_temp = mc_hat.transpose()[0]
    if np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)) > 1:
        err_m = 0
    else:
        err_m = np.arccos(np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)))

    moment_desired = np.array([[0], [0], [0]])
    # 得到应该施加的力和力矩
    force_desired = pid.pid(err_p) + np.array([[0], [0], [gravity - flotage]])
    if (np.cross(mc_hat_temp, h_hat_desired_temp))[0] != 0 or (np.cross(mc_hat_temp, h_hat_desired_temp))[1] != 0 or \
            (np.cross(mc_hat_temp, h_hat_desired_temp))[2] != 0:
        moment_desired = pid1.pid(np.cross(mc_hat_temp, h_hat_desired_temp) / np.linalg.norm(
            np.cross(mc_hat_temp, h_hat_desired_temp)) * err_m)
        moment_desired = np.mat(moment_desired).transpose()

    moment_force = np.mat(np.concatenate((moment_desired, force_desired), axis=0))
    # global k
    # k = k + 1
    # if k == 10:
    #     scene.root_node.addObject('OglLabel', label=moment_force, fontsize="10", color="contrast",
    #                               prefix="force_moment: ", updateLabelEveryNbSteps="5", y="150")
    current = e.back_analytic(pc_location, mc_location, moment_force)
    scene.current = current
    # scene.show_pose.position[0][:] = [1, 1, 1, 0, 0, 0, 1]
    # scene.show_pose.name = str(current.transpose()[0])
    scene.current = current

    moment_force = e.moment_force(pc_location, mc_location, current)
    force = moment_force[3:6, 0:1]
    moment = moment_force[0:3, 0:1]
    force = force + np.array([[0], [0], [flotage]])
    v_pos = np.array([scene.instrument[0]['velocity_active'][0:3]]).transpose()
    v_angle = np.array([scene.instrument[0]['velocity_active'][3:6]]).transpose()
    force = force - v_pos * 1e-3
    moment = moment - v_angle * 1e-10
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_capsule'], force,
                                    moment)


def Create_Animate_Electromagnet_OpenLoop(scene, path, pose_estimate, pid, pid1):
    # 姿态开环控制
    # 创建电磁铁对象
    e = scene.magnetic_source[0]['electromagnet']

    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['position_active'])
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    mc = mc_hat * scene.instrument[0]['moment']

    gravity = scene.instrument[0]['totalMass'] * 9.8
    flotage = scene.instrument[0]['flotage']
    # 定位得到的胶囊位姿
    # pc_location = pc + np.random.normal(0, 0.0002, (3, 1))
    pc_location = pc
    mc_location = mc

    # # 想要的位置和姿态
    p_desired = path.p_desired
    # field_desired = path.h_hat_desired * 1e-8
    field_desired = path.h_hat_desired * 1e-6
    # print(field_desired)

    # 计算误差 # 位置
    err_p = p_desired - pc_location
    force_desired = pid.pid(err_p)

    force_desired = force_desired + np.array([[0], [0], [gravity - flotage]])
    field_force = np.mat(np.concatenate((field_desired, force_desired), axis=0))
    # print("aaaaaa")
    # print(field_force)
    # print("aaaaaa")
    current = e.back_analytic_field_force(pc_location, mc_location, field_force)
    scene.show_pose.name = str(current.transpose()[0])
    scene.current = current

    field_force = e.field_force(pc_location, mc_location, current)
    # print("bbbbb")
    # print(field_force)
    # print("bbbbb")
    # np.cross(mc_hat_temp, h_hat_desired_temp)
    moment = np.cross(mc.transpose(), field_force[0:3, 0:1].transpose()).transpose()
    force = field_force[3:6, 0:1]
    force = force + np.array([[0], [0], [flotage]])
    v_pos = np.array([scene.instrument[0]['velocity_active'][0:3]]).transpose()
    v_angle = np.array([scene.instrument[0]['velocity_active'][3:6]]).transpose()
    force = force - v_pos * 1e-3
    # print(v_angle)
    moment = moment - v_angle * 1e-9
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_capsule'], force, moment)


def Create_Animate_wire_rob(scene, path, pose_estimate, field_norm):
    # 机械臂驱动导丝
    # 通过机械臂初始状态推出驱动磁铁的初始状态
    ma_norm = scene.magnetic_source[0]["moment"]

    field_des = path.h_hat_desired * field_norm
    mc_norm = scene.instrument[0]['moment']

    Pj = pose_estimate.p_pose
    mj_ = pose_estimate.h_hat_pose
    # pos = scene.instrument[0]['position_active']
    # Pj, mj_ = pose_to_position_moment(pos)
    mj = mj_ * mc_norm
    # print(Pj)

    Pi, mi_ = scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
    # print(Pi, mi_)
    mi = mi_ * ma_norm
    # print(np.linalg.norm(Pj - Pi))
    # print(Pj - Pi)

    FF, T = force_moment(Pj - Pi, mi, mj)
    b, c = field_gradient(Pj - Pi, mi)
    FF_hat = FF / np.linalg.norm(FF)  # 磁力单位化
    # print(np.linalg.norm(FF))
    # print(FF)
    field_hat = field_des / linalg.norm(field_des)  # 磁场单位化
    # print(moment_to_angle(field_des))

    # scene.instrument[0]['force_torque_wire'].forces[0][:] = [FF[0, 0]/1, FF[1, 0]/1, FF[2, 0]/1 - 0.00168, T[0, 0], T[1, 0], T[2, 0]]
    # scene.instrument[0]['force_torque_wire'].forces[0][:] = [0, 0, 0, T[0, 0], T[1, 0], T[2, 0]]

    # 更新雅可比矩阵以及伪逆矩阵
    jf_val = force_hat_field_jacob(Pj, Pi, mi_, mj_, ma_norm)
    ja_val = scene.magnetic_source[0]['robot'].fkine_jacob(scene.magnetic_source[0]['theta'])
    jfa = jf_val * ja_val
    jfa = np.array(jfa, dtype=np.float32)
    ja__matrix = np.mat(pinv(jfa))

    # 计算变量delta并更新theta,磁源位姿
    delta_bx = field_des[0, 0] - b[0, 0]
    delta_by = field_des[1, 0] - b[1, 0]
    delta_bz = field_des[2, 0] - b[2, 0]

    field_hat = field_hat + +np.array([[0], [0], [1]])
    field_hat = field_hat / np.linalg.norm(field_hat)
    delta_fx = field_hat[0, 0] - FF_hat[0, 0]  # 计划使力的方向与磁场方向保持一致
    delta_fy = field_hat[1, 0] - FF_hat[1, 0]
    delta_fz = field_hat[2, 0] - FF_hat[2, 0]


    # mj_ = mj_ + +np.array([[0], [0], [1]])
    # mj_ = mj_ / np.linalg.norm(mj_)
    # delta_fx = mj_[0, 0] - FF_hat[0, 0]  # 计划使力的方向与磁场方向保持一致
    # delta_fy = mj_[1, 0] - FF_hat[1, 0]
    # delta_fz = mj_[2, 0] - FF_hat[2, 0]

    delta__matrix = np.mat([[delta_fx], [delta_fy], [delta_fz], [delta_bx], [delta_by], [delta_bz]])
    # print(delta__matrix)
    delta_theta = ja__matrix * delta__matrix

    if np.linalg.norm(delta__matrix[0:3, :]) > 4e-3:
        scene.magnetic_source[0]['theta'] = np.array(delta_theta) + scene.magnetic_source[0]['theta']
    # print(scene.magnetic_source[0]['theta'])

    # angle = np.degrees(np.arccos(
    #     np.dot(np.array([mj_[0, 0], mj_[1, 0], mj_[2, 0]]), field_des.transpose()[0]) / np.linalg.norm(field_des)))
    # scene.mec.position[0] = angle
    # print(angle)
    # print(path.h_hat_desired)
    # print(FF_hat)

    # 对磁导航方案的位姿进行更新
    pose_list = scene.magnetic_source[0]['robot'].fkine_all_link(scene.magnetic_source[0]['theta'])
    for i in range(len(scene.magnetic_source[0]['link_pose_list'])):
        scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]

    Pi, mi_ = scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
    mi = mi_ * ma_norm
    FF, T = force_moment(Pj - Pi, mi, mj)
    # scene.instrument[0]['force_torque_wire'].forces[0][:] = [FF[0, 0] / 1 + scene.instrument[0]['wire_Mechanical'].velocity[100][0]*0.0004,
    #                                                          FF[1, 0] / 1 + scene.instrument[0]['wire_Mechanical'].velocity[100][1]*0.0004,
    #                                                          FF[2, 0] / 1 - 0.001678 + scene.instrument[0]['wire_Mechanical'].velocity[100][2]*0.0004,
    #                                                          T[0, 0], T[1, 0], T[2, 0]]
    scene.instrument[0]['force_torque_wire'].forces[0][:] = [FF[0, 0] / 1, FF[1, 0] / 1, FF[2, 0] / 1 - 0.001878,
                                                             T[0, 0], T[1, 0], T[2, 0]]
    # print(scene.instrument[0]['wire_Mechanical'].velocity[100][0])


def Create_Animate_wire_rob_2(scene, path, pose_estimate, field_norm):
    # 机械臂驱动导丝
    # 通过机械臂初始状态推出驱动磁铁的初始状态
    ma_norm = scene.magnetic_source[0]["moment"]

    field_des = path.h_hat_desired * field_norm
    mc_norm = scene.instrument[0]['moment']

    Pj = pose_estimate.p_pose
    mj_ = pose_estimate.h_hat_pose
    # pos = scene.instrument[0]['position_active']
    # Pj, mj_ = pose_to_position_moment(pos)
    mj = mj_ * mc_norm

    Pi, mi_ = scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
    mi = mi_ * ma_norm
    # print(Pi)
    # print(mi_)

    FF, T = force_moment(Pj - Pi, mi, mj)
    # print(np.linalg.norm(Pj - Pi))
    b, c = field_gradient(Pj - Pi, mi)

    scene.instrument[0]['force_torque_wire'].forces[0][:] = [0,0,0, T[0, 0], T[1, 0], T[2, 0]]

    # 更新雅可比矩阵以及伪逆矩阵
    jf_val = field_jacob(Pj, Pi, mi_, ma_norm)
    ja_val = scene.magnetic_source[0]['robot'].fkine_jacob(scene.magnetic_source[0]['theta'])
    jfa = jf_val * ja_val
    jfa = np.array(jfa, dtype=np.float32)
    ja__matrix = np.mat(pinv(jfa))

    # 计算变量delta并更新theta,磁源位姿
    delta_bx = field_des[0, 0] - b[0, 0]
    delta_by = field_des[1, 0] - b[1, 0]
    delta_bz = field_des[2, 0] - b[2, 0]

    delta__matrix = np.mat([[delta_bx], [delta_by], [delta_bz]])
    delta_theta = ja__matrix * delta__matrix

    scene.magnetic_source[0]['theta'] = np.array(delta_theta) + scene.magnetic_source[0]['theta']
    # print(scene.magnetic_source[0]['theta'].T / 180 * 3.14)

    # 对磁导航方案的位姿进行更新
    pose_list = scene.magnetic_source[0]['robot'].fkine_all_link(scene.magnetic_source[0]['theta'])
    for i in range(len(scene.magnetic_source[0]['link_pose_list'])):
        scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]


def Create_Animate_wire_elc_field_ignore_or_force(scene, path, pose_estimate, field_norm):
    # 姿态开环控制
    # 创建电磁铁对象
    e = scene.magnetic_source[0]['electromagnet']

    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['position_active'])
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    mc = mc_hat * scene.instrument[0]['moment']

    # 定位得到的胶囊位姿
    # pc_location = pc + np.random.normal(0, 0.0002, (3, 1))
    pc_location = pc
    mc_location = mc

    # # 想要的姿态
    field_desired = path.h_hat_desired * field_norm
    # print(field_desired)

    # 期望磁力为零，只需匀强磁场即可
    force_desired = np.array([[0], [0], [0]])
    field_force = np.mat(np.concatenate((field_desired, force_desired), axis=0))

    current = e.back_analytic_field_force(pc_location, mc_location, field_force)
    # current = e.back_analytic_field_ignore(pc_location, mc_location, field_desired)
    scene.show_pose.name = str(current.transpose()[0])
    scene.current = current

    field_force = e.field_force(pc_location, mc_location, current)
    # print(field_force)

    moment = np.cross(mc.transpose(), field_force[0:3, 0:1].transpose()).transpose()
    force = field_force[3:6, 0:1]

    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_wire'], force, moment)


def Create_Animate_wire_elc_moment_ignore_or_force(scene, path, pose_estimate, pid, pid1):
    # 姿态开环控制
    # 创建电磁铁对象
    e = scene.magnetic_source[0]['electromagnet']

    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['position_active'])
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    mc = mc_hat * scene.instrument[0]['moment']

    # 定位得到的胶囊位姿
    # pc_location = pc + np.random.normal(0, 0.0002, (3, 1))
    pc_location = pc
    mc_location = mc


    if np.dot(np.array(mc).transpose()[0], path.h_hat_desired.transpose()[0]) / (np.linalg.norm(path.h_hat_desired.transpose()[0]) * np.linalg.norm(np.array(mc).transpose()[0])) > 1:
        err_m = 0
    else:
        err_m = np.arccos(np.dot(np.array(mc).transpose()[0], path.h_hat_desired.transpose()[0]) / (np.linalg.norm(path.h_hat_desired.transpose()[0]) * np.linalg.norm(np.array(mc).transpose()[0])))
        # print(np.arccos(np.dot(np.array(mc).transpose()[0], path.h_hat_desired.transpose()[0]) / (np.linalg.norm(path.h_hat_desired.transpose()[0]) * np.linalg.norm(np.array(mc).transpose()[0]))))
    moment_desired = np.array([[0], [ 0], [0]])
    if (np.cross(np.array(mc).transpose()[0], path.h_hat_desired.transpose()[0]))[0] != 0 or (np.cross(np.array(mc).transpose()[0], path.h_hat_desired.transpose()[0]))[1] != 0 or (np.cross(np.array(mc).transpose()[0], path.h_hat_desired.transpose()[0]))[2] != 0:
        moment_desired = pid1.pid(np.cross(np.array(mc).transpose()[0], path.h_hat_desired.transpose()[0]) / np.linalg.norm(np.cross(np.array(mc).transpose()[0], path.h_hat_desired.transpose()[0])) * err_m)
        moment_desired = np.mat(moment_desired).transpose()

    err_p = path.p_desired - pc_location
    # print(np.linalg.norm(err_p))
    force_desired = pid.pid(err_p)
    moment_force = np.mat(np.concatenate((moment_desired, force_desired), axis=0))

    if pid.kp == 0:
        current = e.back_analytic_moment_ignore(pc_location, mc_location, moment_desired)
    else:
        current = e.back_analytic(pc_location, mc_location, moment_force)

    scene.show_pose.name = str(current.transpose()[0])
    scene.current = current

    field_force = e.field_force(pc_location, mc_location, current)

    moment = np.cross(mc.transpose(), field_force[0:3, 0:1].transpose()).transpose()
    force = field_force[3:6, 0:1]
    # print(np.linalg.norm(force))
    # force = np.array([[0], [0], [0]])
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_wire'], force, moment)


def Create_Animate_Location(scene, i, path, pose_estimate):
    # 只有定位
    # print("aaaaaaaaaaaaaaaaaaaaaaaaaaa")
    # # 胶囊的位姿
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['pose_capsule'].position[0])
    # mc = mc_hat * scene.instrument[0]['capsule_moment']

    p_desired = path.p_desired
    h_hat_desired = path.h_hat_desired

    scene.instrument[0]["position_active"][:] = position_moment_to_pose(p_desired, h_hat_desired)
    # scene.instrument[0]["pose_capsule"].position[0][:] = position_moment_to_pose(p_desired, h_hat_desired)
    # scene.sensor[0]['pose_sensor'].position[0][0:3] = [p_desired[0, 0], p_desired[1, 0], p_desired[2, 0]]

    X0 = list(scene.instrument[0]['theta'].transpose()[0])
    scene.instrument[0]['theta'] = scene.instrument[0]['robot'].back_numerical(p_desired, h_hat_desired, X0)
    scene.instrument[0]['theta'][5, 0] = 0
    # 对磁导航方案的位姿进行更新
    pose_list = scene.instrument[0]['robot'].fkine_all_link(scene.instrument[0]['theta'])
    for i in range(len(scene.instrument[0]['link_pose_list'])):
        scene.instrument[0]['link_pose_list'][i].position[0][:] = pose_list[i]


def Create_trajectory(scene, i, path, pose_estimate):
    # 只有定位
    # print("aaaaaaaaaaaaaaaaaaaaaaaaaaa")
    # # 胶囊的位姿
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['pose_capsule'].position[0])
    # mc = mc_hat * scene.instrument[0]['capsule_moment']

    p_desired = path.p_desired
    h_hat_desired = path.h_hat_desired

    scene.instrument[0]["position_active"][:] = position_moment_to_pose(p_desired, h_hat_desired)
    # scene.instrument[0]["pose_capsule"].position[0][:] = position_moment_to_pose(p_desired, h_hat_desired)
    # scene.sensor[0]['pose_sensor'].position[0][0:3] = [p_desired[0, 0], p_desired[1, 0], p_desired[2, 0]]

    # X0 = list(scene.instrument[0]['theta'].transpose()[0])
    # scene.instrument[0]['theta'] = scene.instrument[0]['robot'].back_numerical(p_desired, h_hat_desired, X0)
    # scene.instrument[0]['theta'][5, 0] = 0
    # # 对磁导航方案的位姿进行更新
    # pose_list = scene.instrument[0]['robot'].fkine_all_link(scene.instrument[0]['theta'])
    # for i in range(len(scene.instrument[0]['link_pose_list'])):
    #     scene.instrument[0]['link_pose_list'][i].position[0][:] = pose_list[i]


def Create_Animate_Helmholtz_Maxwell_OpenLoop(scene, path, pose_estimate, pid):
    # 姿态开环控制
    # 创建亥姆霍兹和麦克斯韦对象
    h = scene.magnetic_source[0]['Helmholtz']
    m = scene.magnetic_source[0]['Maxwell']

    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['position_active'])
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    mc = mc_hat * scene.instrument[0]['moment']

    gravity = scene.instrument[0]['totalMass'] * 9.8
    flotage = scene.instrument[0]['flotage']

    # 定位得到的胶囊位姿
    pc_location = pc
    mc_location = mc

    # # 想要的位置和姿态
    p_desired = path.p_desired
    field_desired = path.h_hat_desired * 3e-4

    # 计算误差 # 位置
    err_p = p_desired - pc_location
    force_desired = pid.pid(err_p)

    force_desired = force_desired + np.array([[0], [0], [gravity - flotage]])

    current1 = h.back_analytic_field(field_desired)
    current2 = m.back_analytic_force(mc, force_desired)
    scene.current = np.concatenate((current1, current2))

    scene.show_pose.name = str(list(current1.transpose()[0])+list(current2.transpose()[0]))

    field = h.field(current1)
    force = m.force(mc, current2)

    # scene.show_pose.name = str(current.transpose()[0])

    # field_force = e.field_force(pc_location, mc_location, current)
    # print("bbbbb")
    # print(field_force)
    # print("bbbbb")
    # np.cross(mc_hat_temp, h_hat_desired_temp)
    moment = np.cross(mc.transpose(), field.transpose()).transpose()
    force += np.array([[0], [0], [flotage]])

    v_pos = np.array([scene.instrument[0]['velocity_active'][0:3]]).transpose()
    v_angle = np.array([scene.instrument[0]['velocity_active'][3:6]]).transpose()

    # 加上关于速度的阻尼
    force = force - v_pos * 1e-3
    moment = moment - v_angle * 1e-10
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_capsule'], force, moment)


def Create_Only_Mag_Still_OpenLoop(scene, path, pose_estimate):

    mc_norm = scene.instrument[0]['moment']
    Pj = pose_estimate.p_pose
    mj_ = pose_estimate.h_hat_pose
    mj = mj_ * mc_norm

    p_desired = path.p_desired
    # print(p_desired)
    h_hat_desired = path.h_hat_desired

    scene.magnetic_source[0]["magnet_pose"][:] = position_moment_to_pose(p_desired, h_hat_desired)

    ma_norm = scene.magnetic_source[0]["moment"]
    pos = scene.magnetic_source[0]["magnet_pose"]
    Pi, mi_ = pose_to_position_moment(pos)

    # Pi, mi_ = scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
    mi = mi_ * ma_norm

    FF, T = force_moment(Pj - Pi, mi, mj)

    if 'force_torque_wire' in scene.instrument[0]:
        scene.instrument[0]['force_torque_wire'].forces[0][:] = [FF[0, 0], FF[1, 0], FF[2, 0], T[0, 0], T[1, 0], T[2, 0]]
    if 'force_torque_capsule' in scene.instrument[0]:
        scene.instrument[0]['force_torque_capsule'].forces[0][:] = [FF[0, 0], FF[1, 0], FF[2, 0], T[0, 0], T[1, 0], T[2, 0]]
    # print(scene.instrument[0]['force_torque_mag_still'].forces[0][:])


#############################custom#####################################

def Create_Animate_Capsule_Custom(scene, i, path, pose_estimate):
    # 全自由度闭环控制
    # 机械臂驱动胶囊
    # 通过机械臂初始状态推出驱动磁铁的初始状态
    pa_, ma_hat_ = scene.magnetic_source[0]['robot'].fkine(scene.magnetic_source[0]['theta'])
    ma_norm = scene.magnetic_source[0]['moment']
    ma = ma_hat_ * scene.magnetic_source[0]['moment']

    # # 胶囊的位姿
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['pose_capsule'].position[0])
    mc = mc_hat * scene.instrument[0]['moment']

    # 模拟真实世界受到的力和力矩
    gravity_compensation = -5e-4
    force_cal, moment_cal = force_moment(pc - pa_, ma, mc, gravity_compensation)
    v_pos = np.array([scene.instrument[0]['velocity_active'][0:3]]).transpose()
    v_angle = np.array([scene.instrument[0]['velocity_active'][3:6]]).transpose()
    force = force_cal - v_pos * 1e-3
    moment = moment_cal - v_angle * 1e-10
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_capsule'], force_cal, moment_cal)

    # 用户定位得到的pc和姿态 和 认为受到的力和磁力矩
    p = pc - pa_

    # # 想要的位置和姿态
    p_desired = path.p_desired
    h_hat_desired = path.h_hat_desired

    # 计算误差
    err_p = p_desired - pc
    h_hat_desired = np.array(h_hat_desired)
    mc_hat = np.array(mc_hat)
    h_hat_desired_temp = h_hat_desired.transpose()[0]
    mc_hat_temp = mc_hat.transpose()[0]
    # err_moment = np.arccos(
    #     np.dot(h_hat_desired_temp, mc_hat_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)))
    if np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)) > 1:
        err_moment = 0
    else:
        err_moment = np.arccos(np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)))

    if 'custom_controller_p' in globals():
        # 得到应该施加的力和力矩
        force_desired = custom_controller_p(err_p)

    if 'custom_controller_h' in globals():
        force, moment = force_moment(p, ma, mc, gravity_compensation)
        moment_desired = np.cross(mc_hat_temp, h_hat_desired_temp) / np.linalg.norm(
            np.cross(mc_hat_temp, h_hat_desired_temp)) * custom_controller_h(err_moment)
        moment_desired = np.mat(moment_desired).transpose()

        force_change = force_desired - force
        moment_change = moment_desired - moment

        f_m_change = np.mat(np.concatenate((force_change, moment_change), axis=0))

        jf_val = force_moment_jacob(p, ma, mc)
        jf_val = jf_val[0:6, 0:6]
        jf_val[0:6, 0:6] = jf_val[0:6, 0:6]
        jf_val[0:6, 3:6] = jf_val[0:6, 3:6] * ma_norm

        global pc_last
        if i == 3:
            pc_last = scene.instrument[0]["pose"]

        pc_now = scene.instrument[0]['position_active'][:]
        pc_change = np.array([[pc_now[0] - pc_last[0]], [pc_now[1] - pc_last[1]], [pc_now[2] - pc_last[2]], [0], [0], [0]])
        pc_last = pc_now

        d_change = f_m_change - jf_val * pc_change

    else:
        mc_norm = scene.instrument[0]['moment']
        force, direction = force_direction(p, ma, mc_norm)
        force_change = force_desired - force
        h_hat_change = h_hat_desired - direction

        f_h_change = np.mat(np.concatenate((force_change, h_hat_change), axis=0))

        jf_val = force_direction_jacob(p, ma, mc_norm)
        jf_val[0:3, 0:6] = jf_val[0:3, 0:6]
        jf_val[0:6, 3:6] = jf_val[0:6, 3:6] * ma_norm

        # global pc_last
        # if i == 3:
        #     pc_last = scene.instrument[0]["pose"]
        #
        # pc_now = scene.instrument[0]['position_active'][:]

        # pc_change = np.array([[pc_now[0] - pc_last[0]], [pc_now[1] - pc_last[1]], [pc_now[2] - pc_last[2]], [0], [0], [0]])
        pc_change = np.mat(sofa_interface.get_velocity(scene.instrument[0]['velocity_active'])) * scene.dt

        d_change = f_h_change - jf_val * pc_change

    ja_val = scene.magnetic_source[0]['robot'].fkine_jacob(scene.magnetic_source[0]['theta'])

    jfa = jf_val * np.mat([[-1, 0, 0, 0, 0, 0], [0, -1, 0, 0, 0, 0], [0, 0, -1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]]) * ja_val

    theta_change = np.mat(np.linalg.pinv(jfa)) * d_change

    scene.magnetic_source[0]['theta'] = scene.magnetic_source[0]['theta'] + np.array(theta_change)

    # 对磁导航方案的位姿进行更新
    pose_list = scene.magnetic_source[0]['robot'].fkine_all_link(scene.magnetic_source[0]['theta'])
    for i in range(len(scene.magnetic_source[0]['link_pose_list'])):
        scene.magnetic_source[0]['link_pose_list'][i].position[0][:] = pose_list[i]


def Create_Animate_Electromagnet_Custom(scene, path, pose_estimate):
    # 位置和姿态都是闭环控制
    # 创建电磁铁对象
    e = scene.magnetic_source[0]['electromagnet']
    # 胶囊的位姿
    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['pose_active'])

    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    gravity = scene.instrument[0]['totalMass'] * 9.8
    flotage = scene.instrument[0]['flotage']

    mc = mc_hat * scene.instrument[0]['moment']
    # 定位得到的胶囊位姿
    # pc_location = pc + np.random.normal(0, 0.0002, (3, 1))
    pc_location = pc
    mc_location = mc

    # 想要的位置和姿态
    p_desired = path.p_desired
    h_hat_desired = path.h_hat_desired

    # 计算误差
    err_p = p_desired - pc_location
    h_hat_desired = np.array(h_hat_desired)
    mc_hat = np.array(mc_hat)
    h_hat_desired_temp = h_hat_desired.transpose()[0]
    mc_hat_temp = mc_hat.transpose()[0]
    # err_m = np.arccos(
    #     np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)))
    if np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)) > 1:
        err_m = 0
    else:
        err_m = np.arccos(np.dot(mc_hat_temp, h_hat_desired_temp) / (np.linalg.norm(h_hat_desired_temp) * np.linalg.norm(mc_hat_temp)))

    moment_desired = np.array([[0], [0], [0]])

    if 'custom_controller_p' in globals():
        # 得到应该施加的力和力矩
        force_desired = custom_controller_p(err_p) + np.array([[0], [0], [gravity - flotage]])

    if 'custom_controller_h' in globals():
        if (np.cross(mc_hat_temp, h_hat_desired_temp))[0] != 0 or (np.cross(mc_hat_temp, h_hat_desired_temp))[1] != 0 or \
                (np.cross(mc_hat_temp, h_hat_desired_temp))[2] != 0:
            moment_desired = np.cross(mc_hat_temp, h_hat_desired_temp) / np.linalg.norm(
                np.cross(mc_hat_temp, h_hat_desired_temp)) * custom_controller_h(err_m)
            moment_desired = np.mat(moment_desired).transpose()

            moment_force = np.mat(np.concatenate((moment_desired, force_desired), axis=0))
            current = e.back_analytic(pc_location, mc_location, moment_force)
    else:
        #开环
        field_desired = path.h_hat_desired * 1e-6
        field_force = np.mat(np.concatenate((field_desired, force_desired), axis=0))
        current = e.back_analytic_field_force(pc_location, mc_location, field_force)

    scene.current = current

    scene.show_pose.name = str(current.transpose()[0])

    moment_force = e.moment_force(pc_location, mc_location, current)
    force = moment_force[3:6, 0:1]
    moment = moment_force[0:3, 0:1]
    force = force + np.array([[0], [0], [flotage]])
    v_pos = np.array([scene.instrument[0]['velocity_active'][0:3]]).transpose()
    v_angle = np.array([scene.instrument[0]['velocity_active'][3:6]]).transpose()
    force = force - v_pos * 1e-3
    moment = moment - v_angle * 1e-10
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_capsule'], force, moment)


def Create_Animate_Helmholtz_Maxwell_Custom(scene, path, pose_estimate):
    # 姿态开环控制
    # 创建亥姆霍兹和麦克斯韦对象
    h = scene.magnetic_source[0]['Helmholtz']
    m = scene.magnetic_source[0]['Maxwell']

    # pc, mc_hat = pose_to_position_moment(scene.instrument[0]['position_active'])
    pc = pose_estimate.p_pose
    mc_hat = pose_estimate.h_hat_pose
    mc = mc_hat * scene.instrument[0]['moment']

    gravity = scene.instrument[0]['totalMass'] * 9.8
    flotage = scene.instrument[0]['flotage']

    # 定位得到的胶囊位姿
    pc_location = pc
    mc_location = mc

    # # 想要的位置和姿态
    p_desired = path.p_desired
    field_desired = path.h_hat_desired * 3e-4

    # 计算误差 # 位置
    err_p = p_desired - pc_location
    force_desired = custom_controller_p(err_p)

    force_desired = force_desired + np.array([[0], [0], [gravity - flotage]])

    current1 = h.back_analytic_field(field_desired)
    current2 = m.back_analytic_force(mc, force_desired)
    scene.current = np.concatenate((current1, current2))

    scene.show_pose.name = str(list(current1.transpose()[0])+list(current2.transpose()[0]))

    field = h.field(current1)
    force = m.force(mc, current2)

    # scene.show_pose.name = str(current.transpose()[0])

    # field_force = e.field_force(pc_location, mc_location, current)
    # print("bbbbb")
    # print(field_force)
    # print("bbbbb")
    # np.cross(mc_hat_temp, h_hat_desired_temp)
    moment = np.cross(mc.transpose(), field.transpose()).transpose()
    force += np.array([[0], [0], [flotage]])

    v_pos = np.array([scene.instrument[0]['velocity_active'][0:3]]).transpose()
    v_angle = np.array([scene.instrument[0]['velocity_active'][3:6]]).transpose()

    # 加上关于速度的阻尼
    force = force - v_pos * 1e-3
    moment = moment - v_angle * 1e-10
    sofa_interface.set_force_moment(scene.instrument[0]['force_torque_capsule'], force, moment)