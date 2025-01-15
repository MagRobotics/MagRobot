import json

import Monitor
import simulator_setting
import create_scene
import post_processing
import animate_controller
import path_controller
import position_estimate
import field_show
import record


def createScene(root_node):

    f = open("GUI_interface/ceshi.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_capsule_elc_soft_stomach_nc_high.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_capsule_elc_center.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_location.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_Helmholtz_Maxwell.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_elc_hard_brains_nc_peitu1.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_capsule_elc_soft_stomach.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_elc_soft_brains_nc.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_elc_hard_bronchus.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_rob_hard_bronchus2_nc.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_elc_hard_arc.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_elc_soft_arc_nc.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_capsule_rob.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_paper.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_elc_soft_arc_nc_key.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_elc_soft_arc4_nc.txt", "r", encoding="UTF-8")
    # f = open("GUI_interface/info_wire_rob_hard_bronchus_shiyan.txt", "r", encoding="UTF-8")

    # f = open("GUI_interface/video/info_wire_rob_hard_bronchus_nc.txt", "r", encoding="UTF-8")

    json_str = f.read()
    initial_info = json.loads(json_str)

    # 环境
    simulator = simulator_setting.Simulator(root_node, info=initial_info)

    # 场景
    scene = create_scene.Scene(root_node, info=initial_info)

    path = None
    pose_estimate = None

    if initial_info[6]:
        # 路径
        path = path_controller.Path(root_node, initial_info, scene)

    if initial_info[7]:
        # 定位
        pose_estimate = position_estimate.Position_Estimate(initial_info, scene)
        root_node.addObject(pose_estimate)

    if path and pose_estimate:
        # 循环
        mag_controller = animate_controller.MagController(root_node, info=initial_info, simulator=simulator, scene=scene, path=path, pose_estimate=pose_estimate)
        root_node.addObject(mag_controller)

        # 后处理
        post = post_processing.Post_processing(scene, path, pose_estimate, mag_controller=mag_controller, environment=simulator)
        root_node.addObject(post)

        # 监视
        # monitor = Monitor.Monitor(root_node, scene, post)
        # root_node.addObject(monitor)

    # 磁场显示
    # if initial_info[5][0]["field_show"] != 0:
    #     for i in range(len(initial_info[5][0]["field_show"])):
    #         field = field_show.magnetic_field_show(root_node, initial_info[5][0]["field_show"][i], scene, initial_info)
    #         root_node.addObject(field)

    # 操作记录
    # re = record.Operate_Reocrd(root_node, scene, path)
    # root_node.addObject(re)

