import numpy as np

from Manipulator_Kinematics.kinematics import *
from Magnetic_Engine.electromagnet import *
from Environment_Package.environment import *


def Create_Magnetic_wire(
        root_node,
        info,
        T_start_sim,
        name='mag_instrument',
        outer_diam=0.00133,
        inner_diam=0.0008,
        young_modulus_body=170e6,
        young_modulus_tip=21e6,
        num=33,
        length_density=0.015,
        nume_nodes_viz=600,
        fixed_directions=[0, 0, 0, 0, 0, 0],
        color=[0.6098, 0.6098, 0.6098, 1.0]):
        # color=[0.7098, 0.7098, 0.7098, 1.0]):

    outer_diam_qu = ((outer_diam / 2.) ** 4. - (inner_diam / 2.) ** 4.) ** (1. / 4.)

    num_elem_tip = 3
    num_elem_body = num - 2
    # print(inner_diam)
    # print(outer_diam)
    # print(young_modulus_body)

    #0.017
    length_body = num_elem_body * length_density[0]
    length_tip = (num_elem_tip - 1) * length_density[0]

    topoLines_guide = root_node.addChild(name + '_topo_lines')
    topoLines_guide.addObject(
        'WireRestShape',
        name='InstrRestShape',
        straightLength=length_body,
        length=length_body + length_tip,
        numEdges=nume_nodes_viz,
        youngModulus=young_modulus_body,
        spireDiameter=250.0,
        numEdgesCollis=[num_elem_body, num_elem_tip],
        printLog=True,
        template='Rigid3d',
        spireHeight=0.0,
        radius=outer_diam_qu / 2.0,
        radiusExtremity=outer_diam_qu / 2.0,
        densityOfBeams=[num_elem_body, num_elem_tip],
        youngModulusExtremity=young_modulus_tip)
    topoLines_guide.addObject(
        'EdgeSetTopologyContainer',
        name='meshLinesGuide')
    topoLines_guide.addObject(
        'EdgeSetTopologyModifier',
        name='Modifier')
    topoLines_guide.addObject(
        'EdgeSetGeometryAlgorithms',
        name='GeomAlgo',
        template='Rigid3d')
    topoLines_guide.addObject(
        'MechanicalObject',
        name='dofTopo2',
        template='Rigid3d')

    InstrumentCombined = root_node.addChild(name)
    InstrumentCombined.addObject(
        'EulerImplicit',
        rayleighStiffness=0.2,
        printLog=False,
        rayleighMass=0.0)
    InstrumentCombined.addObject(
        'BTDLinearSolver',
        verification=False,
        subpartSolve=False, verbose=False)
    RG = InstrumentCombined.addObject(
        'RegularGrid',
        name='meshLinesCombined',
        zmax=1, zmin=1,
        nx=num_elem_body + num_elem_tip, ny=1, nz=1,
        xmax=0.2, xmin=0, ymin=0, ymax=0)
    MO = InstrumentCombined.addObject(
        'MechanicalObject',
        showIndices="0",
        showIndicesScale=0.01,
        name='DOFs',
        template='Rigid3d')

    InstrumentCombined.addObject('Monitor', template="Rigid3", indices=num, showTrajectories=info[5][0]['tra'], TrajectoriesPrecision="0.1", TrajectoriesColor="1 1 0 1")

    MO.init()
    restPos = []
    indicesAll = []
    i = 0
    for pos in MO.rest_position.value:
        restPos.append(T_start_sim)
        indicesAll.append(i)
        i = i + 1

    forcesList = ""
    for i in range(0, num_elem_body + num_elem_tip):
        forcesList += " 0 0 0 0 0 0 "

    indicesList = list(range(0, num_elem_body + num_elem_tip))

    MO.rest_position.value = restPos

    IC = InstrumentCombined.addObject(
        'WireBeamInterpolation',
        WireRestShape='@../' + name + '_topo_lines' + '/InstrRestShape',
        radius=outer_diam_qu / 2.0,
        printLog=True,
        name='InterpolGuide')
    InstrumentCombined.addObject(
        'AdaptiveBeamForceFieldAndMass',
        massDensity=200.0,
        name='GuideForceField',
        interpolation='@InterpolGuide')

    # CFF = InstrumentCombined.addObject(
    #     'ConstantForceField',
    #     indices=indicesList,
    #     forces=forcesList,
    #     indexFromEnd=True,
    #     showArrowSize=0)
    CFF = InstrumentCombined.addObject(
        'ConstantForceField',
        indices=indicesList,
        forces=forcesList,
        indexFromEnd=True,
        showArrowSize=0)#110#10
    CFF_visu = InstrumentCombined.addObject(
        'ConstantForceField',
        indices=indicesList,
        forces=forcesList,
        indexFromEnd=True,
        showArrowSize=info[5][0]["force_scale"][0],
        name='ceshi')
    CFF_visu_2 = InstrumentCombined.addObject(
        'ConstantForceField',
        indices=indicesList,
        forces=forcesList,
        indexFromEnd=True,
        showArrowSize=info[5][0]["force_scale"][1],
        name='ceshi_2')
    mec = root_node.addObject('MechanicalObject', name="mec", template="Vec3",
                              showObject=0,
                              showObjectScale=0.0)

    # root_node.addObject('OglLabel', label='@mec.position', fontsize="15",
    #                     color="contrast", prefix="angle: ", updateLabelEveryNbSteps="1",
    #                     y="75")

    IRC = InstrumentCombined.addObject(
        'InterventionalRadiologyController',
        xtip=[0.001], name='m_ircontroller',
        instruments='InterpolGuide',
        # step=0.000175,
        step=length_density[1] * root_node.dt.value,
        printLog=True,
        listening=True,
        template='Rigid3d',
        startingPos=T_start_sim,
        rotationInstrument=[0.],
        speed=1e-12,
        mainDirection=[0, 0, 1],
        threshold=5e-9,
        controlledInstrument=0)
    InstrumentCombined.addObject(
        'LinearSolverConstraintCorrection',
        wire_optimization='true',
        printLog=False)
    InstrumentCombined.addObject(
        'FixedConstraint',
        indices=0,
        name='FixedConstraint')
    InstrumentCombined.addObject(
        'RestShapeSpringsForceField',
        points='@m_ircontroller.indexFirstNode',
        angularStiffness=1e8,
        stiffness=1e8)

    # restrict DOF of nodes
    InstrumentCombined.addObject(
        'PartialFixedConstraint',
        indices=indicesAll,
        fixedDirections=fixed_directions,
        fixAll=True)

    # Collision model
    Collis = InstrumentCombined.addChild(name + '_collis')
    Collis.activated = True
    Collis.addObject(
        'EdgeSetTopologyContainer',
        name='collisEdgeSet')
    Collis.addObject(
        'EdgeSetTopologyModifier',
        name='colliseEdgeModifier')
    Collis.addObject(
        'MechanicalObject',
        name='CollisionDOFs')
    Collis.addObject(
        'MultiAdaptiveBeamMapping',
        controller='../m_ircontroller',
        useCurvAbs=True, printLog=False,
        name='collisMap')
    Collis.addObject(
        'LineCollisionModel',
        proximity=0.0,
        group=1)
    Collis.addObject(
        'PointCollisionModel',
        proximity=0.0,
        group=1)

    # VISU ROS
    CathVisuROS = InstrumentCombined.addChild(
        'CathVisuROS')
    CathVisuROS.addObject(
        'RegularGrid',
        name='meshLinesCombined',
        zmax=0., zmin=0., nx=nume_nodes_viz, ny=1, nz=1,
        xmax=1., xmin=0.0, ymin=0.0, ymax=0.0)
    MO_visu = CathVisuROS.addObject(
        'MechanicalObject',
        name='ROSCatheterVisu',
        template='Rigid3d')
    CathVisuROS.addObject(
        'AdaptiveBeamMapping',
        interpolation='@../InterpolGuide',
        printLog='1',
        useCurvAbs='1')

    # visualization sofa
    CathVisu = InstrumentCombined.addChild(name + '_viz')
    CathVisu.addObject('MechanicalObject', name='QuadsCatheter')
    CathVisu.addObject('QuadSetTopologyContainer', name='ContainerCath')
    CathVisu.addObject('QuadSetTopologyModifier', name='Modifier')
    CathVisu.addObject(
        'QuadSetGeometryAlgorithms',
        name='GeomAlgo',
        template='Vec3d')
    CathVisu.addObject(
        'Edge2QuadTopologicalMapping',
        flipNormals='true',
        input='@../../' + name + '_topo_lines' + '/meshLinesGuide',
        nbPointsOnEachCircle='10',
        output='@ContainerCath',
        # radius=0.0015 / 2,
        radius=0.003 / 2,
        # radius=0.002 / 2,
        tags='catheter')
    CathVisu.addObject(
        'AdaptiveBeamMapping',
        interpolation='@../InterpolTube0',
        input='@../DOFs',
        isMechanical='false',
        name='VisuMapCath',
        output='@QuadsCatheter',
        printLog='1',
        useCurvAbs='1')
    VisuOgl = CathVisu.addChild('VisuOgl')
    VisuOgl.addObject(
        'OglModel',
        quads='@../ContainerCath.quads',
        color=color,
        material='texture Ambient 1 0.2 0.2 0.2 0.0 Diffuse 1 1.0 1.0 1.0 1.0 Specular 1 1.0 1.0 1.0 1.0 Emissive 0 0.15 0.05 0.05 0.0 Shininess 1 20',
        name='VisualCatheter')
    VisuOgl.addObject(
        'IdentityMapping',
        input='@../QuadsCatheter',
        output='@VisualCatheter',
        name='VisuCathIM')

    ceshi = root_node.addObject('MechanicalObject', name="body", template="Rigid3", showObject=1,
                                position=T_start_sim, showObjectScale=0.01)
    return MO.position[num], CFF, MO, CFF_visu, CFF_visu_2, mec, IRC


def Create_Magnetic_sphere(root_node, pose, totalMass, volume, inertiaMatrix, info):
    # pose = [0.12, 0, 0.02, 0, 0, 0.7071067811865475, 0.7071067811865475]
    capsule = root_node.addChild("magnetic_sphere")

    pose_capsule = capsule.addObject('MechanicalObject', name="mstate", template="Rigid3",
                                     position=pose, showObject=True, showObjectScale=0.0)

    capsule.addObject('UniformMass', name="mass",
                      vertexMass=[totalMass, volume, inertiaMatrix[:]])
    # capsule.addObject('UniformMass', name="mass", totalMass=totalMass)
    # self.mass = self.addObject('UniformMass', totalMass=self.totalMass.value, name='mass')
    capsule.addObject('UncoupledConstraintCorrection')
    capsule.addObject('EulerImplicitSolver', name='odesolver')
    # capsule.addObject('CGLinearSolver', name='Solver')
    capsule.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')

    capsule.addObject('Monitor', template="Rigid3", indices=0, showTrajectories=info[5][0]['tra'], TrajectoriesPrecision="0.1", TrajectoriesColor="0.216 0.27 0.796 1", sizeFactor=50.0)

    # capsule.addObject('ShewchukPCGLinearSolver', iterations = '1000', tolerance = '1e-9', preconditioners = 'LUSolver', build_precond = '1', update_step = '1000')

    force_torque_capsule = capsule.addObject('ConstantForceField', name='ss', indices=[0],
                                             forces=[[0, 0, 0, 0, 0, 0]],
                                             indexFromEnd=True, showArrowSize="0")
    CFF_visu = capsule.addObject('ConstantForceField', indices=[0], name='ceshi',
                                                forces=[[0, 0, 0, 0, 0, 0]],
                                                indexFromEnd=True, showArrowSize=info[5][0]["force_scale"][0])
    CFF_visu_2 = capsule.addObject('ConstantForceField', indices=[0], name='ceshi_2',
                                                forces=[[0, 0, 0, 0, 0, 0]],
                                                indexFromEnd=True, showArrowSize=info[5][0]["force_scale"][1])
    # 0.00153 * 0.050
    # sphere.addObject('OglLabel', label='@mstate.force', fontsize="15", color="contrast",
    #                       prefix="force and torque: ", updateLabelEveryNbSteps="20", y="150")
    # sphere.addObject('OglLabel', label='@mstate.velocity', fontsize="15", color="contrast",
    #                       prefix="velocity: ", updateLabelEveryNbSteps="20", y="250")

    # sphereVisu后续不会用到，保存为函数的局部变量
    capsule_visu1 = capsule.addChild("sphere_visu1")
    capsule_visu1.loader = capsule_visu1.addObject('MeshSTLLoader', name="loader_visu1",
                                                   filename="model/magnetic instrument/NS.stl",
                                                   scale='0.0008', translation=[0.000, 0, 0], rotation=[0, 0, 0])
    capsule_visu1.addObject('OglModel', name="model_visu1", src="@loader_visu1", color=[1., 0., 0., 1],
                            updateNormals=False)
    capsule_visu1.addObject('RigidMapping')

    capsule_visu2 = capsule.addChild("sphere_visu1")
    capsule_visu2.loader = capsule_visu2.addObject('MeshSTLLoader', name="loader_visu2",
                                                   filename="model/magnetic instrument/NS.stl",
                                                   scale='0.0008', translation=[-0.0028, 0, 0], rotation=[0, 0, 0])
    capsule_visu2.addObject('OglModel', name="model_visu2", src="@loader_visu2", color=[0.2824, 0.46275, 1., 1],
                            updateNormals=False)
    capsule_visu2.addObject('RigidMapping')

    # capsule_visu3 = capsule.addChild("sphere_visu3")
    # capsule_visu3.loader = capsule_visu3.addObject('MeshSTLLoader', name="loader_visu3",
    #                                                filename="model/magnetic instrument/jiaonang_new.stl",
    #                                                scale='0.0007', translation=[0, 0, 0], rotation=[0, 0, 90])
    # capsule_visu3.addObject('OglModel', name="model_visu3", src="@loader_visu3", color=[0, 1, 0, 0.3],
    #                         updateNormals=False)
    # capsule_visu3.addObject('RigidMapping')


    capsule_visu3 = capsule.addChild("sphere_visu3")
    capsule_visu3.loader = capsule_visu3.addObject('MeshSTLLoader', name="loader_visu3",
                                                   filename="model/magnetic instrument/jiaonang_waike.stl",
                                                   scale='0.0007', translation=[0, 0, 0], rotation=[0, 0, 0])
    capsule_visu3.addObject('OglModel', name="model_visu3", src="@loader_visu3", color=[0.21176, 0.21176, 0.21176, 0.15],
                            updateNormals=False)
    capsule_visu3.addObject('RigidMapping')

    capsule_visu4 = capsule.addChild("sphere_visu4")
    capsule_visu4.loader = capsule_visu4.addObject('MeshSTLLoader', name="loader_visu4",
                                                   filename="model/magnetic instrument/jiaonang_waike2.stl",
                                                   scale='0.0007', translation=[0, 0, 0], rotation=[0, 0, 0])
    capsule_visu4.addObject('OglModel', name="model_visu4", src="@loader_visu4", color=[0.21176, 0.21176, 0.21176, 0.15],
                            updateNormals=False)
    capsule_visu4.addObject('RigidMapping')

    capsule_visu5 = capsule.addChild("sphere_visu5")
    capsule_visu5.loader = capsule_visu5.addObject('MeshSTLLoader', name="loader_visu5",
                                                   filename="model/magnetic instrument/jiaonang_peitu1.stl",
                                                   scale='0.0007', translation=[0, 0, 0], rotation=[0, 0, 0])
    capsule_visu5.addObject('OglModel', name="model_visu5", src="@loader_visu5", color=[0.7098, 0.7098, 0.7098, 1],
                            updateNormals=False)
    capsule_visu5.addObject('RigidMapping')

    capsule_visu6 = capsule.addChild("sphere_visu6")
    capsule_visu6.loader = capsule_visu6.addObject('MeshSTLLoader', name="loader_visu6",
                                                   filename="model/magnetic instrument/jiaonang_cam1.stl",
                                                   scale='0.0007', translation=[-0.006, 0, 0.0], rotation=[0, 0, 180])
    capsule_visu6.addObject('OglModel', name="model_visu6", src="@loader_visu6", color=[0.5098, 0.5098, 0.5098, 1],
                            updateNormals=False)
    capsule_visu6.addObject('RigidMapping')

    # Collision Object for the Cube
    collision = capsule.addChild("CubeCollisionModel1")
    collision.addObject('MeshSTLLoader', name="loader", filename="model/magnetic instrument/jiaonang_new.stl",
                        triangulate=True, scale=0.0005, translation=[0, 0, 0], rotation=[0, 0, 90])

    collision.addObject('MeshTopology', src="@loader")
    collision.addObject('MechanicalObject')

    collision.addObject('TriangleCollisionModel')
    collision.addObject('LineCollisionModel')
    collision.addObject('PointCollisionModel')

    collision.addObject('RigidMapping')

    return pose_capsule.position[0], pose_capsule, pose_capsule.velocity[0], force_torque_capsule, CFF_visu, CFF_visu_2


def Create_Magnetic_sphere_dingwei(root_node, pose, info):
    # pose = [0.12, 0, 0.02, 0, 0, 0.7071067811865475, 0.7071067811865475]
    capsule = root_node.addChild("magnetic_sphere")

    pose_capsule = capsule.addObject('MechanicalObject', name="mstate", template="Rigid3",
                                     position=pose, showObject=True, showObjectScale=0.01)

    if 'tra' in info[5][0]:
        capsule.addObject('Monitor', template="Rigid3", indices=0, showTrajectories=1, TrajectoriesPrecision="0.1", TrajectoriesColor="1 1 0 1")

    # capsule.addObject('UniformMass', name="mass",
    #                   vertexMass=[totalMass, volume, inertiaMatrix[:]])
    # capsule.addObject('UniformMass', name="mass")
    # # self.mass = self.addObject('UniformMass', totalMass=self.totalMass.value, name='mass')
    # capsule.addObject('UncoupledConstraintCorrection')
    # capsule.addObject('EulerImplicitSolver', name='odesolver')
    # # capsule.addObject('CGLinearSolver', name='Solver')
    # capsule.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')

    return pose_capsule.position[0]


def Create_Electromagnet(root_node, electromagnet_pose, radius, turns):
    electromagnet_visu_list = list()
    electromagnet_visu_list_2 = list()

    p_array = np.zeros((3, len(electromagnet_pose)))
    m_hat_array = np.zeros((3, len(electromagnet_pose)))
    coef = np.zeros((len(electromagnet_pose), 1))

    # # 八个电磁铁的构型，要用别的构型需要修改这个文件
    # electromagnet_info = [{"translation": [-0.17, 0, 0], "rotation": [0, 0, 0]},
    #                       {"translation": [0.17, 0, 0], "rotation": [-5.31301024e+01, -5.66192218e-15, 1.80000000e+02]},
    #                       {"translation": [0, -0.17, 0],
    #                        "rotation": [-3.50835465e-15, -3.50835465e-15, 9.00000000e+01]},
    #                       {"translation": [0, 0.17, 0], "rotation": [3.50835465e-15, -3.50835465e-15, -9.00000000e+01]},
    #                       {"translation": [0.1512, 0.1512335, 0.1334], "rotation": [-61.9739879, 27.93835273, -135]},
    #                       {"translation": [-0.1512335, 0.1512335, 0.13342525],
    #                        "rotation": [-11.7658074, 27.93835273, -45]},
    #                       {"translation": [-0.1512335, -0.1512335, 0.13342525],
    #                        "rotation": [11.7658074, 27.93835273, 45]},
    #                       {"translation": [0.1512, -0.1512335, 0.1334], "rotation": [61.9739879, 27.93835273, 135]}]
    #目前默认八个线圈的半径匝数相同
    # coef = np.array([[0.7854], [0.7854], [0.7854], [0.7854], [0.7854], [0.7854], [0.7854], [0.7854]]) * ((3.14*radius**2) * turns)
    #
    # num = 8
    # for i in range(8):
    #     p_array[0:3, i:i + 1] = np.array([electromagnet_info[i]["translation"]]).transpose() + np.array(
    #         [[electromagnet_pose[0]], [electromagnet_pose[1]], [electromagnet_pose[2]]])
    #     m_hat_array[0:3, i:i + 1] = euler_to_rot(electromagnet_info[i]["rotation"])[0:3, 0:1]
    #
    # e = Electromagnet(p_array, m_hat_array, coef, num)
    # electromagnet = root_node.addChild('electromagnet')
    # electromagnet.addObject('MechanicalObject', name="electromagnet", template="Rigid3",
    #                         position=electromagnet_pose, showObject=1, showObjectScale=0.0)
    # for i in range(8):
    #     electromagnet_visu_list.append(electromagnet.addChild("electromagnet_" + str(i) + "_visu"))
    #     electromagnet_visu_list[i].addObject('MeshSTLLoader', name="loader" + str(i),
    #                                          filename="model/magnetic source/electromagnet.stl",
    #                                          translation=electromagnet_info[i]["translation"],
    #                                          rotation=electromagnet_info[i]["rotation"], scale='0.0005')
    #     electromagnet_visu_list[i].addObject('OglModel', name="model" + str(i), src="@loader" + str(i),
    #                                          color=[0.498039, 0.376470588, 0, 1], updateNormals=False)
    #     electromagnet_visu_list[i].addObject('RigidMapping')

    num = len(electromagnet_pose)

    for i in range(num):
        coef[i, 0] = 3.14 * radius[i]**2 * turns[i]

    # print(coef)
    for i in range(num):
        p_array[0:3, i:i + 1] = np.array([[electromagnet_pose[i][0]], [electromagnet_pose[i][1]], [electromagnet_pose[i][2]]])
        m_hat_array[0:3, i:i + 1] = np.array([[electromagnet_pose[i][3]], [electromagnet_pose[i][4]], [electromagnet_pose[i][5]]])

    e = Electromagnet(p_array, m_hat_array, coef, num)
    electromagnet = root_node.addChild('electromagnet')
    electromagnet.addObject('MechanicalObject', name="electromagnet", template="Rigid3",
                            position=[0, 0, 0, 0, 0, 0, 1], showObject=1, showObjectScale=0.0)
    for i in range(num):
        #单位向量转欧拉角ZYX
        rotation = rot_to_euler(moment_to_rmatrix(m_hat_array[0:3, i:i + 1]))

        electromagnet_visu_list.append(electromagnet.addChild("electromagnet_" + str(i) + "_visu"))
        electromagnet_visu_list[i].addObject('MeshSTLLoader', name="loader" + str(i),
                                             filename="model/magnetic source/coin.stl",
                                             translation=electromagnet_pose[i][0:3],
                                             rotation=rotation, scale='0.0006')
        electromagnet_visu_list[i].addObject('OglModel', name="model" + str(i), src="@loader" + str(i),
                                             color=[0.5451, 0.27059, 0.07451, 0.5], updateNormals=False)
        electromagnet_visu_list[i].addObject('RigidMapping')

    for i in range(num):
        #单位向量转欧拉角ZYX
        rotation = rot_to_euler(moment_to_rmatrix(m_hat_array[0:3, i:i + 1]))

        electromagnet_visu_list_2.append(electromagnet.addChild("electromagnet_" + str(i) + "_visu2"))
        electromagnet_visu_list_2[i].addObject('MeshSTLLoader', name="loader2" + str(i),
                                             filename="model/magnetic source/elc_coat.stl",
                                             translation=electromagnet_pose[i][0:3],
                                             rotation=rotation, scale='0.0006')
        electromagnet_visu_list_2[i].addObject('OglModel', name="model2" + str(i), src="@loader2" + str(i),
                                             color=[0.7451, 0.7451, 0.7451, 1], updateNormals=False)
        electromagnet_visu_list_2[i].addObject('RigidMapping')

    return e


def Create_Helmholtz_Maxwell(root_node, pose, radius, turns):
    # pose只用来可视化，我们默认胶囊的位置在线圈的中心，所以不需要考虑胶囊和线圈的相对位置

    # radius_Helmholtz = np.array([[0.09, 0.12, 0.16]])
    # n_Helmholtz = np.array([[250, 400, 400]])
    radius_Helmholtz = np.array([[radius[0], radius[1], radius[2]]])
    n_Helmholtz = np.array([[turns[0], turns[1], turns[2]]])
    h = Helmholtz(radius_Helmholtz, n_Helmholtz)

    # radius_Maxwell = np.array([[0.2, 0.24, 0.16]])
    # n_Maxwell = np.array([[600, 600, 600]])
    radius_Maxwell = np.array([[radius[3], radius[4], radius[5]]])
    n_Maxwell = np.array([[turns[3], turns[4], turns[5]]])
    # 中 大 小
    euler = np.array([[0, 0, 0], [45, 45, 90], [0, 90, 0]])
    m = Maxwell(radius_Maxwell, n_Maxwell, euler)

    # 后面修改下面的可视化信息表就行
    # 六个亥姆霍兹线圈的构型
    helmholtz_info = [
        {"translation": [0, 0, 0], "rotation": [0, 0, 0], "filename": "model/magnetic source/Small_Coil.stl"},
        {"translation": [0, 0, 0], "rotation": [0, 0, 180], "filename": "model/magnetic source/Small_Coil.stl"},
        {"translation": [0, 0, 0], "rotation": [0, 0, 90], "filename": "model/magnetic source/Middle_Coil.stl"},
        {"translation": [0, 0, 0], "rotation": [0, 0, -90], "filename": "model/magnetic source/Middle_Coil.stl"},
        {"translation": [0, 0, 0], "rotation": [0, 90, 0], "filename": "model/magnetic source/Big_Coil.stl"},
        {"translation": [0, 0, 0], "rotation": [0, -90, 0], "filename": "model/magnetic source/Big_Coil.stl"},
        {"translation": [0, 0, -0.050], "rotation": [180, 0, 90], "filename": "model/environment/tank.stl"}]

    helmholtz_visu_list = list()
    # 可视化部分
    helmholtz = root_node.addChild('Helmholtz')
    helmholtz.addObject('MechanicalObject', name="Helmholtz", template="Rigid3",
                        position=pose, showObject=1, showObjectScale=0.0)
    for i in range(6):
        helmholtz_visu_list.append(helmholtz.addChild("Helmholtz_" + str(i) + "_visu"))
        helmholtz_visu_list[i].addObject('MeshSTLLoader', name="loader" + str(i),
                                         filename=helmholtz_info[i]["filename"],
                                         translation=helmholtz_info[i]["translation"],
                                         rotation=helmholtz_info[i]["rotation"], scale='0.001')
        helmholtz_visu_list[i].addObject('OglModel', name="model" + str(i), src="@loader" + str(i),
                                         color=[0.6, 0.6, 0.6, 1], updateNormals=False)
        helmholtz_visu_list[i].addObject('RigidMapping')

    helmholtz_visu_list.append(helmholtz.addChild("Helmholtz_tank_visu"))
    helmholtz_visu_list[6].addObject('MeshSTLLoader', name="loader" + str(6),
                                     filename=helmholtz_info[6]["filename"],
                                     translation=helmholtz_info[6]["translation"],
                                     rotation=helmholtz_info[6]["rotation"], scale='0.001')
    helmholtz_visu_list[6].addObject('OglModel', name="model" + str(6), src="@loader" + str(6),
                                     color=[0.6, 0.6, 0.6, 0.3], updateNormals=False)
    helmholtz_visu_list[6].addObject('RigidMapping')

    euler_rotation = np.array([0, 0, 0])
    for i in range(3):
        m1 = euler2_to_rot((0, 180, 0)).dot(euler2_to_rot(euler[i]))
        Rq = rmatrix_to_quat(m1)
        r = R.from_quat(Rq)
        euler_1 = r.as_euler('zyx', degrees=True)
        temp = euler_1[0]
        euler_1[0] = euler_1[2]
        euler_1[2] = temp
        euler_rotation = np.vstack((euler_rotation, euler_1))
    euler_rotation = euler_rotation[1:]

    print("vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
    print(euler)
    print(euler_rotation)
    print("vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
    # 六个麦克斯韦线圈的构型
    maxwell_info = [
        {"translation": [0, 0, 0], "rotation": euler[0], "filename": "model/magnetic source/Big_Coil_Maxwell2.stl"},
        {"translation": [0, 0, 0], "rotation": euler_rotation[0],
         "filename": "model/magnetic source/Big_Coil_Maxwell2.stl"},
        {"translation": [0, 0, 0], "rotation": euler[1], "filename": "model/magnetic source/Big_Coil_Maxwell3.stl"},
        {"translation": [0, 0, 0], "rotation": euler_rotation[1],
         "filename": "model/magnetic source/Big_Coil_Maxwell3.stl"},
        {"translation": [0, 0, 0], "rotation": euler[2], "filename": "model/magnetic source/Big_Coil_Maxwell.stl"},
        {"translation": [0, 0, 0], "rotation": euler_rotation[2],
         "filename": "model/magnetic source/Big_Coil_Maxwell.stl"}]

    maxwell_visu_list = list()
    # 可视化部分
    maxwell = root_node.addChild('Maxwell')
    maxwell.addObject('MechanicalObject', name="Maxwell", template="Rigid3",
                      position=pose, showObject=1, showObjectScale=0.0)
    for i in range(6):
        maxwell_visu_list.append(maxwell.addChild("Maxwell_" + str(i) + "_visu"))
        maxwell_visu_list[i].addObject('MeshSTLLoader', name="loader" + str(i),
                                       filename=maxwell_info[i]["filename"],
                                       translation=maxwell_info[i]["translation"],
                                       rotation=maxwell_info[i]["rotation"], scale='0.001')
        maxwell_visu_list[i].addObject('OglModel', name="model" + str(i), src="@loader" + str(i),
                                       color=[0.7490196078431373, 0.5647058823529412, 0.0, 1], updateNormals=False)
        maxwell_visu_list[i].addObject('RigidMapping')

    return h, m


def Create_Mag_Still_Visual(root_node, pose):
    magnetic_still = root_node.addChild("magnetic_still_Visual")

    pose_magnetic_still = magnetic_still.addObject('MechanicalObject', name="mstate", template="Rigid3",
                                     position=pose, showObject=True, showObjectScale=0.0)

    capsule_visu1 = magnetic_still.addChild("magnetic_still_visu1")
    capsule_visu1.loader = capsule_visu1.addObject('MeshSTLLoader', name="loader_visu1",
                                                   filename="model/magnetic instrument/NS.stl",
                                                   scale='0.0008', translation=[-0.000, 0, 0], rotation=[0, 0, 0])
    capsule_visu1.addObject('OglModel', name="model_visu1", src="@loader_visu1", color=[1., 0., 0.],
                            updateNormals=False)
    capsule_visu1.addObject('RigidMapping')

    capsule_visu2 = magnetic_still.addChild("magnetic_still_visu1")
    capsule_visu2.loader = capsule_visu2.addObject('MeshSTLLoader', name="loader_visu2",
                                                   filename="model/magnetic instrument/NS.stl",
                                                   scale='0.0008', translation=[-0.004, 0, 0], rotation=[0, 0, 0])
    capsule_visu2.addObject('OglModel', name="model_visu2", src="@loader_visu2", color=[0.2824, 0.46275, 1.],
                            updateNormals=False)
    capsule_visu2.addObject('RigidMapping')

    return pose_magnetic_still.position[0]


def Create_Mag_Still_Active(root_node, pose, totalMass, volume, inertiaMatrix):
    magnetic_still = root_node.addChild("magnetic_still_Active")

    pose_magnetic_still = magnetic_still.addObject('MechanicalObject', name="mstate", template="Rigid3",
                                     position=pose, showObject=True, showObjectScale=0.0)
    magnetic_still.addObject('UniformMass', name="mass",
                      vertexMass=[totalMass, volume, inertiaMatrix[:]])
    magnetic_still.addObject('UncoupledConstraintCorrection')
    magnetic_still.addObject('EulerImplicitSolver', name='odesolver')
    # capsule.addObject('CGLinearSolver', name='Solver')
    magnetic_still.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')

    force_torque_capsule = magnetic_still.addObject('ConstantForceField', name='ss', indices=[0],
                                             forces=[[0, 0, 0, 0, 0, 0]],
                                             indexFromEnd=True, showArrowSize="1")

    # sphereVisu后续不会用到，保存为函数的局部变量
    capsule_visu1 = magnetic_still.addChild("magnetic_still_visu1")
    capsule_visu1.loader = capsule_visu1.addObject('MeshSTLLoader', name="loader_visu1",
                                                   filename="model/magnetic instrument/NS.stl",
                                                   scale='0.0008', translation=[-0.000, 0, 0], rotation=[0, 0, 0])
    capsule_visu1.addObject('OglModel', name="model_visu1", src="@loader_visu1", color=[1., 0., 0.],
                            updateNormals=False)
    capsule_visu1.addObject('RigidMapping')

    capsule_visu2 = magnetic_still.addChild("magnetic_still_visu1")
    capsule_visu2.loader = capsule_visu2.addObject('MeshSTLLoader', name="loader_visu2",
                                                   filename="model/magnetic instrument/NS.stl",
                                                   scale='0.0008', translation=[-0.004, 0, 0], rotation=[0, 0, 0])
    capsule_visu2.addObject('OglModel', name="model_visu2", src="@loader_visu2", color=[0.2824, 0.46275, 1.],
                            updateNormals=False)
    capsule_visu2.addObject('RigidMapping')

    # Collision Object for the Cube
    # collision = capsule.addChild("CubeCollisionModel1")
    # collision.addObject('MeshSTLLoader', name="loader", filename="model/magnetic instrument/jiaonang.stl",
    #                     triangulate=True, scale=0.0008)
    #
    # collision.addObject('MeshTopology', src="@loader")
    # collision.addObject('MechanicalObject')
    #
    # collision.addObject('TriangleCollisionModel')
    # collision.addObject('LineCollisionModel')
    # collision.addObject('PointCollisionModel')
    #
    # collision.addObject('RigidMapping')

    return pose_magnetic_still.position[0], force_torque_capsule


def Create_Manipulator(root_node, base_pose, magnet_pose, visual_type, X0=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
    link_list = list()
    link_pose_list = list()
    link_visu_list = list()

    # 用户设定的跟机械臂型号有关的参数
    a = np.array([[0], [-0.42500], [-0.39225], [0], [0], [0]])
    d = np.array([[0.089159], [0], [0], [0.10915], [0.09465], [0.08230]])
    alpha = np.array([[np.pi / 2], [0], [0], [np.pi / 2], [-np.pi / 2], [0]])
    T_bias = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.08089], [0, 0, 0, 1]])
    # T_bias = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.085], [0, 0, 0, 1]])


    link_info = [
        {"filename": "model/magnetic source/Base.stl", "translation": [-0., -0., 0.003], "rotation": [0, 90, 0],
         "color": [0.7451, 0.7451, 0.7451, 1]},
        {"filename": "model/magnetic source/J1.stl", "translation": [0, -0.0845, 0], "rotation": [90, 0, -90],
         "color": [0.7451, 0.7451, 0.7451, 1]},
        {"filename": "model/magnetic source/J2.stl", "translation": [-0.086, 0, 0.007], "rotation": [90, 0, 180],
         "color": [0.7451, 0.7451, 0.7451, 1]},
        {"filename": "model/magnetic source/J3.stl", "translation": [-0.034, -0.05, -0.062], "rotation": [90, 0, 0],
         "color": [0.7451, 0.7451, 0.7451, 1]},
        {"filename": "model/magnetic source/j4_b.stl", "translation": [0.002, 0, 0.002], "rotation": [0, -90, 0],
         "color": [0.7451, 0.7451, 0.7451, 1]},
        {"filename": "model/magnetic source/J5.stl", "translation": [0, 0.9982, -0.11], "rotation": [90, 0, 90],
         "color": [0.7451, 0.7451, 0.7451, 1]},
        {"filename": "model/magnetic source/J6.stl", "translation": [0, 0., -0.035], "rotation": [90, -0, 0],
         "color": [0.7451, 0.7451, 0.7451, 1]},
        {"filename": "model/magnetic source/support2.stl", "translation": [0, 0, -0.081], "rotation": [90, 0, -45],
         "color": [0.32157, 0.5451, 0.5451, 0.5]},
        {"filename": "model/magnetic source/NN.stl", "translation": [0, 0, 0], "rotation": [0, 0, 0],
         "color": [1, 0, 0, 1]},
        {"filename": "model/magnetic source/SS.stl", "translation": [0, 0, 0], "rotation": [0, 0, 0],
         "color": [0.2824, 0.46275, 1, 1]}]

    # 用户设定的机械臂基座的初始位姿
    robot = Manipulator2(base_pose, a, d, alpha, T_bias)

    # 用户设定永磁铁的初始位姿
    position, moment = pose_to_position_moment(magnet_pose)
    # 逆解得到机械臂的关节角度
    theta = robot.back_numerical(position, moment, X0)
    # print(theta)
    # theta = np.array([[-1.753613], [-1.65546861], [-1.5911363], [2.99254862], [-1.44792851], [2.26469802]]) #[-100.52558598726115, -94.89947445859873, -91.21163503184714, 171.5473731210191, -83.00227127388534, 129.82345337579616]
    # 由机械臂的关节角度正向得到各个连杆的位姿
    link_init_pose_list = robot.fkine_all_link(theta)

    # 机械臂底座
    visu_object(root_node, path="model/magnetic source/box.stl",
                pose=[base_pose[0], base_pose[1] + 0, base_pose[2] - 0.725, 0.0, 0.7071067811865475, 0.0,
                      -0.7071067811865475], scale="0.0009",
                color=[0.6509803, 0.6509803, 0.6509803, 1])

    # 初始化机械臂的各个关节对象、返回位姿对象列表、可视化对象列表（可视化多一个）
    for i in range(len(link_init_pose_list)):

        link_list.append(root_node.addChild('robot' + str(i)))
        link_pose_list.append(
            link_list[i].addObject('MechanicalObject', name="robot_" + str(i), template="Rigid3",
                                   position=link_init_pose_list[i], showObject=1,
                                   showObjectScale=0.0))

        if i != (len(link_init_pose_list) - 1):
            link_visu_list.append(link_list[i].addChild("robot_" + str(i) + "_visu"))
            link_visu_list[i].addObject('MeshSTLLoader', name="loader" + str(i),
                                        filename=link_info[i]["filename"],
                                        translation=link_info[i]["translation"],
                                        rotation=link_info[i]["rotation"],
                                        scale='0.001')
            link_visu_list[i].addObject('OglModel', name="model" + str(i), src="@loader" + str(i),
                                        color=link_info[i]["color"],
                                        updateNormals=False)
            link_visu_list[i].addObject('RigidMapping')

        if i == (len(link_init_pose_list) - 1):
            # 可视化部分后面暂时不会用到，保存为函数内部的局部变量
            link_visu_list.append(link_list[i].addChild("robot_" + str(i) + "_visu"))
            link_visu_list[i].addObject('MeshSTLLoader', name="loader" + str(i),
                                        filename=link_info[i]["filename"],
                                        translation=link_info[i]["translation"],
                                        rotation=link_info[i]["rotation"],
                                        scale3d=[0.001] * 3)
            link_visu_list[i].addObject('OglModel', name="model" + str(i), src="@loader" + str(i),
                                        color=link_info[i]["color"],
                                        updateNormals=False)
            link_visu_list[i].addObject('RigidMapping')

            if visual_type == 'mag':
                link_visu_list.append(link_list[i].addChild("robot_" + str(i + 1) + "_visu"))
                link_visu_list[i + 1].addObject('MeshSTLLoader', name="loader" + str(i + 1),
                                                filename=link_info[i + 1]["filename"],
                                                translation=link_info[i + 1]["translation"],
                                                rotation=link_info[i + 1]["rotation"],
                                                scale3d=[0.001] * 3)
                link_visu_list[i + 1].addObject('OglModel', name="model" + str(i + 1), src="@loader" + str(i + 1),
                                                color=link_info[i + 1]["color"],
                                                updateNormals=False)
                link_visu_list[i + 1].addObject('RigidMapping')

                link_visu_list.append(link_list[i].addChild("robot_" + str(i + 2) + "_visu"))
                link_visu_list[i + 2].addObject('MeshSTLLoader', name="loader" + str(i + 2),
                                                filename=link_info[i + 2]["filename"],
                                                translation=link_info[i + 2]["translation"],
                                                rotation=link_info[i + 2]["rotation"],
                                                scale3d=[0.001] * 3)
                link_visu_list[i + 2].addObject('OglModel', name="model" + str(i + 2), src="@loader" + str(i + 2),
                                                color=link_info[i + 2]["color"],
                                                updateNormals=False)
                link_visu_list[i + 2].addObject('RigidMapping')

            if visual_type == 'elc':
                link_visu_list.append(link_list[i].addChild("robot_" + str(i + 1) + "_visu"))
                link_visu_list[i + 1].addObject('MeshSTLLoader', name="loader" + str(i + 1),
                                                filename="model/magnetic source/electromagnet.stl",
                                                translation=[0,0,0],
                                                rotation=[0,0,0],
                                                scale3d=[0.0005] * 3)
                link_visu_list[i + 1].addObject('OglModel', name="model" + str(i + 1), src="@loader" + str(i + 1),
                                                color=link_info[i + 2]["color"],
                                                updateNormals=False)
                link_visu_list[i + 1].addObject('RigidMapping')
    return robot, theta, link_pose_list


def Create_Manipulator_dingwei(root_node, base_pose, magnet_pose, visual_type):
    link_list = list()
    link_pose_list = list()
    link_visu_list = list()

    # 用户设定的跟机械臂型号有关的参数
    a = np.array([[0], [-0.42500], [-0.39225], [0], [0], [0]])
    d = np.array([[0.089159], [0], [0], [0.10915], [0.09465], [0.08230]])
    alpha = np.array([[np.pi / 2], [0], [0], [np.pi / 2], [-np.pi / 2], [0]])
    T_bias = np.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0.06], [0, 0, 0, 1]])
    link_info = [
        {"filename": "model/magnetic source/Base.stl", "translation": [-0., -0., 0.003], "rotation": [0, 90, 0],
         "color": [0.949019, 0.949019, 0.949019, 1]},
        {"filename": "model/magnetic source/J1.stl", "translation": [0, -0.0845, 0], "rotation": [90, 0, -90],
         "color": [0.949019, 0.949019, 0.949019, 1]},
        {"filename": "model/magnetic source/J2.stl", "translation": [-0.086, 0, 0.007], "rotation": [90, 0, 180],
         "color": [0.949019, 0.949019, 0.949019, 1]},
        {"filename": "model/magnetic source/J3.stl", "translation": [-0.034, -0.05, -0.062], "rotation": [90, 0, 0],
         "color": [0.949019, 0.949019, 0.949019, 1]},
        {"filename": "model/magnetic source/j4_b.stl", "translation": [0.002, 0, 0.002], "rotation": [0, -90, 0],
         "color": [0.949019, 0.949019, 0.949019, 1]},
        {"filename": "model/magnetic source/J5.stl", "translation": [0, 0.9982, -0.11], "rotation": [90, 0, 90],
         "color": [0.949019, 0.949019, 0.949019, 1]},
        {"filename": "model/magnetic source/J6.stl", "translation": [0., 0., -0.035], "rotation": [90, 0, 45],
         "color": [0.949019, 0.949019, 0.949019, 1]},
        {"filename": "model/magnetic source/support.stl", "translation": [-0.058, 0, 0], "rotation": [90, 90, 0],
         "color": [0.949019, 0.949019, 0.949019, 0.4]},
        {"filename": "model/magnetic source/NN.stl", "translation": [0.0023, 0., 0], "rotation": [0, -0, 0],
         "color": [1, 0, 0, 1]},
        {"filename": "model/magnetic source/SS.stl", "translation": [0.0023, 0, 0], "rotation": [0, -0, 0],
         "color": [0, 0, 1, 1]}]

    # 用户设定的机械臂基座的初始位姿
    robot = Manipulator2(base_pose, a, d, alpha, T_bias)

    # 用户设定永磁铁的初始位姿
    position, moment = pose_to_position_moment(magnet_pose)

    # 逆解得到机械臂的关节角度
    theta = robot.back_numerical(position, moment, [1, 1, 1, 1, 1, 1])
    theta[5, 0] = 0
    # 由机械臂的关节角度正向得到各个连杆的位姿
    link_init_pose_list = robot.fkine_all_link(theta)

    # 机械臂底座
    visu_object(root_node, path="model/magnetic source/box.stl",
                pose=[base_pose[0], base_pose[1] + 0, base_pose[2] - 0.725, 0.0, 0.7071067811865475, 0.0,
                      -0.7071067811865475], scale="0.0009",
                color=[0.6509803, 0.6509803, 0.6509803, 1])

    # 初始化机械臂的各个关节对象、返回位姿对象列表、可视化对象列表（可视化多一个）
    for i in range(len(link_init_pose_list)):

        link_list.append(root_node.addChild('robot' + str(i)))
        link_pose_list.append(
            link_list[i].addObject('MechanicalObject', name="robot_" + str(i), template="Rigid3",
                                   position=link_init_pose_list[i], showObject=1,
                                   showObjectScale=0.0))

        if i != (len(link_init_pose_list) - 1):
            link_visu_list.append(link_list[i].addChild("robot_" + str(i) + "_visu"))
            link_visu_list[i].addObject('MeshSTLLoader', name="loader" + str(i),
                                        filename=link_info[i]["filename"],
                                        translation=link_info[i]["translation"],
                                        rotation=link_info[i]["rotation"],
                                        scale='0.001')
            link_visu_list[i].addObject('OglModel', name="model" + str(i), src="@loader" + str(i),
                                        color=link_info[i]["color"],
                                        updateNormals=False)
            link_visu_list[i].addObject('RigidMapping')

        if i == (len(link_init_pose_list) - 1):
            # 可视化部分后面暂时不会用到，保存为函数内部的局部变量
            link_visu_list.append(link_list[i].addChild("robot_" + str(i) + "_visu"))
            link_visu_list[i].addObject('MeshSTLLoader', name="loader" + str(i),
                                        filename=link_info[i]["filename"],
                                        translation=link_info[i]["translation"],
                                        rotation=link_info[i]["rotation"],
                                        scale3d=[0.001] * 3)
            link_visu_list[i].addObject('OglModel', name="model" + str(i), src="@loader" + str(i),
                                        color=link_info[i]["color"],
                                        updateNormals=False)
            link_visu_list[i].addObject('RigidMapping')

            if visual_type == 'mag':
                link_visu_list.append(link_list[i].addChild("robot_" + str(i + 1) + "_visu"))
                link_visu_list[i + 1].addObject('MeshSTLLoader', name="loader" + str(i + 1),
                                                filename=link_info[i + 1]["filename"],
                                                translation=link_info[i + 1]["translation"],
                                                rotation=link_info[i + 1]["rotation"],
                                                scale3d=[0.0004] * 3)
                link_visu_list[i + 1].addObject('OglModel', name="model" + str(i + 1), src="@loader" + str(i + 1),
                                                color=link_info[i + 1]["color"],
                                                updateNormals=False)
                link_visu_list[i + 1].addObject('RigidMapping')

                link_visu_list.append(link_list[i].addChild("robot_" + str(i + 2) + "_visu"))
                link_visu_list[i + 2].addObject('MeshSTLLoader', name="loader" + str(i + 2),
                                                filename=link_info[i + 2]["filename"],
                                                translation=link_info[i + 2]["translation"],
                                                rotation=link_info[i + 2]["rotation"],
                                                scale3d=[0.0004] * 3)
                link_visu_list[i + 2].addObject('OglModel', name="model" + str(i + 2), src="@loader" + str(i + 2),
                                                color=link_info[i + 2]["color"],
                                                updateNormals=False)
                link_visu_list[i + 2].addObject('RigidMapping')

            if visual_type == 'elc':
                link_visu_list.append(link_list[i].addChild("robot_" + str(i + 1) + "_visu"))
                link_visu_list[i + 1].addObject('MeshSTLLoader', name="loader" + str(i + 1),
                                                filename="model/magnetic source/electromagnet.stl",
                                                translation=[0,0,0],
                                                rotation=[0,0,0],
                                                scale3d=[0.0005] * 3)
                link_visu_list[i + 1].addObject('OglModel', name="model" + str(i + 1), src="@loader" + str(i + 1),
                                                color=link_info[i + 1]["color"],
                                                updateNormals=False)
                link_visu_list[i + 1].addObject('RigidMapping')

    return robot, theta, link_pose_list


def Create_Sensor(root_node, pose_sensor):
    sensor = root_node.addChild('sensor')
    # sensor.addObject('MechanicalObject', name="sensor", template="Rigid3", position=[0, -0.55, -0.15, 0, 0.707, 0, -0.707], showObject=1, showObjectScale=0.01)
    pose_sensor = sensor.addObject('MechanicalObject', name="sensor", template="Rigid3", position=pose_sensor,
                                   showObject=1, showObjectScale=0.0)
    # sensor.addObject('MechanicalObject', name="sensor", template="Rigid3",
    #                  position=[0, 0, 0, 0, 0, 0, 1], showObject=1, showObjectScale=0.01)

    sensor_visu = sensor.addChild("sensor_visu1")
    sensor_visu.addObject('MeshSTLLoader', name="loader", translation=[-0.0526, -0.026, 0], rotation=[0, 0, 0],
                          filename="./model/sensor/sensor.stl", scale='0.001')
    sensor_visu.addObject('OglModel', name="model", src="@loader", color=[0., 0.5451, 0., 1], updateNormals=False)
    sensor_visu.addObject('RigidMapping')
    sensor_visu.addObject('EulerImplicitSolver', name='odesolver')

    sensor_visu = sensor.addChild("sensor_visu2")
    sensor_visu.addObject('MeshSTLLoader', name="loader", translation=[-0.0526, 0.026, 0], rotation=[0, 0, 0],
                          filename="./model/sensor/sensor.stl", scale='0.001')
    sensor_visu.addObject('OglModel', name="model", src="@loader", color=[0., 0.5451, 0., 1], updateNormals=False)
    sensor_visu.addObject('RigidMapping')
    sensor_visu.addObject('EulerImplicitSolver', name='odesolver')

    sensor_visu = sensor.addChild("sensor_visu3")
    sensor_visu.addObject('MeshSTLLoader', name="loader", translation=[0.0526, -0.026, 0], rotation=[0, 0, 0],
                          filename="./model/sensor/sensor.stl", scale='0.001')
    sensor_visu.addObject('OglModel', name="model", src="@loader", color=[0., 0.5451, 0., 1], updateNormals=False)
    sensor_visu.addObject('RigidMapping')
    sensor_visu.addObject('EulerImplicitSolver', name='odesolver')

    sensor_visu = sensor.addChild("sensor_visu4")
    sensor_visu.addObject('MeshSTLLoader', name="loader", translation=[0.0526, 0.026, 0], rotation=[0, 0, 0],
                          filename="./model/sensor/sensor.stl", scale='0.001')
    sensor_visu.addObject('OglModel', name="model", src="@loader", color=[0., 0.5451, 0., 1], updateNormals=False)
    sensor_visu.addObject('RigidMapping')
    sensor_visu.addObject('EulerImplicitSolver', name='odesolver')

    return pose_sensor


def Create_Sensor2(root_node, pose_sensor):
    sensor = root_node.addChild('sensor')
    # sensor.addObject('MechanicalObject', name="sensor", template="Rigid3", position=[0, -0.55, -0.15, 0, 0.707, 0, -0.707], showObject=1, showObjectScale=0.01)
    pose_sensor = sensor.addObject('MechanicalObject', name="sensor", template="Rigid3", position=pose_sensor,
                                   showObject=1, showObjectScale=0.007)
    # sensor.addObject('MechanicalObject', name="sensor", template="Rigid3",
    #                  position=[0, 0, 0, 0, 0, 0, 1], showObject=1, showObjectScale=0.01)

    sensor_visu = sensor.addChild("sensor_visu1")
    sensor_visu.addObject('MeshSTLLoader', name="loader", translation=[0, 0, 0], rotation=[0, 0, 0],
                          filename="./model/sensor/sensor2.stl", scale='0.0014')
    sensor_visu.addObject('OglModel', name="model", src="@loader", color=[0.4117, 0.7215, 0.1725, 1], updateNormals=False)
    sensor_visu.addObject('RigidMapping')
    sensor_visu.addObject('EulerImplicitSolver', name='odesolver')

    return pose_sensor.position[0]


def Create_Tank(root_node, pose_env):
    tank = root_node.addChild("tank")
    tank.addObject('MechanicalObject', name="tank", template="Rigid3",
                   position=pose_env, showObject=0.0,
                   showObjectScale=0.0)
    tank.addObject('UncoupledConstraintCorrection')
    totalMass = 0.1
    tank.addObject('UniformMass', name="mass", totalMass=totalMass)

    # 水缸框架
    visu0 = tank.addChild("VisualTank0")
    visu0.loader = visu0.addObject('MeshSTLLoader', name="loader_tank0", filename="model/environment/tank1.stl",
                                   translation=[0., 0, 0], rotation=[90, -0, 0], scale='0.001')
    visu0.addObject('OglModel', name="tank0", src="@loader_tank0", color=[0.949019, 0.949019, 0.949019, 1],
                    updateNormals=False)
    visu0.addObject('RigidMapping')

    # Collision Object for the Cube
    collision = tank.addChild("CubeCollisionModel")
    collision.addObject('MeshSTLLoader', name="loader", filename="model/environment/tank1.stl", translation=[0., 0, 0],
                        rotation=[90, -0, 0],
                        triangulate=True, scale=0.001)

    collision.addObject('MeshTopology', src="@loader")
    collision.addObject('MechanicalObject')
    # , moving = False, simulated = False
    # collision.addObject('TriangleCollisionModel', moving = False, simulated = False)
    # collision.addObject('LineCollisionModel', moving = False, simulated = False)
    # collision.addObject('PointCollisionModel', moving = False, simulated = False)
    collision.addObject('TriangleCollisionModel')
    collision.addObject('LineCollisionModel')
    collision.addObject('PointCollisionModel')
    collision.addObject('RigidMapping')

    # # 水缸玻璃1
    # visu1 = tank.addChild("VisualTank1")
    # visu1.loader = visu1.addObject('MeshSTLLoader', name="loader_tank1", filename="model/environment/tank2.stl",
    #                                translation=[0., -0.110, 0.090], rotation=[90, -0, 0], scale='0.001')
    # visu1.addObject('OglModel', name="tank1", src="@loader_tank1", color=[0.9, 0.9, 0.9, 0.2],
    #                 updateNormals=False)
    # visu1.addObject('RigidMapping')
    #
    # # 水缸玻璃2
    # visu2 = tank.addChild("VisualTank2")
    # visu2.loader = visu2.addObject('MeshSTLLoader', name="loader_tank2", filename="model/environment/tank2.stl",
    #                                translation=[0., 0.120, 0.090], rotation=[90, -0, 0], scale='0.001')
    # visu2.addObject('OglModel', name="tank2", src="@loader_tank2", color=[0.9, 0.9, 0.9, 0.2],
    #                 updateNormals=False)
    # visu2.addObject('RigidMapping')
    #
    # # 水缸玻璃3
    # visu3 = tank.addChild("VisualTank3")
    # visu3.loader = visu3.addObject('MeshSTLLoader', name="loader_tank3", filename="model/environment/tank2.stl",
    #                                translation=[0.110, 0, 0.090], rotation=[90, 0, 90], scale='0.001')
    # visu3.addObject('OglModel', name="tank3", src="@loader_tank3", color=[0.9, 0.9, 0.9, 0.2],
    #                 updateNormals=False)
    # visu3.addObject('RigidMapping')
    #
    # # 水缸玻璃4
    # visu4 = tank.addChild("VisualTank4")
    # visu4.loader = visu4.addObject('MeshSTLLoader', name="loader_tank4", filename="model/environment/tank2.stl",
    #                                translation=[-0.120, 0, 0.090], rotation=[90, 0, 90], scale='0.001')
    # visu4.addObject('OglModel', name="tank4", src="@loader_tank4", color=[0.9, 0.9, 0.9, 0.2],
    #                 updateNormals=False)
    # visu4.addObject('RigidMapping')


def Create_Show(root_node):
    current_0 = root_node.addChild("current_0")
    show_pose = current_0.addObject('MechanicalObject', name="current_0", template="Rigid3",
                                    position=[0, 0, 0, 0, 0, 0, 1],
                                    showObject=0)
    current_0.addObject('OglLabel', label='@current_0.name', fontsize="1", color="contrast",
                        prefix="Current: ", updateLabelEveryNbSteps="20", y="150")
    return show_pose


def pose_estimation(root_node, pose):
    pos = root_node.addChild("pose")
    show_pose = pos.addObject('MechanicalObject', name="pose", template="Rigid3", position=pose, showObject=1,
                              showObjectScale=0.01)

    return show_pose
