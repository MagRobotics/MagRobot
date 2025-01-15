import Sofa
import numpy as np


class Monitor(Sofa.Core.Controller):

    def __init__(self, root_node, scene, post, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.root_node = root_node
        self.scene = scene
        self.post = post


        self.monitor_force = self.root_node.addChild("force_show")
        self.MO = self.monitor_force.addObject('MechanicalObject', name="force", template="Rigid3d", position=[0, 0, 0, 0, 0, 0, 1], showObject=True, showObjectScale=0.0)
        self.monitor_force.addObject('UniformMass', name="mass", vertexMass=0.0000000001)
        self.Collision_force = self.monitor_force.addObject('ConstantForceField', indices=0, name='ceshi', forces=[0, 0, 0, 0, 0, 0], indexFromEnd=True, showArrowSize=0)
        self.monitor_force.addObject('FixedConstraint', template="Rigid3d", name="default3", indices=0)
        self.monitor_force.addObject('UncoupledConstraintCorrection')
        self.monitor_force.addObject('EulerImplicitSolver', name='odesolver')
        self.monitor_force.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')
        self.monitor_force.addObject('Monitor', template="Rigid3d", indices=0, showForces=1, ForcesColor="0.4196 0.7176 0.7921 1", sizeFactor=1, showMinThreshold="0.0001")

        force_visu1 = self.monitor_force.addChild("sphere_visu1")
        force_visu1.loader = force_visu1.addObject('MeshSTLLoader', name="loader_visu1",
                                                       filename="model/magnetic instrument/wire_mag.stl",
                                                       scale='0.000', translation=[-0.0040, 0, 0], rotation=[0, 0, 0])
        force_visu1.addObject('OglModel', name="model_visu1", src="@loader_visu1", color=[1., 0., 0., 1],
                                updateNormals=False)
        force_visu1.addObject('RigidMapping')

        force_visu2 = self.monitor_force.addChild("sphere_visu1")
        force_visu2.loader = force_visu2.addObject('MeshSTLLoader', name="loader_visu2",
                                                       filename="model/magnetic instrument/wire_mag.stl",
                                                       scale='0.000', translation=[-0.0080, 0, 0], rotation=[0, 0, 0])
        force_visu2.addObject('OglModel', name="model_visu2", src="@loader_visu2", color=[0.4196, 0.7176, 0.7921, 1],
                                updateNormals=False)
        force_visu2.addObject('RigidMapping')


        self.monitor_force2 = self.root_node.addChild("force_show2")
        self.MO2 = self.monitor_force2.addObject('MechanicalObject', name="force", template="Rigid3d", position=[0, 0, 0, 0, 0, 0, 1], showObject=True, showObjectScale=0.0)
        self.monitor_force2.addObject('UniformMass', name="mass", vertexMass=0.0000000001)
        self.Collision_force2 = self.monitor_force2.addObject('ConstantForceField', indices=0, name='ceshi', forces=[0, 0, 0, 0, 0, 0], indexFromEnd=True, showArrowSize=0)
        self.monitor_force2.addObject('FixedConstraint', template="Rigid3d", name="default3", indices=0)
        self.monitor_force2.addObject('UncoupledConstraintCorrection')
        self.monitor_force2.addObject('EulerImplicitSolver', name='odesolver')
        self.monitor_force2.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')
        self.monitor_force2.addObject('Monitor', template="Rigid3d", indices=0, showForces=1, ForcesColor="0.6549 0.5922 0.8549 1", sizeFactor=1, showMinThreshold="0.0001")


        self.monitor_force3 = self.root_node.addChild("force_show3")
        self.MO3 = self.monitor_force3.addObject('MechanicalObject', name="force", template="Rigid3d", position=[0, 0, 0, 0, 0, 0, 1], showObject=True, showObjectScale=0.0)
        self.monitor_force3.addObject('UniformMass', name="mass", vertexMass=0.0000000001)
        self.Collision_force3 = self.monitor_force3.addObject('ConstantForceField', indices=0, name='ceshi', forces=[0, 0, 0, 0, 0, 0], indexFromEnd=True, showArrowSize=0)
        self.monitor_force3.addObject('FixedConstraint', template="Rigid3d", name="default3", indices=0)
        self.monitor_force3.addObject('UncoupledConstraintCorrection')
        self.monitor_force3.addObject('EulerImplicitSolver', name='odesolver')
        self.monitor_force3.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')
        self.monitor_force3.addObject('Monitor', template="Rigid3d", indices=0, showForces=1, ForcesColor="0.4705 0.5411 0.7137 1", sizeFactor=1, showMinThreshold="0.0001")

    def onAnimateBeginEvent(self, event):

        self.MO.position[0][:] = self.scene.instrument[0]['position_active'][:]
        self.MO2.position[0][:] = self.scene.instrument[0]['position_active'][:]
        self.MO3.position[0][:] = self.scene.instrument[0]['position_active'][:]

        # 场图
        # self.Collision_force.forces[0][0:3] = self.scene.instrument[0]['CFF_visu'].forces[0][0:3] * 3000 * 1.8 * 0.5
        # self.Collision_force2.forces[0][0:3] = self.scene.instrument[0]['force_torque_wire'].forces[0][0:3] * 25 * 2.5
        # self.Collision_force3.forces[0][0:3] = self.scene.instrument[0]['CFF_visu_2'].forces[0][0:3] * 90 * 3000
        # print(self.scene.instrument[0]['CFF_visu_2'].forces[0][0:3])

        # case2图
        self.Collision_force.forces[0][0:3] = self.scene.instrument[0]['CFF_visu'].forces[0][0:3] * 3000 * 1.8 * 5
        self.Collision_force2.forces[0][0:3] = self.scene.instrument[0]['force_torque_wire'].forces[0][0:3] * 1.8 * 5
        self.Collision_force3.forces[0][0:3] = self.scene.instrument[0]['CFF_visu_2'].forces[0][0:3] * 90 * 3000
        print(self.scene.instrument[0]['CFF_visu_2'].forces[0][0:3])

        # if self.root_node.time.value > 1.0:
        #     self.scene.instrument[0]['force_torque_wire'].forces[0][0:3] = [0.01, 0, 0]
        #
        # if self.root_node.time.value > 1.5:
        #     self.scene.instrument[0]['force_torque_wire'].forces[0][0:3] = [0.05, 0, 0]
        #
        # if self.root_node.time.value > 2.0:
        #     self.scene.instrument[0]['force_torque_wire'].forces[0][0:3] = [0.4, 0, 0]