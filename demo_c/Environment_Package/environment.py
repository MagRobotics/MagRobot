from Pose_Transform.pose_transform import *
import Sofa


def rigid_object(root_node, path, pose, totalMass = 1, color=[1.0, 0.0, 0.0, 0.5], scale="0.001", isAStaticObject=True, flipnormals=False):
    object = root_node.addChild('object')
    object.addObject('MechanicalObject', name="object", template="Rigid3",
                     position=pose, showObject=0, showObjectScale=0.0)
    object.addObject('UniformMass', totalMass=totalMass, name='mass', showAxisSizeFactor=0)
    # object.addObject("FixedConstraint", indices=[i for i in range(1)])
    if not isAStaticObject:
        object.addObject('UncoupledConstraintCorrection')
        object.addObject('EulerImplicitSolver', name='odesolver')
        object.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')

    object_visu = object.addChild("object_visu")

    if path.endswith(".stl") or path.endswith(".STL"):
        object_visu.addObject('MeshSTLLoader', name="loader_visu", filename=path,
                              translation=[0, 0, 0], rotation=[0, 0, 0], scale=scale)
    elif path.endswith(".obj") or path.endswith(".OBJ"):
        object_visu.addObject('MeshOBJLoader', name="loader_visu", filename=path,
                              translation=[0, 0, 0], rotation=[0, 0, 0], scale=scale)

    object_visu.addObject('OglModel', name="model", src="@loader_visu", color=color,
                          updateNormals=False)
    object_visu.addObject('RigidMapping')

    object_collision = object.addChild("object_collision")

    if path.endswith(".stl") or path.endswith(".STL"):
        object_collision.addObject('MeshSTLLoader', name="loader_collision", filename=path, triangulate=True,
                                   scale=scale, flipNormals=flipnormals)
    elif path.endswith(".obj") or path.endswith(".OBJ"):
        object_collision.addObject('MeshOBJLoader', name="loader_collision", filename=path, triangulate=True,
                                   scale=scale, flipNormals=flipnormals)

    object_collision.addObject('MeshTopology', src="@loader_collision")
    object_collision.addObject('MechanicalObject')

    if isAStaticObject:
        object_collision.addObject('TriangleCollisionModel', moving=False, simulated=False)
        object_collision.addObject('LineCollisionModel', moving=False, simulated=False)
        object_collision.addObject('PointCollisionModel', moving=False, simulated=False)
    else:
        object_collision.addObject('TriangleCollisionModel')
        object_collision.addObject('LineCollisionModel')
        object_collision.addObject('PointCollisionModel')
    object_collision.addObject('RigidMapping')


def soft_object(root_node, path, volume_path, pose, isAStaticObject, boxroi, totalMass=0.1, poissonRatio=0.3, youngModulus=300, scale="0.001", color=[1.0, 0.0, 0.0, 0.5]):
    object = root_node.addChild("object")
    translation = list(pose[0:3])
    rotation = list(rot_to_euler(quat_to_rmatrix(pose[3:7])))
    mobject = ElasticMaterialObject(totalMass=totalMass, translation=translation, rotation=rotation, poissonRatio=poissonRatio,
                                    youngModulus=youngModulus, volumeMeshFileName=volume_path, surfaceMeshFileName=path,
                                    collisionMesh=path, scale=[float(scale), float(scale), float(scale)], surfaceColor=color, isAStaticObject=isAStaticObject, boxroi=boxroi)
    object.addChild(mobject)


def visu_object(root_node, path, pose, scale="0.001", color=[1.0, 0.0, 0.0, 0.5]):
    # position = pose[0:3]
    # rotation = rot_to_euler(quat_to_rmatrix(pose[3:7]))

    object = root_node.addChild('object')
    object.addObject('MechanicalObject', name="object", template="Rigid3",
                     position=pose, showObject=1, showObjectScale=0.0)

    visu_object = object.addChild("visu_object")
    if path.endswith(".stl") or path.endswith(".STL"):
        visu_object.addObject('MeshSTLLoader', name="loader", filename=path,
                              translation=[0, 0, 0], rotation=[0, 0, 0], scale=scale)
    elif path.endswith(".obj") or path.endswith(".OBJ"):
        visu_object.addObject('MeshOBJLoader', name="loader", filename=path,
                              translation=[0, 0, 0], rotation=[0, 0, 0], scale=scale)
    visu_object.addObject('OglModel', name="model", src="@loader", color=color,
                          updateNormals=False)
    visu_object.addObject('RigidMapping')


class ElasticMaterialObject(Sofa.Prefab):
    """Creates an object composed of an elastic material."""
    prefabParameters = [
        {'name': 'volumeMeshFileName', 'type': 'string', 'help': 'Path to volume mesh file', 'default': ''},
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [0.001, 0.001, 0.001]},
        {'name': 'surfaceMeshFileName', 'type': 'string', 'help': 'Path to surface mesh file', 'default': ''},
        {'name': 'collisionMesh', 'type': 'string', 'help': 'Path to collision mesh file', 'default': ''},
        {'name': 'withConstrain', 'type': 'bool', 'help': 'Add constraint correction', 'default': True},
        {'name': 'surfaceColor', 'type': 'Vec4d', 'help': 'Color of surface mesh', 'default': [1., 1., 1., 1.]},
        {'name': 'poissonRatio', 'type': 'double', 'help': 'Poisson ratio', 'default': 0.3},
        {'name': 'youngModulus', 'type': 'double', 'help': "Young's modulus", 'default': 18000},
        {'name': 'totalMass', 'type': 'double', 'help': 'Total mass', 'default': 1.0},
        {'name': 'solverName', 'type': 'string', 'help': 'Solver name', 'default': ''}]

    def __init__(self, isAStaticObject, boxroi, *args, **kwargs):

        Sofa.Prefab.__init__(self, *args, **kwargs)

        self.isAStaticObject = isAStaticObject
        self.boxroi = boxroi

        self.AStaticObject()

    def init(self):

        if self.solverName.value == '':
            self.integration = self.addObject('EulerImplicitSolver', name='integration')
            self.solver = self.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')

        if self.volumeMeshFileName.value == '':
            Sofa.msg_error(self, "Unable to create an elastic object because there is no volume mesh provided.")
            return None

        if self.volumeMeshFileName.value.endswith(".msh"):
            self.loader = self.addObject('MeshGmshLoader', name='loader', filename=self.volumeMeshFileName.value,
                                         rotation=list(self.rotation.value), translation=list(self.translation.value),
                                         scale3d=list(self.scale.value))
        elif self.volumeMeshFileName.value.endswith(".gidmsh"):
            self.loader = self.addObject('GIDMeshLoader', name='loader', filename=self.volumeMeshFileName.value,
                                         rotation=list(self.rotation.value), translation=list(self.translation.value),
                                         scale3d=list(self.scale.value))
        else:
            self.loader = self.addObject('MeshVTKLoader', name='loader', filename=self.volumeMeshFileName.value,
                                         rotation=list(self.rotation.value), translation=list(self.translation.value),
                                         scale3d=list(self.scale.value))

        # self.container = self.addObject('MeshTopology', src="@loader")
        self.container = self.addObject('TetrahedronSetTopologyContainer', src="@loader", name='container')

        self.dofs = self.addObject('MechanicalObject', template='Vec3', name='dofs', src="@loader")

        self.mass = self.addObject('UniformMass', totalMass=self.totalMass.value, name='mass')

        self.forcefield = self.addObject('TetrahedronFEMForceField', template='Vec3',
                                         method='large', name='forcefield',
                                         poissonRatio=self.poissonRatio.value, youngModulus=self.youngModulus.value)

        if self.withConstrain.value:
            self.correction = self.addObject('LinearSolverConstraintCorrection', name='correction')

        if self.collisionMesh:
            self.addCollisionModel(self.collisionMesh.value, list(self.rotation.value), list(self.translation.value),
                                   list(self.scale.value))

        if self.surfaceMeshFileName:
            self.addVisualModel(self.surfaceMeshFileName.value, list(self.surfaceColor.value),
                                list(self.rotation.value), list(self.translation.value), list(self.scale.value))

    def AStaticObject(self):
        if self.isAStaticObject:
            for i in range(len(self.boxroi)):
                self.addObject("BoxROI", box=self.boxroi[i], name="box_" + str(i), drawBoxes=False)
                self.addObject('FixedConstraint', indices="@box_" + str(i) + ".indices")
        else:
            pass

    def addCollisionModel(self, collisionMesh, rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0], scale=[1., 1., 1.]):

        self.collisionmodel = self.addChild('CollisionModel')

        if collisionMesh.endswith('.stl'):
            self.collisionmodel.addObject('MeshSTLLoader', name='loader', filename=collisionMesh, rotation=rotation,
                                      translation=translation, scale3d=scale, flipNormals=False)
        elif collisionMesh.endswith('.obj'):
            self.collisionmodel.addObject('MeshOBJLoader', name='loader', filename=collisionMesh, rotation=rotation,
                                      translation=translation, scale3d=scale)

        self.collisionmodel.addObject('TriangleSetTopologyContainer', src='@loader', name='container')

        self.collisionmodel.addObject('MechanicalObject', template='Vec3', name='dofs')

        self.collisionmodel.addObject('TriangleCollisionModel')
        self.collisionmodel.addObject('LineCollisionModel')
        self.collisionmodel.addObject('PointCollisionModel')
        self.collisionmodel.addObject('BarycentricMapping')

    def addVisualModel(self, filename, color, rotation, translation, scale=[1., 1., 1.]):

        object_visu = self.addChild("object_visu")

        if filename.endswith(".stl") or filename.endswith(".STL"):
            object_visu.addObject('MeshSTLLoader', name="loader_visu", filename=filename)
        elif filename.endswith(".obj") or filename.endswith(".OBJ"):
            object_visu.addObject('MeshOBJLoader', name="loader_visu", filename=filename)

        object_visu.addObject('OglModel', name="OglModel", src="@loader_visu",
                       rotation=rotation,
                       translation=translation,
                       scale3d=scale,
                       color=color, updateNormals=False)
        # Add a BarycentricMapping to deform the rendering model to follow the ones of the
        # mechanical model.
        object_visu.addObject('BarycentricMapping')


# class VisualModel(Sofa.Prefab):
#     """  """
#     prefabParameters = [
#         {'name': 'visualMeshPath', 'type': 'string', 'help': 'Path to visual mesh file', 'default': ''},
#         {'name': 'translation', 'type': 'Vec3d', 'help': 'translate visual model', 'default': [0., 0., 0.]},
#         {'name': 'rotation', 'type': 'Vec3d', 'help': 'rotate visual model', 'default': [0., 0., 0.]},
#         {'name': 'scale', 'type': 'Vec3d', 'help': 'scale visual model', 'default': [1., 1., 1.]},
#         {'name': 'color', 'type': 'Vec4d', 'help': 'color put to visual model', 'default': [1., 1., 1., 1.]}]
#
#     def __init__(self, *args, **kwargs):
#         Sofa.Prefab.__init__(self, *args, **kwargs)
#
#     def init(self):
#         self.addObject('RequiredPlugin', pluginName=['SofaOpenglVisual', 'SofaLoader'])
#         path = self.visualMeshPath.value
#         if path.endswith('.stl'):
#             self.addObject('MeshSTLLoader', name='loader', filename=path)
#         elif path.endswith('.obj'):
#             self.addObject('MeshOBJLoader', name='loader', filename=path)
#         else:
#             print("Extension not handled in STLIB/python3/stlib3/visuals for file: " + str(path))
#
#         self.addObject('OglModel', name="OglModel", src="@loader",
#                        rotation=list(self.rotation.value),
#                        translation=list(self.translation.value),
#                        scale3d=list(self.scale.value),
#                        color=list(self.color.value), updateNormals=False)