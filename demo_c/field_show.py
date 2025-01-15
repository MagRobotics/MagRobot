import Sofa
import numpy as np
from Magnetic_Engine.permanent_magnet import field_gradient


#参数：长宽高、间距、箭头尺寸
class magnetic_field_show(Sofa.Core.Controller):

    def __init__(self, root_node, show, scene, info, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.root_node = root_node

        self.field_show = show

        self.scene = scene

        self.info = info

        self.topology()

        self.show = self.root_node.addChild("field_show")
        self.MO = self.show.addObject('MechanicalObject', name="field", template="Vec3d", position=self.grid_points, showObject=True, showObjectScale=0)
        self.show.addObject('UniformMass', name="mass", vertexMass=0.0000000001)
        self.cons = self.show.addObject('ConstantForceField', indices=np.arange(len(self.grid_points)), name='ceshi', forces=np.zeros((len(self.grid_points), 3)), indexFromEnd=True)
        self.show.addObject('FixedConstraint', template="Vec3d", name="default3", indices=np.arange(len(self.grid_points)))
        self.show.addObject('UncoupledConstraintCorrection')
        self.show.addObject('EulerImplicitSolver', name='odesolver')
        self.show.addObject('SparseLDLSolver', name="solver", template='CompressedRowSparseMatrixd')
        # self.show.addObject('Monitor', template="Vec3d", indices=np.arange(len(self.grid_points)), showForces=1, ForcesColor="0 1 1 1", sizeFactor=0.55, showMinThreshold="0.000001")
        self.show.addObject('Monitor', template="Vec3d", indices=np.arange(len(self.grid_points)), showForces=1, ForcesColor="0 1 1 1", sizeFactor=1, showMinThreshold="0.000001") # 场图

    def onAnimateBeginEvent(self, event):

        for i in range(len(self.grid_points)):

            pos = self.MO.position[len(self.grid_points) - i - 1]

            if self.info[1][0]["drive_yuan"] == "still_elc":

                field = self.scene.magnetic_source[0]['electromagnet'].field_gradient(np.array([[pos[0]], [pos[1]], [pos[2]]]), self.scene.current)
                # self.cons.forces[i][:] = np.array([field[0, 0], field[1, 0], field[2, 0]]) / 40
                # if np.linalg.norm(field[0:3]) > 0.3:
                if np.linalg.norm(field[0:3]) > 3:
                    self.cons.forces[i][:] = np.array([0, 0, 0])
                else:
                    # self.cons.forces[i][:] = np.array([field[0, 0], field[1, 0], field[2, 0]]) * 1.3 / 2    #主动脉弓
                    # self.cons.forces[i][:] = np.array([field[0, 0], field[1, 0], field[2, 0]]) * 1.3 / 60    #消化道
                    self.cons.forces[i][:] = np.array([field[0, 0], field[1, 0], field[2, 0]]) * 1.3 / 0.15  # 场图
            else:
                Pa, ma_ = self.scene.magnetic_source[0]['robot'].fkine(self.scene.magnetic_source[0]['theta'])
                ma = ma_ * self.scene.magnetic_source[0]['moment']
                field, gradient = field_gradient(np.array([[pos[0]], [pos[1]], [pos[2]]]) - Pa, ma)
                if np.linalg.norm(field) > 0.3:
                    self.cons.forces[i][:] = np.array([0, 0, 0])
                else:
                    self.cons.forces[i][:] = np.array([field[0, 0], field[1, 0], field[2, 0]]) / 4


    def topology(self):
        # 给定的长方体的xyz的最小值和最大值  [-0.1,0.2,1.1,0.1,0.4,1.3,0.025]  [0.93333,0.77255,0.56863,0.1]
        #胶囊[[-0.1,0.15,1.075,0.1,0.35,1.15,0.025]]
        xmin, ymin, zmin = self.field_show[0], self.field_show[1], self.field_show[2]
        xmax, ymax, zmax = self.field_show[3], self.field_show[4], self.field_show[5]

        # 网格点距离
        grid_spacing = self.field_show[6]

        # 计算网格点数量（向下取整）
        num_x = int(round((xmax - xmin), 8) / grid_spacing) + 1
        num_y = int(round((ymax - ymin), 8) / grid_spacing) + 1
        num_z = int(round((zmax - zmin), 8) / grid_spacing) + 1

        # 生成网格点的坐标
        x = np.linspace(xmin, xmin + num_x * grid_spacing, num_x, endpoint=False)
        y = np.linspace(ymin, ymin + num_y * grid_spacing, num_y, endpoint=False)
        z = np.linspace(zmin, zmin + num_z * grid_spacing, num_z, endpoint=False)

        # 生成网格点
        xx, yy, zz = np.meshgrid(x, y, z, indexing='ij')
        self.grid_points = np.vstack([xx.ravel(), yy.ravel(), zz.ravel()]).T



