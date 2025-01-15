
class Simulator:
    def __init__(self, root_node, info):
        self.info = info
        self.root_node = root_node
        self.dt = self.info[5][0]["step"]
        self.gravity = self.info[5][0]["gravity"]
        self.friction_coefficient = self.info[5][0]["friction_coefficient"]

        self.alarmDistance = self.info[5][0]["contactDistance"]*2
        self.contactDistance = self.info[5][0]["contactDistance"]
        # self.contactDistance = self.info[5][0]["contactDistance"]*1.5

        self.background_color = self.info[5][0]["background_color"]

        self.Create_Simulator()

    def Create_Simulator(self):
        self.root_node.findData('gravity').value = self.gravity
        self.root_node.findData('dt').value = self.dt
        # self.root_node.addObject("OglGrid", nbSubdiv=20, size=4, plane="z")  # 0.8就是整个框（包含所有的小框）800mm
        # self.root_node.addObject("OglLineAxis", size=0.1)  # 这里0.1代表箭头的宽度
        # self.root_node.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
        self.root_node.addObject('RequiredPlugin', name="SofaValidation")
        self.root_node.addObject('VisualStyle')

        # confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
        # confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

        # self.root_node.VisualStyle.displayFlags = "showCollisionModels"

        self.root_node.addObject('FreeMotionAnimationLoop')
        self.root_node.addObject('CollisionPipeline')

        self.root_node.addObject('BruteForceBroadPhase')
        self.root_node.addObject('BVHNarrowPhase')
        self.root_node.addObject('LocalMinDistance',
                           alarmDistance=self.alarmDistance, contactDistance=self.contactDistance,
                           angleCone=0.01)

        self.root_node.addObject('RuleBasedContactManager', responseParams="mu=" + str(self.friction_coefficient),
                           name='Response', response='FrictionContactConstraint')

        self.constraints = self.root_node.addObject('GenericConstraintSolver', tolerance=1e-6, maxIterations=1000, computeConstraintForces=1)

        self.root_node.addObject('BackgroundSetting', color=self.background_color)

        self.root_node.addObject('LightManager', name="lightManager",  listening="1")
        # self.root_node.addObject("OglShadowShader")
        #支气管整体导航
        self.root_node.addObject('DirectionalLight', name="light1", color="0.7451 0.7451 0.7451", direction="0 -1 -1")
        self.root_node.addObject('DirectionalLight', name="light2", color="0.7451 0.7451 0.7451", direction="0 1 0")
        self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="0 0 1")
        #支气管局部导航
        # self.root_node.addObject('DirectionalLight', name="light2", color="0.7451 0.7451 0.7451", direction="0 1 0")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="0 0 1")
        #主动脉弓整体导航
        # self.root_node.addObject('DirectionalLight', name="light1", color="0.7 0.7 0.7", direction="0 -1 -1")
        # self.root_node.addObject('DirectionalLight', name="light2", color="0.7451 0.7451 0.7451", direction="0 1 0")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="0 0 1")
        #胶囊局部导航
        # self.root_node.addObject('DirectionalLight', name="light2", color="0.7451 0.7451 0.7451", direction="0 1 0")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="0 0 1")
        #胶囊整体导航
        # self.root_node.addObject('DirectionalLight', name="light1", color="0.7451 0.7451 0.7451", direction="0 -1 -1")
        # self.root_node.addObject('DirectionalLight', name="light2", color="0.7451 0.7451 0.7451", direction="0 1 0")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="0 0 1")


        #视频胶囊整体导航
        # self.root_node.addObject('DirectionalLight', name="light1", color="0.5 0.5 0.5", direction="0 -1 -1")
        # self.root_node.addObject('DirectionalLight', name="light2", color="0.7451 0.7451 0.7451", direction="0 1 0")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="0 0 1")


        #论文截图1
        # self.root_node.addObject('DirectionalLight', name="light1", color="0.5 0.5 0.5", direction="-1 -1 -1")
        # self.root_node.addObject('DirectionalLight', name="light2", color="0.2 0.2 0.2", direction="-1 -0 -1")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.5 0.5 0.5", direction="1 1 1")
        #论文截图2
        # self.root_node.addObject('DirectionalLight', name="light1", color="0.7451 0.7451 0.7451", direction="-1 -1 -1")
        # # self.root_node.addObject('DirectionalLight', name="light2", color="0.2 0.2 0.2", direction="-1 -0 -1")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="1 1 1")
        #论文截图3
        # self.root_node.addObject('DirectionalLight', name="light1", color="0.7451 0.7451 0.7451", direction="-1 -1 -1")
        # self.root_node.addObject('DirectionalLight', name="light2", color="0.7451 0.7451 0.7451", direction="-1 -0 -1")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="1 1 1")
        #论文截图4
        # self.root_node.addObject('DirectionalLight', name="light1", color="0.7451 0.7451 0.7451", direction="-1 -1 -1")
        # self.root_node.addObject('DirectionalLight', name="light2", color="0.7451 0.7451 0.7451", direction="-1 -0 -1")
        # self.root_node.addObject('DirectionalLight', name="light3", color="0.7451 0.7451 0.7451", direction="1 1 1")

        if self.info[5][0]['cam'] == 1 and self.info[2]:
            if self.info[2][0]['object'] == "capsule":
                self.cam = self.root_node.addObject("OglViewport", screenPosition=[0, -400], screenSize=[400, 400], cameraRigid=[0, 0, 0, 0, 0, 0, 1],  zNear="0.005", zFar="1", fovy="30", drawCamera=0)
            if self.info[2][0]['object'] == "wire":
                self.cam = self.root_node.addObject("OglViewport", screenPosition=[0, -400], screenSize=[400, 400], cameraRigid=[0, 0, 0, 0, 0, 0, 1],  zNear="0.001", zFar="1", fovy="30", drawCamera=0)
            # self.root_node.addObject("LightManager", name="lightManager2",  listening="1",  shadows="1",  softShadows="1")
            # self.root_node.addObject("OglShadowShader", name="oglShadowShader1")
            # self.light = self.root_node.addObject('SpotLight', name="spotLight1", shadowTextureSize="2048", position=[0, 0, 0], direction=[0, -1, 0], cutoff="10", drawSource="false", zNear="0.005", zFar="0.5")