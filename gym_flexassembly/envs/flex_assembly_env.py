#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3

# SYSTEM IMPORTS
import os, inspect
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# print("current_dir=" + currentdir)
# os.sys.path.insert(0, currentdir)

# UTILITY IMPORTS
import math
import numpy as np
import random
import time

# PYBULLET IMPORTS
import pybullet as p
import pybullet_data

# GYM IMPORTS
import gym
from gym import error, spaces, utils
from gym.utils import seeding

# DEPLOYMENT IMPORTS
from pkg_resources import parse_version

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data

# FLEX ASSEMBLY ROBOT IMPORTS
from gym_flexassembly.robots.kuka_iiwa import KukaIIWA, KukaIIWA7, KukaIIWA14
from gym_flexassembly.robots.kuka_iiwa_egp_40 import KukaIIWA_EGP40, KukaIIWA7_EGP40

# FLEX ASSEMBLY SMARTOBJECTS IMPORTS
from gym_flexassembly.smartobjects.spring_clamp import SpringClamp

from gym_flexassembly.envs.env_interface import EnvInterface

class FlexAssemblyEnv(EnvInterface):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self,
               stepping=True,
               gui=True):
        super().__init__(gui)

        self.robotList = []

        self._stepping = stepping
        self._timeStep = 1.0 / 1000.0
        self._urdfRoot_pybullet = pybullet_data.getDataPath()
        self._urdfRoot_flexassembly = flexassembly_data.getDataPath()
        # self._observation = []
        # self._envStepCounter = 0
        self._terminated = False
        # self._cam_dist = 1.3
        # self._cam_yaw = 180
        # self._cam_pitch = -40

        self.largeValObservation = 100 # TODO
        self.RENDER_HEIGHT = 720 # TODO
        self.RENDER_WIDTH = 960 # TODO

        self.seed()

        self.reset()

        # observationDim = len(self.getExtendedObservation())
        # observation_high = np.array([self.largeValObservation] * observationDim)
        # if (self._isDiscrete):
        #     self.action_space = spaces.Discrete(7)
        # else:
        #     action_dim = 3
        #     self._action_bound = 1
        #     action_high = np.array([self._action_bound] * action_dim)
        #     self.action_space = spaces.Box(-action_high, action_high)
        # self.observation_space = spaces.Box(-observation_high, observation_high)
    

    def loadEnvironment(self):
        # print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Disable rendering
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 0)

        # # Floor SHOULD BE ALWAYS ID 0
        # self._p.loadURDF(os.path.join(self._urdfRoot_flexassembly, "objects/plane_solid.urdf"), useMaximalCoordinates=True) # Brauche ich fuer die hit rays

        # Table
        table_id = self._p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/objects", "table_profile_1.urdf"), useFixedBase=True, flags = self._p.URDF_USE_INERTIA_FROM_FILE)
        table_offset_world_x = -0.85
        table_offset_world_y = 0
        table_offset_world_z = 0
        self._p.resetBasePositionAndOrientation(table_id, [table_offset_world_x, table_offset_world_y, table_offset_world_z], [0,0,0,1])

        # Load Rail
        rail_id = self._p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/flexassembly", "rail.urdf"), useFixedBase=True)
        self._p.resetBasePositionAndOrientation(rail_id, [table_offset_world_x+0.50, table_offset_world_y+0.25, table_offset_world_z+0.75], [0,0,0,1])

        # Workpiece clamp 1
        workpiece_1_offset_table_x = 0.60
        workpiece_1_offset_table_y = 0.20
        workpiece_1_offset_table_z = 0.75
        workpiece_1_offset_world = [table_offset_world_x + workpiece_1_offset_table_x, table_offset_world_y + workpiece_1_offset_table_y, table_offset_world_z + workpiece_1_offset_table_z]
        # workpiece_1 = SpringClamp(pos=workpiece_1_offset_world, orn=[0,-0.131,0.991,0])workpiece_1 = SpringClamp(pos=workpiece_1_offset_world, orn=[0,-0.131,0.991,0])
        workpiece_1 = SpringClamp(pos=workpiece_1_offset_world)

        # Workpiece clamp 2
        workpiece_2_offset_table_x = 0.70
        workpiece_2_offset_table_y = 0.20
        workpiece_2_offset_table_z = 0.75
        workpiece_2_offset_world = [table_offset_world_x + workpiece_2_offset_table_x, table_offset_world_y + workpiece_2_offset_table_y, table_offset_world_z + workpiece_2_offset_table_z]
        workpiece_2 = SpringClamp(pos=workpiece_2_offset_world)

        # Workpiece clamp 3
        workpiece_3_offset_table_x = 0.80
        workpiece_3_offset_table_y = 0.20
        workpiece_3_offset_table_z = 0.75
        workpiece_3_offset_world = [table_offset_world_x + workpiece_3_offset_table_x, table_offset_world_y + workpiece_3_offset_table_y, table_offset_world_z + workpiece_3_offset_table_z]
        workpiece_3 = SpringClamp(pos=workpiece_3_offset_world)

        # Enable rendering again
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

    def loadRobot(self):
        # Disable rendering
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 0)

        self.kuka7_1 = KukaIIWA7_EGP40()
        self._p.resetBasePositionAndOrientation(self.kuka7_1.getUUid(), [0,-0.2,0.5], [0,0,0,1])
        collisionFilterGroup_kuka = 0x10
        collisionFilterMask_kuka = 0x1
        for i in range(self._p.getNumJoints(self.kuka7_1.getUUid())):
            self._p.setCollisionFilterGroupMask(self.kuka7_1.getUUid(), i-1, collisionFilterGroup_kuka, collisionFilterMask_kuka)
        self._p.enableJointForceTorqueSensor(self.kuka7_1.getUUid(), 8) # Why 8?

        arm_ft_7 = self._p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0.6, 0.3, 0.1], parentObjectUniqueId=self.kuka7_1.getUUid(), parentLinkIndex=7)

        # Enable rendering again
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

        self.robotList.append(self.kuka7_1.getUUid())

    def getRobots(self):
        return self.robotList

    def reset(self):
        self._terminated = False
        self._p.resetSimulation()
        # self._p.setPhysicsEngineParameter(numSolverIterations=150)
        self._p.setTimeStep(self._timeStep)
        self._p.setGravity(0, 0, -9.81)


        # TODO how to delete all elements in preface?
        # Floor SHOULD BE ALWAYS ID 0
        self._p.loadURDF(os.path.join(flexassembly_data.getDataPath(), "objects/plane_solid.urdf"), useMaximalCoordinates=True) # Brauche ich fuer die hit rays

        self.loadEnvironment()
        self.loadRobot()

        # self._p.loadURDF(os.path.join(self._urdfRoot_pybullet, "plane.urdf"), [0, 0, -1])
        # self._p.loadURDF(os.path.join(self._urdfRoot_pybullet, "table/table.urdf"), 0.5000000, 0.00000, -.820000,
        #         0.000000, 0.000000, 0.0, 1.0)
        # xpos = 0.55 + 0.12 * random.random()
        # ypos = 0 + 0.2 * random.random()
        # ang = 3.14 * 0.5 + 3.1415925438 * random.random()
        # orn = self._p.getQuaternionFromEuler([0, 0, ang])
        # self.blockUid = self._p.loadURDF(os.path.join(self._urdfRoot_pybullet, "block.urdf"), xpos, ypos, -0.15,
        #                         orn[0], orn[1], orn[2], orn[3])
        # self._kuka = kuka.Kuka(urdfRootPath=self._urdfRoot_pybullet, timeStep=self._timeStep)

        self._envStepCounter = 0
        # Do one simulation step
        self._p.stepSimulation()
        # self._observation = self.getExtendedObservation()
        self._observation = [] # TODO
        return np.array(self._observation)


    def __del__(self):
        self._p.disconnect()


    def getExtendedObservation(self):
        self._observation = self._kuka.getObservation()
        gripperState = self._p.getLinkState(self._kuka.kukaUid, self._kuka.kukaGripperIndex)
        gripperPos = gripperState[0]
        gripperOrn = gripperState[1]
        blockPos, blockOrn = self._p.getBasePositionAndOrientation(self.blockUid)

        invGripperPos, invGripperOrn = self._p.invertTransform(gripperPos, gripperOrn)
        gripperMat = self._p.getMatrixFromQuaternion(gripperOrn)
        dir0 = [gripperMat[0], gripperMat[3], gripperMat[6]]
        dir1 = [gripperMat[1], gripperMat[4], gripperMat[7]]
        dir2 = [gripperMat[2], gripperMat[5], gripperMat[8]]

        gripperEul = self._p.getEulerFromQuaternion(gripperOrn)
        #print("gripperEul")
        #print(gripperEul)
        blockPosInGripper, blockOrnInGripper = self._p.multiplyTransforms(invGripperPos, invGripperOrn,
                                                                    blockPos, blockOrn)
        projectedBlockPos2D = [blockPosInGripper[0], blockPosInGripper[1]]
        blockEulerInGripper = self._p.getEulerFromQuaternion(blockOrnInGripper)
        #print("projectedBlockPos2D")
        #print(projectedBlockPos2D)
        #print("blockEulerInGripper")
        #print(blockEulerInGripper)

        #we return the relative x,y position and euler angle of block in gripper space
        blockInGripperPosXYEulZ = [blockPosInGripper[0], blockPosInGripper[1], blockEulerInGripper[2]]

        #self._p.addUserDebugLine(gripperPos,[gripperPos[0]+dir0[0],gripperPos[1]+dir0[1],gripperPos[2]+dir0[2]],[1,0,0],lifeTime=1)
        #self._p.addUserDebugLine(gripperPos,[gripperPos[0]+dir1[0],gripperPos[1]+dir1[1],gripperPos[2]+dir1[2]],[0,1,0],lifeTime=1)
        #self._p.addUserDebugLine(gripperPos,[gripperPos[0]+dir2[0],gripperPos[1]+dir2[1],gripperPos[2]+dir2[2]],[0,0,1],lifeTime=1)

        self._observation.extend(list(blockInGripperPosXYEulZ))
        return self._observation


    def step(self, action):
        # for i in range(self._actionRepeat):
        #     self._kuka.applyAction(action)
        #     self._p.stepSimulation()
        #     if self._termination():
        #         break

        #     self._envStepCounter += 1

        # if self._gui:
        #     time.sleep(self._timeStep)

        # self._observation = self.getExtendedObservation()

        # self.kuka7_1.getObservation()

        super().step_sim()

        done = self._termination()

        # return np.array(self._observation), done, {}
        return

    def render(self, mode="rgb_array", close=False):
        if mode != "rgb_array":
            return np.array([])

        base_pos, orn = self._p.getBasePositionAndOrientation(self._kuka.kukaUid)
        view_matrix = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
                                                                distance=self._cam_dist,
                                                                yaw=self._cam_yaw,
                                                                pitch=self._cam_pitch,
                                                                roll=0,
                                                                upAxisIndex=2)
        proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
                                                        aspect=float(self.RENDER_WIDTH) / self.RENDER_HEIGHT,
                                                        nearVal=0.1,
                                                        farVal=100.0)
        (_, _, px, _, _) = self._p.getCameraImage(width=self.RENDER_WIDTH,
                                                height=self.RENDER_HEIGHT,
                                                viewMatrix=view_matrix,
                                                projectionMatrix=proj_matrix,
                                                renderer=self._p.ER_BULLET_HARDWARE_OPENGL)
        #renderer=self._p.ER_TINY_RENDERER)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (self.RENDER_HEIGHT, self.RENDER_WIDTH, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def _termination(self):
        # state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)
        # actualEndEffectorPos = state[0]

        # #print("self._envStepCounter")
        # #print(self._envStepCounter)
        # if (self._terminated or self._envStepCounter > self._maxSteps):
        #     self._observation = self.getExtendedObservation()
        #     return True

        # maxDist = 0.005
        # closestPoints = p.getClosestPoints(self._kuka.trayUid, self._kuka.kukaUid, maxDist)

        # if (len(closestPoints)):  #(actualEndEffectorPos[2] <= -0.43):
        #     self._terminated = True
        #     #print("terminating, closing gripper, attempting grasp")
        #     #start grasp and terminate
        #     fingerAngle = 0.3
        #     for i in range(100):
        #         graspAction = [0, 0, 0.0001, 0, fingerAngle]
        #         self._kuka.applyAction(graspAction)
        #         p.stepSimulation()
        #         fingerAngle = fingerAngle - (0.3 / 100.)
        #         if (fingerAngle < 0):
        #             fingerAngle = 0

        #     for i in range(1000):
        #         graspAction = [0, 0, 0.001, 0, fingerAngle]
        #         self._kuka.applyAction(graspAction)
        #         p.stepSimulation()
        #         blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)
        #         if (blockPos[2] > 0.23):
        #             #print("BLOCKPOS!")
        #             #print(blockPos[2])
        #             break

        #         state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)
        #         actualEndEffectorPos = state[0]
        #         if (actualEndEffectorPos[2] > 0.5):
        #             break

        #     self._observation = self.getExtendedObservation()
        #     return True
        return False
    

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed
        _step = step
