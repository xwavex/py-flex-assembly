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

# FLEX ASSEMBLY IMPORTS
from gym_flexassembly import data as flexassembly_data

class FlexPolishingEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self,
               gui=True):
        self._timeStep = 1.0 / 1000.0
        self._urdfRoot_pybullet = pybullet_data.getDataPath()
        self._urdfRoot_flexassembly = flexassembly_data.getDataPath()
        # self._observation = []
        # self._envStepCounter = 0
        self._gui = gui
        self._terminated = False
        # self._cam_dist = 1.3
        # self._cam_yaw = 180
        # self._cam_pitch = -40
        self._client_id = -1;

        self.largeValObservation = 100
        self.RENDER_HEIGHT = 720
        self.RENDER_WIDTH = 960

        self._p = p
        if self._gui:
            self._client_id = p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
            # p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])
        else:
            self._client_id = p.connect(p.SHARED_MEMORY_SERVER)


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
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

        # Floor SHOULD BE ALWAYS ID 0
        p.loadURDF(os.path.join(self._urdfRoot_flexassembly, "plane_solid.urdf"), useMaximalCoordinates=True) # Brauche ich fuer die hit rays

        # Table
        table_id = p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/3d", "table_profile_1.urdf"), useFixedBase=True, flags = p.URDF_USE_INERTIA_FROM_FILE)
        table_offset_world_x = -0.85
        table_offset_world_y = 0
        table_offset_world_z = 0
        p.resetBasePositionAndOrientation(table_id, [table_offset_world_x, table_offset_world_y, table_offset_world_z], [0,0,0,1])

        # Workpiece to be polished
        workpiece_id = p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/3d", "bend_wood.urdf"), useFixedBase=False, flags = p.URDF_USE_INERTIA_FROM_FILE)
        wood_offset_table_x = 0.88
        wood_offset_table_y = 0.22
        wood_offset_table_z = 0.73
        wood_offset_world = [table_offset_world_x + wood_offset_table_x, table_offset_world_y + wood_offset_table_y, table_offset_world_z + wood_offset_table_z]
        p.resetBasePositionAndOrientation(workpiece_id, wood_offset_world, [0,0,0,1])

        # Enable rendering again
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


    def reset(self):
        self._terminated = False
        p.resetSimulation()
        # p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(self._timeStep)
        p.setGravity(0, 0, -9.81)


        # TODO how to delete all elements in preface?
        self.loadEnvironment()

        # p.loadURDF(os.path.join(self._urdfRoot_pybullet, "plane.urdf"), [0, 0, -1])
        # p.loadURDF(os.path.join(self._urdfRoot_pybullet, "table/table.urdf"), 0.5000000, 0.00000, -.820000,
        #         0.000000, 0.000000, 0.0, 1.0)
        # xpos = 0.55 + 0.12 * random.random()
        # ypos = 0 + 0.2 * random.random()
        # ang = 3.14 * 0.5 + 3.1415925438 * random.random()
        # orn = p.getQuaternionFromEuler([0, 0, ang])
        # self.blockUid = p.loadURDF(os.path.join(self._urdfRoot_pybullet, "block.urdf"), xpos, ypos, -0.15,
        #                         orn[0], orn[1], orn[2], orn[3])
        # self._kuka = kuka.Kuka(urdfRootPath=self._urdfRoot_pybullet, timeStep=self._timeStep)

        self._envStepCounter = 0
        # Do one simulation step
        p.stepSimulation()
        # self._observation = self.getExtendedObservation()
        self._observation = [] # TODO
        return np.array(self._observation)


    def __del__(self):
        p.disconnect()


    def getExtendedObservation(self):
        self._observation = self._kuka.getObservation()
        gripperState = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaGripperIndex)
        gripperPos = gripperState[0]
        gripperOrn = gripperState[1]
        blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)

        invGripperPos, invGripperOrn = p.invertTransform(gripperPos, gripperOrn)
        gripperMat = p.getMatrixFromQuaternion(gripperOrn)
        dir0 = [gripperMat[0], gripperMat[3], gripperMat[6]]
        dir1 = [gripperMat[1], gripperMat[4], gripperMat[7]]
        dir2 = [gripperMat[2], gripperMat[5], gripperMat[8]]

        gripperEul = p.getEulerFromQuaternion(gripperOrn)
        #print("gripperEul")
        #print(gripperEul)
        blockPosInGripper, blockOrnInGripper = p.multiplyTransforms(invGripperPos, invGripperOrn,
                                                                    blockPos, blockOrn)
        projectedBlockPos2D = [blockPosInGripper[0], blockPosInGripper[1]]
        blockEulerInGripper = p.getEulerFromQuaternion(blockOrnInGripper)
        #print("projectedBlockPos2D")
        #print(projectedBlockPos2D)
        #print("blockEulerInGripper")
        #print(blockEulerInGripper)

        #we return the relative x,y position and euler angle of block in gripper space
        blockInGripperPosXYEulZ = [blockPosInGripper[0], blockPosInGripper[1], blockEulerInGripper[2]]

        #p.addUserDebugLine(gripperPos,[gripperPos[0]+dir0[0],gripperPos[1]+dir0[1],gripperPos[2]+dir0[2]],[1,0,0],lifeTime=1)
        #p.addUserDebugLine(gripperPos,[gripperPos[0]+dir1[0],gripperPos[1]+dir1[1],gripperPos[2]+dir1[2]],[0,1,0],lifeTime=1)
        #p.addUserDebugLine(gripperPos,[gripperPos[0]+dir2[0],gripperPos[1]+dir2[1],gripperPos[2]+dir2[2]],[0,0,1],lifeTime=1)

        self._observation.extend(list(blockInGripperPosXYEulZ))
        return self._observation


    def step(self, action):
        # for i in range(self._actionRepeat):
        #     self._kuka.applyAction(action)
        #     p.stepSimulation()
        #     if self._termination():
        #         break

        #     self._envStepCounter += 1

        # if self._gui:
        #     time.sleep(self._timeStep)

        # self._observation = self.getExtendedObservation()

        done = self._termination()

        return np.array(self._observation), done, {}

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