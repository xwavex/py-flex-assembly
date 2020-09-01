#!/usr/bin/python3

""" MAIN: Environment for the clamp assembly scenario.
:Author:
  `Dennis Leroy Wigand <dwigand@cor-lab.de>`
"""

# SYSTEM IMPORTS
import os, inspect

# UTILITY IMPORTS
import math
import numpy as np
import random
import time
import sys

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
from gym_flexassembly.robots.prismatic_2_finger_gripper_plugin import Prismatic2FingerGripperPlugin

# FLEX ASSEMBLY SMARTOBJECTS IMPORTS
from gym_flexassembly.smartobjects.spring_clamp import SpringClamp

from gym_flexassembly.envs.env_interface import EnvInterface

class FlexAssemblyEnv(EnvInterface):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50} # TODO do we need this?)

    def __init__(self,
               stepping=True,
               gui=True,
               direct=False,
               use_real_interface=True):
        super().__init__(gui, direct, use_real_interface=use_real_interface, hz=1000.0, stepping=stepping)

        self._urdfRoot_pybullet = pybullet_data.getDataPath()
        self._urdfRoot_flexassembly = flexassembly_data.getDataPath()
        # self._observation = []
        # self._cam_dist = 1.3
        # self._cam_yaw = 180
        # self._cam_pitch = -40

        self.largeValObservation = 100 # TODO
        self.RENDER_HEIGHT = 720 # TODO
        self.RENDER_WIDTH = 960 # TODO

        self.seed()

        self.cam_global_settings = {'width': 1280,
                                    'height': 720,
                                    'fov': 65,
                                    'near': 0.16,
                                    'far': 10,
                                    'framerate': 5,
                                    'up': [0, -1.0, 0]}

        self.env_reset()

        self.env_loop() # TODO

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

        # Global camera
        self.cam_global_settings['pos'] = [workpiece_2_offset_world[0], workpiece_2_offset_world[1], workpiece_2_offset_world[2] + 0.6]
        self.cam_global_settings['orn'] = [0,0,0,1]
        self.cam_global_settings['target_pos'] = workpiece_2_offset_world
        realsense_camera_id = self._p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/objects", "RealSense_D435.urdf"), useFixedBase=True)
        self._p.resetBasePositionAndOrientation(realsense_camera_id, self.cam_global_settings['pos'], self.cam_global_settings['orn'])
        # tmp_name = str(self._p.getBodyInfo(realsense_camera_id)[1].decode()) + "_0"
        tmp_name = "global"
        self._camera_map[tmp_name] = realsense_camera_id

        # Enable rendering again
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

    def loadRobot(self):
        # Disable rendering
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 0)

        self.kuka7_1 = KukaIIWA7_EGP40(pos=[0,-0.2,0.5], orn=[0,0,0,1])

        # Enable rendering again
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)
        # Store name with as unique identified + "_0" and the id
        self._robot_map[str(self._p.getBodyInfo(self.kuka7_1.getUUid())[1].decode()) + "_0"] = self.kuka7_1.getUUid()

        # Load gripper
        self.kuka7_1_egp = Prismatic2FingerGripperPlugin(self.kuka7_1.getUUid(), "gripper1", "SchunkEGP40_Finger1_joint", "SchunkEGP40_Finger2_joint")

    def loadCameras(self):
        if not self._use_real_interface:
            return
        
        for k,v in self._camera_map.items():
            self.remove_camera(name=k)
            self.add_camera(settings=self.cam_global_settings, name=k, model_id=v)

    def step_internal(self):
        self.kuka7_1_egp.update()

    def reset_internal(self):
        self._p.setGravity(0, 0, -9.81)

        self.loadEnvironment()
        self.loadRobot()
        self.loadCameras()

        # Do one simulation step
        self._p.stepSimulation()

    def observation_internal(self):
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

    def render(self, mode="rgb_array", close=False):
        return np.array([]) # TODO
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
        return False

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def __del__(self):
        self._p.disconnect()

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = super().env_reset
        _seed = seed
        _step = super().env_step

if __name__ == "__main__":
    tmp = ""
    if len(sys.argv) == 2:
        tmp = str(sys.argv[1])
    elif len(sys.argv) > 2:
        print("Invalid arguments!", file=sys.stderr)
        print("Usage: python3 -m gym_flexassembly.planning.flex_planning_ros [extrigger]\n")
        sys.exit(1)

    if tmp == "extrigger":
        inst = FlexAssemblyEnv(stepping=False)
    else:
        inst = FlexAssemblyEnv(stepping=True)