import pybullet as p
import pybullet as pi
import pybullet_data
import pybullet_planning as planning
import numpy as np
import time
import os
from pybullet_planning.utils import INF
from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion, get_distance_fn, get_extend_fn
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info
from pybullet_planning.interfaces.robots import get_movable_joints, set_joint_positions, get_link_pose, get_custom_limits, get_joint_limits, body_from_name, get_body_name, get_joint_names, get_joint_positions, get_joint_q_index, get_joint
import pybullet_planning.interfaces.robots.joint as jt
from pybullet_planning.interfaces.env_manager import create_box

class FlexPlanning(object):
    def __init__(self, pybullet, robot):
        self._p = pybullet
        self._robot = robot
        ### Nullspace ###
        useNullSpace = 1
        ikSolver = 0
        pandaEndEffectorIndex = 11 #8
        pandaNumDofs = 7 # TODO retrieve from robot

        ll = [-7]*pandaNumDofs
        #upper limits for null space (todo: set them to proper range)
        ul = [7]*pandaNumDofs
        #joint ranges for null space (todo: set them to proper range)
        jr = [7]*pandaNumDofs
        #restposes for null space
        jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
        rp = jointPositions
        ### Nullspace ###

        # HERE = os.path.dirname(__file__)
        # IIWA_ROBOT_URDF = os.path.join(HERE, 'kuka-iiwa-7-bullet', 'model_schunk.urdf')
        #IIWA_ROBOT_URDF = os.path.join(HERE, 'kuka-iiwa-7-egp-40', 'model.urdf')
        # iiwapos = [0.0, 0.0, 0.0]
        # self._p.setAdditionalSearchPath(pybullet_data.getDataPath())
        #robot = p.loadURDF("kuka_iiwa/model.urdf", iiwapos[0], iiwapos[1], iiwapos[2])
        # robot = self._p.loadURDF(IIWA_ROBOT_URDF, iiwapos[0], iiwapos[1], iiwapos[2])

        self._obstacles = []

        block_x = 0.2
        block_y = 0.2
        block_z = 1.5
        block = create_box(block_x, block_y, block_z)
        set_pose(block, Pose(Point(x=block_x, y=block_y, z=block_z), Euler(yaw=np.pi/2)))

        self._obstacles.append(block)

        ###### GET DATA FROM BULLET

    def calculatePath(self, goalPose):
        # orn = p.getQuaternionFromEuler([0.,-1.57,0.]) # TODO

        # Nullspace
        # https://github.com/erwincoumans/xArm-Python-SDK/blob/master/example/wrapper/xarm7/xarm_sim.py

        # strtJntPos = p.calculateInverseKinematics(self._robot, 6, startPose[0:3], startPose[3:7])

        goalJntPos = p.calculateInverseKinematics(self._robot, 6, goalPose[0:3], goalPose[3:7], ll, ul, jr, rp, maxNumIterations=5)

        # set_joint_positions(self._robot, jt.get_movable_joints(self._robot), strtJntPos)
 
        path = planning.plan_joint_motion(self._robot, jt.get_movable_joints(self._robot), goalJntPos, obstacles=self._obstacles)
        print("Path: ", path)

        if path is None:
            print('no plan found')
        else:
            print('a motion plan is found! Press enter to start simulating!')

        return path
