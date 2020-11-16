"""TODO.
:Author:
  `Dennis Leroy Wigand <dwigand@cor-lab.de>`
"""

import os, inspect

import pybullet as p
import numpy as np
import copy
import math

class Prismatic2FingerGripperPlugin:
    def __init__(self, connect_to_loaded_model_id, finger_1_joint_name, finger_2_joint_name):
        self._connect_to_loaded_model_id = connect_to_loaded_model_id
        self._finger_1_joint_name = finger_1_joint_name
        self._finger_2_joint_name = finger_2_joint_name
        self._finger_1_joint_index = -1
        self._finger_2_joint_index = -1

        # upper and lower joint limits
        self._closing_vel = 0.1
        self._opening_vel = -0.1
        self._max_force = 10.0

        self.convergence_threshold = 0.001 # TODO

        # (OPTIONAL) ROS IMPORTS
        self.ros_loaded = False
        try:
            import rospy
            from std_msgs.msg import Header
            from geometry_msgs.msg import Pose
            from sensor_msgs.msg import Image

            global rospy, Pose, Image, Header

            self.ros_loaded = True
        except ImportError:
            self.ros_loaded = False

        self.reset()

    def reset(self):
        self._already_converged = False
        self._iteration_count = 0
        # Find indices associated with links names
        total_joints = p.getNumJoints(self._connect_to_loaded_model_id)
        self.motorNames = []
        self.motorIndices = []
        self.zeroForces = []
        for i in range(total_joints):
            jointInfo = p.getJointInfo(self._connect_to_loaded_model_id, i)
            if str(self._finger_1_joint_name) == (str(jointInfo[1].decode('UTF-8'))):
                self._finger_1_joint_index = i
            elif str(self._finger_2_joint_name) == (str(jointInfo[1].decode('UTF-8'))):
                self._finger_2_joint_index = i
        
        if self._finger_1_joint_index == -1 or self._finger_2_joint_index == -1:
            # TODO error message
            return False


        self.cmd = self._opening_vel

        # Take care of the gripper mechanism reset
        p.setJointMotorControl2(bodyIndex=self._connect_to_loaded_model_id, jointIndex=self._finger_1_joint_index, controlMode=p.Velocity_Control, targetPosition=0.0, targetVelocity=self.cmd, force=self._max_force)
        p.setJointMotorControl2(bodyIndex=self._connect_to_loaded_model_id, jointIndex=self._finger_2_joint_index, controlMode=p.Velocity_Control, targetPosition=0.0, targetVelocity=self.cmd, force=self._max_force)

    def update(self, convergence_func):
        # TODO calculate convergence based on velocity and not position
        finger_1_joint_state = p.getJointState(self._connect_to_loaded_model_id, self._finger_1_joint_index)
        finger_2_joint_state = p.getJointState(self._connect_to_loaded_model_id, self._finger_2_joint_index)

        if math.fabs(finger_1_joint_state[1]) < self.convergence_threshold and math.fabs(finger_2_joint_state[1]) < self.convergence_threshold:
            if self._iteration_count == 5: # TODO
                if not self._already_converged:
                    # Converged: call convergence_func
                    self._already_converged = True
                    convergence_func() # TODO
            else:
                if self._already_converged:
                    # Moving again
                    self._already_converged = False

        p.setJointMotorControl2(bodyIndex=self._connect_to_loaded_model_id, jointIndex=self._finger_1_joint_index, controlMode=p.Velocity_Control, targetPosition=0.0, targetVelocity=self.cmd, force=self._max_force)
        p.setJointMotorControl2(bodyIndex=self._connect_to_loaded_model_id, jointIndex=self._finger_2_joint_index, controlMode=p.Velocity_Control, targetPosition=0.0, targetVelocity=self.cmd, force=self._max_force)

    def close(self):
        self.cmd = self._closing_vel

    def open(self):
        self.cmd = self._opening_vel

__all__ = ['Prismatic2FingerGripperPlugin']