"""This is the interface object that encapsulates the setup of the Schunk EGP40 gripper. Since the priority of this project ATM resides on the OROCOS RTT side for configuring the robot, this class may not be fully functional. 
:Author:
  `Dennis Leroy Wigand <dwigand@cor-lab.de>`
"""

import os, inspect

import pybullet as p
import numpy as np
import copy
import math

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data

class SchunkEGP40:
    def __init__(self, connect_to_loaded_model_id=-1, connect_to_start_link=-1, urdfRootPath=flexassembly_data.getDataPath(), use_inertia_from_urdf=False):
        self._urdfRoot_flexassembly = urdfRootPath

        self._model_id = -1
        self._connect_to_loaded_model_id = connect_to_loaded_model_id
        self._connect_to_start_link = connect_to_start_link
        if self._connect_to_loaded_model_id > 0:
            self._model_id = connect_to_loaded_model_id
        
        self._num_joints = -1

        # upper and lower joint limits
        self._ul = 0.0
        self._ll = -0.0009

        self._pos = [0,0,0]
        self._orn = [0,0,0,1]

        self._use_inertia_from_urdf = use_inertia_from_urdf

        self.reset()

    def getModelId(self):
        return self._model_id

    def setPose(self, pos, orn):
        self._pos = pos
        self._orn = orn
        p.resetBasePositionAndOrientation(self._model_id, self._pos, self._orn)

    def reset(self):
        if self._connect_to_loaded_model_id == -1:
            if self._use_inertia_from_urdf:
                self._model_id = p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/robots/schunk-egp-40", "model.urdf"), useFixedBase=False, flags = p.URDF_USE_INERTIA_FROM_FILE)
            else:
                self._model_id = p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/robots/schunk-egp-40", "model.urdf"), useFixedBase=False)
            p.resetBasePositionAndOrientation(self._model_id, self._pos, self._orn)

        # TODO Take care of the gripper mechanism
        p.setJointMotorControl2(bodyIndex=self._model_id, jointIndex=self._joint_clip_index, controlMode=p.POSITION_CONTROL, targetPosition=self._ul, targetVelocity=0, force=self._max_force)

        # TODO set specific control mode
        

    def getMotorIndices(self):
        return self.motorIndices

    def getObservationDimension(self):
        return len(self.getObservation())

    def getUUid(self):
        return self._model_id

    def getObservation(self):
        observation = []

        state = p.getLinkState(self._model_id, -1)
        observation.extend(list(state))

        joint_state = p.getJointState(self._model_id, self._joint_clip_index)
        observation.extend(list(joint_state))

        # Print Debug Stuff for Joint 0
        # js_0 = joint_states[0]
        # print("len(js_0) " + str(len(js_0)))
        # q_0 = js_0[0]
        # print("q_0 " + str(q_0))
        # qd_0 = js_0[1]
        # print("qd_0 " + str(qd_0))
        # For Force Sensor
        # jointReactionForces_0 = js_0[2]
        # appliedJointMotorTorque_0 = js_0[3]
        # print("appliedJointMotorTorque_0 " + str(appliedJointMotorTorque_0))
        # q_pos[0][i] = joint_states[0][0]
        # a = joint_states[1][0]
        # print("joint_states[1][0] = " + str(joint_states[1][0]))
        # q_pos[1][i] = a

        # q_vel[0][i] = joint_states[0][1]
        # q_vel[1][i] = joint_states[1][1]

        
        return observation

    def setCommand(self, cmd):
        # TODO
        pass

__all__ = ['SchunkEGP40']