import os, inspect

import pybullet as p
import numpy as np
import copy
import math

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data

class SpringClamp:
    def __init__(self, variant='W_QS_1', pos=[0,0,0], orn=[0,0,0,1], max_force = 50.0, joint_clip_index = 1, urdfRootPath=flexassembly_data.getDataPath(), use_inertia_from_urdf=False):
        self._variant = variant
        self._urdfRoot_flexassembly = urdfRootPath
        self._pos = pos
        self._orn = orn

        self._model_id = -1
        self._num_joints = -1

        # upper and lower joint limits
        self._ul = 0.0
        self._ll = -0.0009

        self._max_force = max_force
        self._joint_clip_index = joint_clip_index

        self._use_inertia_from_urdf = use_inertia_from_urdf

        self.reset()

    def getModelId(self):
        return self._model_id

    def setPose(self, pos, orn):
        self._pos = pos
        self._orn = orn
        p.resetBasePositionAndOrientation(self._model_id, self._pos, self._orn)

    def reset(self):
        if self._use_inertia_from_urdf:
            self._model_id = p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/W_QS_1", "W_QS_1.urdf"), useFixedBase=False, flags = p.URDF_USE_INERTIA_FROM_FILE)
        else:
            self._model_id = p.loadURDF(os.path.join(self._urdfRoot_flexassembly+"/W_QS_1", "W_QS_1.urdf"), useFixedBase=False)
        p.resetBasePositionAndOrientation(self._model_id, self._pos, self._orn)

        # Take care of the spring loaded simulation of the clipping mechanism
        p.setJointMotorControl2(bodyIndex=self._model_id, jointIndex=self._joint_clip_index, controlMode=p.POSITION_CONTROL, targetPosition=self._ul, targetVelocity=0, force=self._max_force)

        # Collision
        #                      0x10
        collisionFilterGroup = 0x10
        #                      0x11
        collisionFilterMask =  0x11

        p.setCollisionFilterGroupMask(self._model_id, -1, collisionFilterGroup, collisionFilterMask)
        for i in range(p.getNumJoints(self._model_id)):
            p.setCollisionFilterGroupMask(self._model_id, i, collisionFilterGroup, collisionFilterMask)

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

__all__ = ['SpringClamp']