# https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/gym/pybullet_utils/pd_controller_stable.py

import numpy as np

class JointGravityCompensationController(object):

    def __init__(self, pb):
        self._pb = pb

    def compute(self, bodyUniqueId, jointIndices):
        numBaseDofs = 0
        numPosBaseDofs = 0
        baseMass = self._pb.getDynamicsInfo(bodyUniqueId, -1)[0]
        curPos, curOrn = self._pb.getBasePositionAndOrientation(bodyUniqueId)
        q1 = []
        qdot1 = []
        zeroAccelerations = []
        qError = []
        if (baseMass > 0):
            numBaseDofs = 6
            numPosBaseDofs = 7
            q1 = [curPos[0], curPos[1], curPos[2], curOrn[0], curOrn[1], curOrn[2], curOrn[3]]
            qdot1 = [0] * numBaseDofs
            zeroAccelerations = [0] * numBaseDofs
            angDiff = [0, 0, 0]

        numJoints = len(jointIndices)
        jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
        
        for i in range(numJoints):
            q1.append(jointStates[i][0])
            qdot1.append(jointStates[i][1])
            zeroAccelerations.append(0)
        q = np.array(q1)
        qdot = np.array(qdot1)
        c = np.array(self._pb.calculateInverseDynamics(bodyUniqueId, q1, qdot1, zeroAccelerations))
        tau = c
        return tau