#!/usr/bin/python

import pybullet as p
import time
import numpy as np
import math

try:
    from gym_flexassembly.robots import KukaIIWA7
except (ImportError, SystemError):
    from robots import KukaIIWA7

try:
    from gym_flexassembly.controller import JointGravityCompensationController, JointPDController, OperationalSpaceController
except (ImportError, SystemError):
    from controller import JointGravityCompensationController, JointPDController, OperationalSpaceController

import gym_flexassembly.data as data
urdfRootPath = data.getDataPath()

import os

# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client)
# p.setGravity(0, 0, 0, physicsClientId=client)

import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")

p.setRealTimeSimulation(0)

# frame_ghost_id = p.loadURDF(os.path.join(urdfRootPath, "frame_ghost.urdf"), useFixedBase=True)
# p.resetBasePositionAndOrientation(frame_ghost_id, [0, 0.5, 1],
#                                       [0.000000, 0.000000, 0.000000, 1.000000])
# p.changeVisualShape(frame_ghost_id, -1, rgbaColor=[1, 1, 1, 0])


p.setTimeStep(0.001)

# Load Object
schunk = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/schunk-egp-40/model.urdf", useFixedBase=False, flags = p.URDF_USE_INERTIA_FROM_FILE)
p.resetBasePositionAndOrientation(schunk, [1, 1, 1], [0,0,0,1])

p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=schunk, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=schunk, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=schunk, parentLinkIndex=-1)

# # GRAV COMP
ctrl = JointGravityCompensationController(p)

# # Create a constraint to keep the fingers centered
# c = p.createConstraint(schunk,
#                     0,
#                     schunk,
#                     1,
#                     jointType=p.JOINT_GEAR,
#                     jointAxis=[1, 0, 0],
#                     parentFramePosition=[0, 0, 0],
#                     childFramePosition=[0, 0, 0])
# p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

dp_joint_pos_0 = p.addUserDebugParameter("force Z", -10, 10, 9.81*0.6+2*0.04)

# kpp = p.addUserDebugParameter("kp", 0, 500, 200)
# kdd = p.addUserDebugParameter("kd", 0, 30, 10)
kps = p.addUserDebugParameter("kp", 0, 500, 10)
kds = p.addUserDebugParameter("kd", 0, 30, 0.1)

count = 0
timeStep = 1.0/1000.0
while 1:
    motorNames = []
    motorIndices = []
    zeroForces = []
    numJoints = p.getNumJoints(schunk)
    for i in range(numJoints):
        jointInfo = p.getJointInfo(schunk, i)
        qIndex = jointInfo[3]
        if qIndex > -1:
            print("motorname " + str(jointInfo[1]) + ", index " + str(i))
            motorNames.append(str(jointInfo[1]))
            motorIndices.append(i)
            zeroForces.append(0.0)
        else:
            print("ignored joint " + str(jointInfo[1]) + ", index " + str(i))

    for i in range(len(motorIndices)):
        p.resetJointState(schunk, motorIndices[i], 0)

    # joint_pos_0 = p.readUserDebugParameter(dp_joint_pos_0)
    # kp = p.readUserDebugParameter(kpp)
    # kd = p.readUserDebugParameter(kdd)

    # pos, orn = p.getBasePositionAndOrientation(schunk)
    # # _base_mass_urdf = p.getDynamicsInfo(schunk, -1)
    # p.changeDynamics(schunk, -1, mass=0.6)
    # print(p.getDynamicsInfo(schunk, -1))
    # # print(_base_mass_urdf)
    # # print(p.getDynamicsInfo(schunk, 0))
    # # print(p.getDynamicsInfo(schunk, 1))
    # vpos, vorn = p.getBaseVelocity(schunk)

    # pError = kp*([0,0,0.5]-np.array(pos)) - kd*np.array(vpos)
    # # pError[2] = 2*(0.5-np.array(pos)[2]) - 0.1*np.array(vpos)[2]
    # pError = np.array(pError)
    # pError = pError + np.array([0,0,joint_pos_0])

    # p.applyExternalForce(schunk, -1, np.array(pError), pos, flags=p.WORLD_FRAME)

    # p.getNumJoints(arm.getUUid())
    jointStates = p.getJointStates(schunk, motorIndices)
    q1 = []
    qdot1 = []
    zeroAccelerations = []
    for i in range(numJoints):
        q1.append(jointStates[i][0])
        qdot1.append(jointStates[i][1])
        zeroAccelerations.append(0)

    jointStates = p.getJointStates(schunk, motorIndices)

    for i in range(numJoints):
        q1.append(jointStates[i][0])
        qdot1.append(jointStates[i][1])
        zeroAccelerations.append(0)
    q = np.array(q1)
    qdot = np.array(qdot1)
    # qdes = np.array(desiredPositions)
    # qdotdes = np.array(desiredVelocities)
    # for j in range(numJoints):
    #     qError.append(desiredPositions[j + numPosBaseDofs] - q1[j + numPosBaseDofs])
    # qdotError = qdotdes - qdot
    Kp = np.diagflat(kps)
    Kd = np.diagflat(kds)
    # p = Kp.dot(qError)
    # d = Kd.dot(qdotError)


    M1 = p.calculateMassMatrix(schunk, q1)
    M = np.array(M1)
    M = (M + Kd * timeStep)
    c1 = p.calculateInverseDynamics(schunk, q1, qdot1, zeroAccelerations)
    c = np.array(c1)
    A = M
    b = -c + p + d
    qddot = np.linalg.solve(A, b)
    tau = p + d - Kd.dot(qddot) * timeStep

    # tau = p + d + c
    maxF = np.array(maxForces)
    tau = np.clip(tau, -maxF, maxF)

    # tau = tau + c
    tau = c

    # if (count % 10 == 0):
    #     count = 0
    #     state = p.getLinkState(arm.getUUid(), 7)
    #     pos = state[0]
    #     orn = state[1]
    #     orn_des_bullet = p.getQuaternionFromEuler([crr, crp, cry])
    #     p.resetBasePositionAndOrientation(frame_ghost_id, [0, 0.5, 1], orn_des_bullet)
    #     p.resetBasePositionAndOrientation(frame_ghost_id_2, [0, 0.2, 1], orn)

    # arm.setCommand(cmd)

    p.stepSimulation()
    count = count + 1

    # time.sleep(1/500) # TODO DLW
