#!/usr/bin/python

import pybullet as p
import time
import numpy as np
import math

try:
    from .robots import KukaIIWA7
except (ImportError, SystemError):
    from robots import KukaIIWA7

client = p.connect(p.GUI_SERVER)
p.setGravity(0, 0, -9.81, physicsClientId=client)

import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")

arm = KukaIIWA7()

# # Change position and orientation
# p.resetBasePositionAndOrientation(arm, [-0.100000, 0.000000, 0.070000], [0.000000, 0.000000, 0.000000, 1.000000])

# Default Joint Positions
jointPositions = [
    0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
    -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
]
# Default Parameters
maxForce = 200.0

motorNames = []
motorIndices = []

dv = 2.0
dp_joint_pos_0 = p.addUserDebugParameter("j0", -dv, dv, 0)
dp_joint_pos_1 = p.addUserDebugParameter("j1", -dv, dv, 0)
dp_joint_pos_2 = p.addUserDebugParameter("j2", -dv, dv, 0)
dp_joint_pos_3 = p.addUserDebugParameter("j3", -dv, dv, 0)
dp_joint_pos_4 = p.addUserDebugParameter("j4", -dv, dv, 0)
dp_joint_pos_5 = p.addUserDebugParameter("j5", -dv, dv, 0)
dp_joint_pos_6 = p.addUserDebugParameter("j6", -dv, dv, 0)

p.setRealTimeSimulation(0)

arm.setControlMode("JOINT_TORQUE_CONTROL")
# print("arm.getUUid() = " + str(arm.getUUid()))

p.setTimeStep(0.001) # TODO DLW

# p.setRealTimeSimulation(1)

# specify which parameters [x, y, z, alpha, beta, gamma] to control
# NOTE: needs to be an array to properly select elements of J and u_task
ctrlr_dof = np.array([True, True, True, True, True, True])

# control gains
kp = 300
ko = 300
kv = np.sqrt(kp+ko) * 1.5

orientations = [
    [0, 0, 0],
    [np.pi/4, np.pi/4, np.pi/4],
    [-np.pi/4, -np.pi/4, np.pi/2],
    [0, 0, 0],
    ]
positions = [
    [0.15, -0.1, 0.6],
    [-.15, 0.0, .7],
    [.2, .2, .6],
    [0.15, -0.1, 0.6]
    ]

pos = positions[0]
orn = orientations[0]
count = 0
index = 0

while 1:
    joint_pos_0 = p.readUserDebugParameter(dp_joint_pos_0)
    joint_pos_1 = p.readUserDebugParameter(dp_joint_pos_1)
    joint_pos_2 = p.readUserDebugParameter(dp_joint_pos_2)
    joint_pos_3 = p.readUserDebugParameter(dp_joint_pos_3)
    joint_pos_4 = p.readUserDebugParameter(dp_joint_pos_4)
    joint_pos_5 = p.readUserDebugParameter(dp_joint_pos_5)
    joint_pos_6 = p.readUserDebugParameter(dp_joint_pos_6)

    numJoints = len(arm.getMotorIndices())
    # p.getNumJoints(arm.getUUid())
    jointStates = p.getJointStates(arm.getUUid(), arm.getMotorIndices())
    q1 = []
    qdot1 = []
    zeroAccelerations = []
    for i in range(numJoints):
        q1.append(jointStates[i][0])
        qdot1.append(jointStates[i][1])
        zeroAccelerations.append(0)
    # q = np.array(q1)
    # qdot = np.array(qdot1)
    # # print("len qdot " + str(len(qdot)))
    # qdes = np.array([joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6])
    # # print("len qdes " + str(len(qdes)))
    # qdotdes = np.array([0,0,0,0,0,0,0])
    # # print("len qdotdes " + str(len(qdotdes)))
    # qError = qdes - q
    # # print("len qError " + str(len(qError)))
    # qdotError = qdotdes - qdot
    # # print("len qdotError " + str(len(qdotError)))
    # Kp = np.diagflat([2,2,2,2,2,2,2])
    # # print("Kp " + str(Kp))
    # Kd = np.diagflat([0.1,0.1,0.1,0.1,0.1,0.1,0.1])
    # pp = Kp.dot(qError)
    # dd = Kd.dot(qdotError)
    # # forces = pp + dd

    ##########################################################################
    if (count % 200) == 0:
        # change target once every simulated second
        if index >= len(orientations):
            break
        orn = orientations[index]
        pos = positions[index]
        index += 1
    ##########################################################################
    result = p.getLinkState(arm.getUUid(), numJoints - 1, computeLinkVelocity=1, computeForwardKinematics=1)
    link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

    target = np.hstack([pos, orn])
    # calculate the Jacobian for the end effector
    # and isolate relevate dimensions
    # # Get the joint and link state directly from Bullet.
    # mpos, mvel, mtorq = self.getMotorJointStates(bodyUniqueId)
    mpos = q1
    zero_vec = [0.0] * len(mpos)
    jac_t, jac_r = p.calculateJacobian(arm.getUUid(), numJoints - 1, com_trn, mpos, zero_vec, zero_vec)
    J = np.concatenate((jac_t, jac_r))

    ##########################################################################
    # calculate the inertia matrix in task space
    M = np.array(p.calculateMassMatrix(arm.getUUid(), q1))
    M_inv = np.linalg.inv(M)
    Mx_inv = np.dot(J, np.dot(M_inv, J.T))
    if np.linalg.det(Mx_inv) != 0:
        # do the linalg inverse if matrix is non-singular
        # because it's faster and more accurate
        Mx = np.linalg.inv(Mx_inv)
    else:
        # using the rcond to set singular values < thresh to 0
        # singular values < (rcond * max(singular_values)) set to 0
        Mx = np.linalg.pinv(Mx_inv, rcond=.005)
    
    c = np.array(p.calculateInverseDynamics(arm.getUUid(), q1, qdot1, zeroAccelerations))

    u_task = np.zeros(6)  # [x, y, z, alpha, beta, gamma]
    # calculate position error
    state = p.getLinkState(arm.getUUid(), numJoints - 1)
    # pos = state[0]
    # orn = state[1]
    u_task[:3] = -kp * (state[0] - target[:3])

    # Method 1 ------------------------------------------------------------
    #
    # transform the orientation target into a quaternion
    q_d = transformations.unit_vector(
        transformations.quaternion_from_euler(
            target[3], target[4], target[5], axes='rxyz'))

    # get the quaternion for the end effector
    q_e = transformations.unit_vector(
        transformations.quaternion_from_matrix(
            robot_config.R('EE', feedback['q'])))
    # calculate the rotation from the end-effector to target orientation
    q_r = transformations.quaternion_multiply(
        q_d, transformations.quaternion_conjugate(q_e))
    u_task[3:] = q_r[1:] * np.sign(q_r[0])

    # # # arm.getInertiaMatrix()

    # # obs = arm.getObservation()
    # # q = []
    # # qd = []
    # # for i in range(len(obs)):
    # #     q.append(obs[i][0])
    # #     qd.append(obs[i][1])
    # # q = np.array(q)
    # # qd = np.array(qd)

    # # kp = 10.0
    # # kd = 0.5

    # # cmd = kp * (q_des - q) - kd * qd

    # cmd = tau

    # # GRAV COMP
    # cmd = ctrl.compute(arm.getUUid(), arm.getMotorIndices())
    # # JOINT PD
   

    # TODO REALTIME ? How to do this? PD Controller Example?
    # time.sleep(0.01)
    # p.setJointMotorControlArray(arm.getUUid(),jointIndices=[1,2,3,4,5,6,7],controlMode=p.PD_CONTROL,targetPositions=[0,0,0,0,0,0,0])
    # p.setJointMotorControlArray(arm.getUUid(),jointIndices=arm.getMotorIndices(),controlMode=p.TORQUE_CONTROL,forces=c)


    p.stepSimulation()
    
    count += 1
    # time.sleep(1/500) # TODO DLW

    # pass
# QUELLEN
# https://github.com/studywolf/blog/blob/master/orientation/position_and_orientation_control.py