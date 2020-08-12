#!/usr/bin/python

"""This is just a sandbox test of bullet.
:Author:
  `Dennis Leroy Wigand <dwigand@cor-lab.de>`
"""

import pybullet as p
import time
import numpy as np
import math

from gym_flexassembly.robots import KukaIIWA7

from gym_flexassembly.controller import JointGravityCompensationController, JointPDController

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data
print("flexassembly_data.getDataPath() = " + str(flexassembly_data.getDataPath()))

client = p.connect(p.GUI_SERVER)
p.setGravity(0, 0, -9.81, physicsClientId=client)

p.setAdditionalSearchPath(flexassembly_data.getDataPath())

planeId = p.loadURDF("objects/plane_solid.urdf")

schunk = p.loadURDF("robots/schunk-egp-40/model.urdf", useFixedBase=True)
p.resetBasePositionAndOrientation(schunk, [1, 1, 1], [1,0,0,0])

arm = KukaIIWA7()

# Default Joint Positions
jointPositions = [
    0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
    -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
]
# Default Parameters
maxForce = 200.

endEffectorPos = [0.537, 0.0, 0.5]
endEffectorAngle = 0

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

# Because an SDF is assumed to hold multiple objects
# objects = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
# kuka_gripper = objects[0]
# print("objects = " + str(objects))
# kuka_gripper = hand
# kuka_gripper = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
# print("kuka_gripper id = " + str(kuka_gripper))
#
# p.resetBasePositionAndOrientation(kuka_gripper, [0.923103, -0.200000, 1.250036],
#                                   [-0.000000, 0.964531, -0.000002, -0.263970])
# jointPositions = [
#     0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000
# ]
# for jointIndex in range(p.getNumJoints(kuka_gripper)):
#   p.resetJointState(kuka_gripper, jointIndex, jointPositions[jointIndex])
#   p.setJointMotorControl2(kuka_gripper, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex],
#                           0)
#
# kuka_cid = p.createConstraint(arm, 6, kuka_gripper, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05],
#                               [0, 0, 0])

# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/debugDrawItems.py
# p.addUserDebugText("tip", [0, 0, 0.1],
#                    textColorRGB=[1, 0, 0],
#                    textSize=1.5,
#                    parentObjectUniqueId=arm,
#                    parentLinkIndex=6)
# p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=arm, parentLinkIndex=6)
# p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=arm, parentLinkIndex=6)
# p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=arm, parentLinkIndex=6)
p.setRealTimeSimulation(0)

ctrl = JointGravityCompensationController(p)
ctrl_j_pd = JointPDController(p)

# p.setJointMotorControlArray(arm.getUUid()=arm,jointIndices=[1,2,3,4,5,6,7],controlMode=p.PD_CONTROL,targetPositions=[0,1.2,0,1.2,0,0,0],positionGains=[500,500,500,500,500,500,500],velocityGains=[100,100,100,100,100,100,100])


arm.setControlMode("JOINT_TORQUE_CONTROL")
# print("arm.getUUid() = " + str(arm.getUUid()))

p.setTimeStep(0.001) # TODO DLW

# p.setRealTimeSimulation(1)

print("arm.getMotorIndices() = " + str(arm.getMotorIndices()))

while 1:
    joint_pos_0 = p.readUserDebugParameter(dp_joint_pos_0)
    joint_pos_1 = p.readUserDebugParameter(dp_joint_pos_1)
    joint_pos_2 = p.readUserDebugParameter(dp_joint_pos_2)
    joint_pos_3 = p.readUserDebugParameter(dp_joint_pos_3)
    joint_pos_4 = p.readUserDebugParameter(dp_joint_pos_4)
    joint_pos_5 = p.readUserDebugParameter(dp_joint_pos_5)
    joint_pos_6 = p.readUserDebugParameter(dp_joint_pos_6)

    # # # q_des = np.array([joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6])

    # # drawInertiaBox(arm.getUUid(), -1, [1, 0, 0])
    # # posX = p.readUserDebugParameter(dp_posX)
    # # posY = p.readUserDebugParameter(dp_posY)
    # # posZ = p.readUserDebugParameter(dp_posZ)
    # # yaw = p.readUserDebugParameter(dp_yaw)
    # # fingerAngle = p.readUserDebugParameter(dp_fingerAngle)
    # # for i in range(7):
    # #     drawInertiaBox(arm.getUUid(), i, [0, 1, 0])

    numJoints = len(arm.getMotorIndices())
    # p.getNumJoints(arm.getUUid())
    jointStates = p.getJointStates(arm.getUUid(), arm.getMotorIndices())
    q1 = []
    qdot1 = []
    zeroAccelerations = []
    for i in range(numJoints):
        # print('i ' + str(i))
        # print('index ' + str(arm.getMotorIndices()[i]))
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

    # timeStep = 0.1

    M1 = p.calculateMassMatrix(arm.getUUid(), q1)
    M2 = np.array(M1)
    # print(M2)
    # M = (M2 + Kd * timeStep)
    # print("POS = " + str(q1))
    # print("VEL = " + str(qdot1))
    c1 = p.calculateInverseDynamics(arm.getUUid(), q1, qdot1, zeroAccelerations)
    # print("GRA = " + str(c1))
    c = np.array(c1)
    # b = -c + pp + dd
    # qddot = np.linalg.solve(M, b)
    # tau = pp + dd - Kd.dot(qddot) * timeStep
    # # tau = c
    # # maxF = np.array(1000.0)
    # # forces = np.clip(tau, -maxF, maxF)

    # cmd = tau


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
    

    time.sleep(1/1000) # TODO DLW

    # pass

# # Torque control
# https://pybullet.org/Bullet/phpB3/viewtopic.php?t=12644
# pybullet.setJointMotorControl2(objUid, linkIndex, p.VELOCITY_CONTROL, force=0)

# # Disable the motors first
# Disable the motors for torque control:
# bullet.setJointMotorControlArray(id_robot,
#                                  id_revolute_joints,
#                                  bullet.VELOCITY_CONTROL,
#                                  forces=[0.0, 0.0])
# # Use Torque control in the loop
# # Set the Joint Torques:
# bullet.setJointMotorControlArray(id_robot,
#                                id_revolute_joints,
#                                bullet.TORQUE_CONTROL,
#                                forces=[torque[0], torque[1]])
# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_dynamics.py


# # Panda Gripper Gear Constraint
# #create a constraint to keep the fingers centered
# c = bullet_client.createConstraint(panda,
#                    9,
#                    panda,
#                    10,
#                    jointType=bullet_client.JOINT_GEAR,
#                    jointAxis=[1, 0, 0],
#                    parentFramePosition=[0, 0, 0],
#                    childFramePosition=[0, 0, 0])
# bullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
# https://github.com/bulletphysics/bullet3/blob/6428ed5e5f0641321dfbffe29b32a7fd423ca9c2/examples/pybullet/gym/pybullet_robots/panda/panda_sim_grasp.py#L46

# # # Convert Obj to SDF?
# https://github.com/bulletphysics/bullet3/blob/master/Extras/obj2sdf/obj2sdf.cpp