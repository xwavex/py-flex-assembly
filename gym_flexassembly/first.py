#!/usr/bin/python

import pybullet as p
import time
import numpy as np
import math

try:
    from .robots import KukaIIWA7
except (ImportError, SystemError):
    from robots import KukaIIWA7

try:
    from .controller import JointGravityCompensationController, JointPDController
except (ImportError, SystemError):
    from controller import JointGravityCompensationController, JointPDController

def drawInertiaBox(parentUid, parentLinkIndex, color):
  dyn = p.getDynamicsInfo(parentUid, parentLinkIndex)
  mass = dyn[0]
  frictionCoeff = dyn[1]
  inertia = dyn[2]
  if (mass > 0):
    Ixx = inertia[0]
    Iyy = inertia[1]
    Izz = inertia[2]
    boxScaleX = 0.5 * math.sqrt(6 * (Izz + Iyy - Ixx) / mass)
    boxScaleY = 0.5 * math.sqrt(6 * (Izz + Ixx - Iyy) / mass)
    boxScaleZ = 0.5 * math.sqrt(6 * (Ixx + Iyy - Izz) / mass)

    halfExtents = [boxScaleX, boxScaleY, boxScaleZ]
    pts = [[halfExtents[0], halfExtents[1], halfExtents[2]],
           [-halfExtents[0], halfExtents[1], halfExtents[2]],
           [halfExtents[0], -halfExtents[1], halfExtents[2]],
           [-halfExtents[0], -halfExtents[1], halfExtents[2]],
           [halfExtents[0], halfExtents[1], -halfExtents[2]],
           [-halfExtents[0], halfExtents[1], -halfExtents[2]],
           [halfExtents[0], -halfExtents[1], -halfExtents[2]],
           [-halfExtents[0], -halfExtents[1], -halfExtents[2]]]

    p.addUserDebugLine(pts[0],
                       pts[1],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[1],
                       pts[3],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[3],
                       pts[2],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[2],
                       pts[0],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)

    p.addUserDebugLine(pts[0],
                       pts[4],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[1],
                       pts[5],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[2],
                       pts[6],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[3],
                       pts[7],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)

    p.addUserDebugLine(pts[4 + 0],
                       pts[4 + 1],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[4 + 1],
                       pts[4 + 3],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[4 + 3],
                       pts[4 + 2],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)
    p.addUserDebugLine(pts[4 + 2],
                       pts[4 + 0],
                       color,
                       1,
                       parentObjectUniqueId=parentUid,
                       parentLinkIndex=parentLinkIndex)

# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client)

# import data as gfd
# p.setAdditionalSearchPath(gfd.getDataPath())
# p.setAdditionalSearchPath('/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data')

import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
# p.createCollisionShape(p.GEOM_PLANE)
# p.createMultiBody(0,0)

# scene_ele1 = p.loadSDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/pybullettest/models/klemme1/model.sdf")

# terminal_case = p.loadSDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/pybullettest/models/terminal-case-world/terminal-case-world.sdf")
# print("terminal_case = " + str(terminal_case))
# obj_terminal_case = terminal_case[0]

# test_link = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/test.urdf")
# test_jenga = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/test_link/test_link.urdf", 0,0,0.2, 0.000000, 0.707107, 0.000000, 0.707107)

# hand = p.loadURDF("/home/dwigand/citk/systems/cogimon-minimal-nightly/share/gazebo/models/cogimon/schunk-egp-40/model.urdf",[1.0, 1.0, 0], useFixedBase=False)
# for i in range(p.getNumJoints(hand)):
#   p.setJointMotorControl2(hand, i, p.VELOCITY_CONTROL, force=0)

# arm = p.loadURDF("/home/dwigand/citk/systems/cogimon-minimal-nightly/share/gazebo/models/cogimon/kuka-iiwa-7-schunk-egp-40/model.urdf",[0, 0, 0], useFixedBase=True)
# arm = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/kuka-iiwa-7/model.urdf",[0, 0.3, 0], useFixedBase=True)

arm = KukaIIWA7()

# # arm = p.loadURDF("kuka_iiwa/model.urdf",[0, 0, 0], useFixedBase=True)

# number_of_joints = p.getNumJoints(arm)
# for joint_number in range(number_of_joints):
#     info = p.getJointInfo(arm, joint_number)
#     print(info)

# # Change position and orientation
# p.resetBasePositionAndOrientation(arm, [-0.100000, 0.000000, 0.070000], [0.000000, 0.000000, 0.000000, 1.000000])

# Default Joint Positions
jointPositions = [
    0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
    -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
]
# Default Parameters
maxForce = 200.

# numJoints = p.getNumJoints(arm)
# for jointIndex in range(numJoints):
#     p.resetJointState(arm, jointIndex, jointPositions[jointIndex])
#     p.setJointMotorControl2(arm,
#                       jointIndex,
#                       p.POSITION_CONTROL,
#                       targetPosition=jointPositions[jointIndex],
#                       force=maxForce)


endEffectorPos = [0.537, 0.0, 0.5]
endEffectorAngle = 0

motorNames = []
motorIndices = []

# for i in range(numJoints):
#     jointInfo = p.getJointInfo(arm, i)
#     qIndex = jointInfo[3]
#     if qIndex > -1:
#         print("motorname = " + str(jointInfo[1]))
#         motorNames.append(str(jointInfo[1]))
#         motorIndices.append(i)




#############################################################

# def step(self):
#     t = self.t
#     self.t += 1./60.
#     pos = [self.offset[0]+0.2 * math.sin(1.5 * t), self.offset[1]+0.044, self.offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)]
#     orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
#     jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul,
#       jr, rp, maxNumIterations=5)
#     for i in range(pandaNumDofs):
#         self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
#     pass

# # Torque Control and Inverse Dynamics
# # https://github.com/bulletphysics/bullet3/blob/aec9968e281faca7bc56bc05ccaf0ef29d82d062/examples/pybullet/examples/inverse_dynamics.py

# # Controller with MassMatrix
# # https://github.com/bulletphysics/bullet3/blob/aec9968e281faca7bc56bc05ccaf0ef29d82d062/examples/pybullet/examples/pdControllerStable.py
# # https://github.com/bulletphysics/bullet3/blob/0aaae872451a69d0c93b0c8ed818667de4ad5653/examples/pybullet/gym/pybullet_utils/pd_controller_stable.py
# # # mit computeAngVelRel
# dyn = p.getDynamicsInfo(quadruped, -1)
# mass = dyn[0]
# friction = dyn[1]
# localInertiaDiagonal = dyn[2]

# # # Interesting for dynamically jointing urdfs https://github.com/bulletphysics/bullet3/blob/aec9968e281faca7bc56bc05ccaf0ef29d82d062/examples/pybullet/gym/pybullet_utils/examples/combineUrdf.py


# GUI Debug Input
# dv = 0.6
# dp_posX = p.addUserDebugParameter("posX", -dv, dv, 0)
# dp_posY = p.addUserDebugParameter("posY", -dv, dv, 0)
# dp_posZ = p.addUserDebugParameter("posZ", -dv, dv, 0)
# dp_yaw = p.addUserDebugParameter("yaw", -dv, dv, 0)
# dp_fingerAngle = p.addUserDebugParameter("fingerAngle", 0, 0.3, .3)

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
p.setTimeStep(0.001) # TODO DLW
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

    # numJoints = len(arm.getMotorIndices())
    # # p.getNumJoints(arm.getUUid())
    # jointStates = p.getJointStates(arm.getUUid(), arm.getMotorIndices())
    # q1 = []
    # qdot1 = []
    # zeroAccelerations = []
    # for i in range(numJoints):
    #     # print('i ' + str(i))
    #     # print('index ' + str(arm.getMotorIndices()[i]))
    #     q1.append(jointStates[i][0])
    #     qdot1.append(jointStates[i][1])
    #     zeroAccelerations.append(0)
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

    # M1 = p.calculateMassMatrix(arm.getUUid(), q1)
    # M2 = np.array(M1)
    # M = (M2 + Kd * timeStep)
    # c1 = p.calculateInverseDynamics(arm.getUUid(), q1, qdot1, zeroAccelerations)
    # c = np.array(c1)
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
    desiredPositions = [joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6]
    desiredVelocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    kps = [600,600,600,600,600,600,600]
    kds = [200,200,200,200,200,200,200]
    # kps = [0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    # kds = [0.05,0.05,0.05,0.05,0.05,0.05,0.05]
    maxForces = 10000000
    timeStep = 1
    cmd = ctrl_j_pd.compute(arm.getUUid(), arm.getMotorIndices(), desiredPositions, desiredVelocities, kps, kds,
                maxForces)

    arm.setCommand(cmd)

    # TODO REALTIME ? How to do this? PD Controller Example?
    # time.sleep(0.01)
    # p.setJointMotorControlArray(arm.getUUid()=arm,jointIndices=[1,2,3,4,5,6,7],controlMode=p.PD_CONTROL,targetPositions=[0,0,0,0,0,0,0])
    p.stepSimulation()

    # time.sleep(1/500) # TODO DLW
    
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