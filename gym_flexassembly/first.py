#!/usr/bin/python

import pybullet as p
import time

try:
    from .robots import KukaIIWA7
except (ImportError, SystemError):
    from robots import KukaIIWA7


# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client)

# import data as gfd
# p.setAdditionalSearchPath(gfd.getDataPath())
# p.setAdditionalSearchPath('/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data')

import pybullet_data
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
dv = 0.6
dp_posX = p.addUserDebugParameter("posX", -dv, dv, 0)
dp_posY = p.addUserDebugParameter("posY", -dv, dv, 0)
dp_posZ = p.addUserDebugParameter("posZ", -dv, dv, 0)
dp_yaw = p.addUserDebugParameter("yaw", -dv, dv, 0)
dp_fingerAngle = p.addUserDebugParameter("fingerAngle", 0, 0.3, .3)

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

# p.setJointMotorControlArray(bodyUniqueId=arm,jointIndices=[1,2,3,4,5,6,7],controlMode=p.PD_CONTROL,targetPositions=[0,1.2,0,1.2,0,0,0],positionGains=[500,500,500,500,500,500,500],velocityGains=[100,100,100,100,100,100,100])
while 1:
    posX = p.readUserDebugParameter(dp_posX)
    posY = p.readUserDebugParameter(dp_posY)
    posZ = p.readUserDebugParameter(dp_posZ)
    yaw = p.readUserDebugParameter(dp_yaw)
    fingerAngle = p.readUserDebugParameter(dp_fingerAngle)
    # TODO REALTIME ? How to do this? PD Controller Example?
    # time.sleep(0.01)
    # p.setJointMotorControlArray(bodyUniqueId=arm,jointIndices=[1,2,3,4,5,6,7],controlMode=p.PD_CONTROL,targetPositions=[0,0,0,0,0,0,0])
    p.stepSimulation()
    # pass

# # Torque control
# https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12644
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