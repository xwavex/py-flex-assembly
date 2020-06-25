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

# Create a constraint to keep the fingers centered
c = p.createConstraint(schunk,
                    0,
                    schunk,
                    1,
                    jointType=p.JOINT_GEAR,
                    jointAxis=[1, 0, 0],
                    parentFramePosition=[0, 0, 0],
                    childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

dp_joint_pos_0 = p.addUserDebugParameter("force Z", -10, 10, 9.81*0.6+2*0.04)

# kpp = p.addUserDebugParameter("kp", 0, 500, 200)
# kdd = p.addUserDebugParameter("kd", 0, 30, 10)
kpp = p.addUserDebugParameter("kp", 0, 500, 10)
kdd = p.addUserDebugParameter("kd", 0, 30, 0.1)

count = 0
while 1:
    joint_pos_0 = p.readUserDebugParameter(dp_joint_pos_0)
    kp = p.readUserDebugParameter(kpp)
    kd = p.readUserDebugParameter(kdd)

    pos, orn = p.getBasePositionAndOrientation(schunk)
    # _base_mass_urdf = p.getDynamicsInfo(schunk, -1)
    p.changeDynamics(schunk, -1, mass=0.6)
    print(p.getDynamicsInfo(schunk, -1))
    # print(_base_mass_urdf)
    # print(p.getDynamicsInfo(schunk, 0))
    # print(p.getDynamicsInfo(schunk, 1))
    vpos, vorn = p.getBaseVelocity(schunk)

    pError = kp*([0,0,0.5]-np.array(pos)) - kd*np.array(vpos)
    # pError[2] = 2*(0.5-np.array(pos)[2]) - 0.1*np.array(vpos)[2]
    pError = np.array(pError)
    pError = pError + np.array([0,0,joint_pos_0])

    p.applyExternalForce(schunk, -1, np.array(pError), pos, flags=p.WORLD_FRAME)



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
