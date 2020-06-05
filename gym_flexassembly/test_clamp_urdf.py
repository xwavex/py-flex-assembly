#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3

import os, inspect

import pybullet as p
import time
import math
import numpy as np
from pyquaternion import Quaternion

import trimesh

import signal

import sys
print(sys.path)


# Open server and changing background color in visualizer
p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')

################################
### Simulation configuration ###
################################

# Setting gravity
p.setGravity(0,0,-9.81)

p.setTimeStep(0.001)
# p.setRealTimeSimulation(1)

#########################################################
### Setting up the environment for the polishing task ###
#########################################################

# Getting pybullet assets
import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Disable rendering
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# Floor SHOULD BE ALWAYS ID 0
p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/catkin_py_ws/src/py-flex-assembly/gym_flexassembly/data/plane_solid.urdf", useMaximalCoordinates=True) # Brauche ich fuer die hit rays

# Enable rendering again
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

################################
### CLAMP MOCK ###
################################

w_qs_1 = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/catkin_py_ws/src/py-flex-assembly/gym_flexassembly/data/W_QS_1/W_QS_1.urdf", useFixedBase=False)
p.resetBasePositionAndOrientation(w_qs_1, [0,0,0.037],[0,0,0,1])
max_force_for_clip = 10.0
p.setJointMotorControl2(bodyIndex=w_qs_1, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=0, targetVelocity=0, force=max_force_for_clip)

# cid = p.createConstraint(parentBodyUniqueId=w_qs_1, parentLinkIndex=1, childBodyUniqueId=w_qs_1, childLinkIndex=2, jointType=p.JOINT_POINT2POINT, jointAxis=[0, 0, 0], parentFramePosition=[0, 0.0042, 0.0025], childFramePosition=[0, 0, -0.0035070000000000006])
# cid = p.createConstraint(parentBodyUniqueId=w_qs_1, parentLinkIndex=-1, childBodyUniqueId=-1, childLinkIndex=-1, jointType=p.JOINT_POINT2POINT, jointAxis=[0, 0, 0], parentFramePosition=[0, 0.0042, 0.0025], childFramePosition=[0, 0, 1])
# cid = p.createConstraint(parentBodyUniqueId=w_qs_1, parentLinkIndex=-1,
#                         childBodyUniqueId=-1, childLinkIndex=-1,
#                         jointType=p.JOINT_POINT2POINT,
#                         jointAxis=[1, 0, 0],
#                         parentFramePosition=[0, 0, 0],
#                         childFramePosition=[0, 0, 1])
# p.changeConstraint(userConstraintUniqueId=cid, gearRatio=1, maxForce=10000)

# for i in range(p.getNumJoints(w_qs_1)):
#     p.setJointMotorControl2(bodyIndex=w_qs_1,
#                             jointIndex=i,
#                             controlMode=p.VELOCITY_CONTROL,
#                             force=0)

p.stepSimulation()

while (1):
    # p.changeConstraint(userConstraintUniqueId=cid, maxForce=5000, jointChildFrameOrientation=[0,0,0,1])
    p.setJointMotorControl2(bodyIndex=w_qs_1, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=0, targetVelocity=0, force=max_force_for_clip)
    p.stepSimulation()
    time.sleep(1/1000) # TODO DLW

try:
    signal.pause()
except (KeyboardInterrupt, SystemExit):
    print("Shutting down...")