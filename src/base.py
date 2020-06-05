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

client = p.connect(p.GUI_SERVER)
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
p.resetBasePositionAndOrientation(arm.getUUid(), [1, 0, 0.07],
                                      [0.0, 0.0, 0.0, 1.0])

p.setRealTimeSimulation(0)

p.setTimeStep(0.001) # TODO DLW
# while 1:
#     p.stepSimulation()
