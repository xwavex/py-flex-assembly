#!/usr/bin/python

"""This is a basic test that loads a custom ground plane and instantiates a KUKAIIWA7 robot object, repositions it, and runs the simulation with a frequency of 0.001 ms. Since it uses the GUI_SERVER option as connection, an OROCOS RTT component could take over the control of the spawned robot.
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

arm = KukaIIWA7()
p.resetBasePositionAndOrientation(arm.getUUid(), [1, 0, 0.07], [0.0, 0.0, 0.0, 1.0])

p.setRealTimeSimulation(0)

p.setTimeStep(0.001)
while 1:
    p.stepSimulation()
