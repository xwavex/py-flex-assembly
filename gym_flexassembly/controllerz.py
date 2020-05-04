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

# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client)

import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")

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

p.setRealTimeSimulation(0)

# # GRAV COMP
ctrl = JointGravityCompensationController(p)
# # JOINT PD
ctrl_j_pd = JointPDController(p)

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

    # # GRAV COMP
    # cmd = ctrl.compute(arm.getUUid(), arm.getMotorIndices())
    # # JOINT PD
    # desiredPositions = [joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6]
    # desiredVelocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    # kps = [600,600,600,600,600,600,600]
    # kds = [200,200,200,200,200,200,200]
    # maxForces = 10000000
    # timeStep = 1
    # cmd = ctrl_j_pd.compute(arm.getUUid(), arm.getMotorIndices(), desiredPositions, desiredVelocities, kps, kds,
    #             maxForces)
    # # CART PD
    desiredPositions = [joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6]
    desiredVelocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    arm.setCommand(cmd)

    p.stepSimulation()

    # time.sleep(1/500) # TODO DLW
