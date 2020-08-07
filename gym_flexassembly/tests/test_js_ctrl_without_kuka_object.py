#!/usr/bin/python

"""This is a test for the custom joint space controller implemented in gym_flexassembly.controller. It does not use the KUKAIIWA7 object, instead it load the robot model and configures it directly in this file.
:Author:
  `Dennis Leroy Wigand <dwigand@cor-lab.de>`
"""

import pybullet as p
import time
import numpy as np
import math
import os

from gym_flexassembly.robots import KukaIIWA7

from gym_flexassembly import data as flexassembly_data

from gym_flexassembly.controller import JointGravityCompensationController, JointPDController, OperationalSpaceController, JointPDControllerSimple

# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI_SERVER)
p.setGravity(0, 0, -9.81, physicsClientId=client)

import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")

# arm = KukaIIWA7()
# p.resetBasePositionAndOrientation(arm.getUUid(), [2, 2, 0.5], [0,0,0,1])
# for indice in arm.getMotorIndices():
#     print(indice)

kukaUid = p.loadURDF(os.path.join(flexassembly_data.getDataPath(), "robots/kuka-iiwa-7/model.urdf"), useFixedBase=True)
p.resetBasePositionAndOrientation(kukaUid, [0, -0.2, 0.5], [0,0,0,1])
numJoints = p.getNumJoints(kukaUid)
allJoint = []
zero = []
for jon in range(numJoints):
    allJoint.append(jon)
    zero.append(0.0)
p.setJointMotorControlArray(kukaUid, allJoint, p.VELOCITY_CONTROL, forces=zero)

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
# # OPERATIONAL SPACE CONTROLLER
ctrl_osc = OperationalSpaceController(
    p,
    kp=50,
    # null_controllers=[damping],
    # control (x, y, gamma) out of [x, y, z, alpha, beta, gamma]
    # ctrlr_dof=[True, True, False, False, False, True],
)

ctrl_j_pd_s = JointPDControllerSimple(p)

# arm.setControlMode("JOINT_TORQUE_CONTROL")
p.setTimeStep(0.001) # TODO DLW
while 1:
    joint_pos_0 = p.readUserDebugParameter(dp_joint_pos_0)
    joint_pos_1 = p.readUserDebugParameter(dp_joint_pos_1)
    joint_pos_2 = p.readUserDebugParameter(dp_joint_pos_2)
    joint_pos_3 = p.readUserDebugParameter(dp_joint_pos_3)
    joint_pos_4 = p.readUserDebugParameter(dp_joint_pos_4)
    joint_pos_5 = p.readUserDebugParameter(dp_joint_pos_5)
    joint_pos_6 = p.readUserDebugParameter(dp_joint_pos_6)

    # GRAV COMP
    # cmd = ctrl.compute(arm.getUUid(), arm.getMotorIndices())
    # # JOINT PD
    desiredPositions = [joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6]
    desiredVelocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    kps = [600,600,600,600,600,600,600]
    kds = [200,200,200,200,200,200,200]
    # kps = [100,100,100,100,100,100,100]
    # kds = [10,10,10,10,10,10,10]
    maxForces = 10000000
    # timeStep = 1
    cmd = ctrl_j_pd.compute(kukaUid, [1,2,3,4,5,6,7], desiredPositions, desiredVelocities, kps, kds,
                maxForces)
    # # CART PD
    # # OSC
    # cmd = ctrl_osc.generate(q=feedback["q"], dq=feedback["dq"], target=target,)

    # desiredPositions = [joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5]
    # desiredVelocities = [0.0,0.0,0.0,0.0,0.0,0.0]
    # cmd = ctrl_osc.compute(arm.getUUid(), arm.getMotorIndices(), desiredPositions, desiredVelocities)

    # arm.setCommand(cmd)

    p.setJointMotorControlArray(kukaUid,
                                        [1,2,3,4,5,6,7],
                                        p.TORQUE_CONTROL,
                                        forces=cmd)

    p.stepSimulation()

    time.sleep(1/1000) # TODO DLW
