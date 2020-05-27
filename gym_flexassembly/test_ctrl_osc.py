#!/usr/bin/python

import pybullet as p
import time
import numpy as np
import math

def getRayFromTo(mouseX, mouseY):
    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
    )
    camPos = [
        camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
        camTarget[2] - dist * camForward[2]
    ]
    farPlane = 10000
    rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
    invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] *
                                        rayForward[1] + rayForward[2] * rayForward[2]))
    rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
    rayFrom = camPos
    oneOverWidth = float(1) / float(width)
    oneOverHeight = float(1) / float(height)
    dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
    dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
    rayToCenter = [
        rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
    ]
    rayTo = [
        rayToCenter[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
        float(mouseY) * dVer[0], rayToCenter[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
        float(mouseX) * dHor[1] - float(mouseY) * dVer[1], rayToCenter[2] - 0.5 * horizon[2] +
        0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
    ]
    return rayFrom, rayTo



try:
    from .robots import KukaIIWA7
except (ImportError, SystemError):
    from robots import KukaIIWA7

try:
    from .controller import JointGravityCompensationController, JointPDController, OperationalSpaceController
except (ImportError, SystemError):
    from controller import JointGravityCompensationController, JointPDController, OperationalSpaceController

import data
urdfRootPath = data.getDataPath()

import os

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
dp_joint_pos_x = p.addUserDebugParameter("x", -dv, dv, 0.526)
dp_joint_pos_y = p.addUserDebugParameter("y", -dv, dv, 0.0)
dp_joint_pos_z = p.addUserDebugParameter("z", -dv, dv, 0.463)
dp_joint_pos_rr = p.addUserDebugParameter("rr", -6, 6, 0)
dp_joint_pos_rp = p.addUserDebugParameter("rp", -6, 6, 3.14)
dp_joint_pos_ry = p.addUserDebugParameter("ry", -6, 6, 0)

dp_joint_pos_kp = p.addUserDebugParameter("kp", 0, 1000, 100)
dp_joint_pos_ko = p.addUserDebugParameter("ko", 0, 1000, 30)

p.setRealTimeSimulation(0)

# orientations = [
#     [0, 0, 0],
#     [np.pi/4, np.pi/4, np.pi/4],
#     [-np.pi/4, -np.pi/4, np.pi/2],
#     [0, 0, 0],
#     ]
# positions = [
#     [0.15, -0.1, 0.6],
#     [-.15, 0.0, .7],
#     [.2, .2, .6],
#     [0.15, -0.1, 0.6]
#     ]

# # GRAV COMP
# ctrl = JointGravityCompensationController(p)
# # # JOINT PD
# ctrl_j_pd = JointPDController(p)
# # OPERATIONAL SPACE CONTROLLER
ctrl_osc = OperationalSpaceController(
    p,
    kp=800,
    # null_controllers=[damping],
    # control (x, y, gamma) out of [x, y, z, alpha, beta, gamma]
    # ctrlr_dof=[True, True, False, False, False, True],
    ko=800,
    kv=None,
    ki=0,
    vmax=None,
    ctrlr_dof=None,
    null_controllers=None,
    use_g=True,
    use_C=True,
    orientation_algorithm=0
)

frame_ghost_id = p.loadURDF(os.path.join(urdfRootPath, "frame_ghost.urdf"), useFixedBase=True)
p.resetBasePositionAndOrientation(frame_ghost_id, [0, 0.5, 1],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
# p.changeVisualShape(frame_ghost_id, -1, rgbaColor=[1, 1, 1, 0])
p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
# p.addUserDebugText("tip", [0, 0, 0.1],
#                    textColorRGB=[1, 0, 0],
#                    textSize=1.5,
#                    parentObjectUniqueId=frame_ghost_id,
#                    parentLinkIndex=-1)
frame_ghost_id_2 = p.loadURDF(os.path.join(urdfRootPath, "frame_ghost.urdf"), useFixedBase=True)
p.resetBasePositionAndOrientation(frame_ghost_id_2, [0, 0.2, 1],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
# p.changeVisualShape(frame_ghost_id_2, -1, rgbaColor=[1, 1, 1, 0])
p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=frame_ghost_id_2, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=frame_ghost_id_2, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=frame_ghost_id_2, parentLinkIndex=-1)


p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=arm.getUUid(), parentLinkIndex=7)
p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=arm.getUUid(), parentLinkIndex=7)
p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=arm.getUUid(), parentLinkIndex=7)

arm.setControlMode("JOINT_TORQUE_CONTROL")
p.setTimeStep(0.001) # TODO DLW

# schunk = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/schunk-egp-40/model.urdf", useFixedBase=False)
# p.resetBasePositionAndOrientation(schunk, [1, 1, 1], [0,0,0,1])

# collisionFilterGroup = 0x10
# collisionFilterMask = 0x1
# for i in range(p.getNumJoints(arm.getUUid())):
#     p.setCollisionFilterGroupMask(arm.getUUid(), i, collisionFilterGroup, collisionFilterMask)

# p.setCollisionFilterGroupMask(arm.getUUid(), -1, collisionFilterGroup, collisionFilterMask)
# p.setCollisionFilterGroupMask(arm.getUUid(), 0, collisionFilterGroup, collisionFilterMask)
# p.setCollisionFilterGroupMask(arm.getUUid(), 1, collisionFilterGroup, collisionFilterMask)
# p.setCollisionFilterGroupMask(arm.getUUid(), 2, collisionFilterGroup, collisionFilterMask)
# p.setCollisionFilterGroupMask(arm.getUUid(), 3, collisionFilterGroup, collisionFilterMask)
# p.setCollisionFilterGroupMask(arm.getUUid(), 4, collisionFilterGroup, collisionFilterMask)
# p.setCollisionFilterGroupMask(arm.getUUid(), 5, collisionFilterGroup, collisionFilterMask)
# p.setCollisionFilterGroupMask(arm.getUUid(), 6, collisionFilterGroup, collisionFilterMask)

# Binary, ray only works on 0x0000000000001
collisionFilterGroup2 = 0x100
collisionFilterMask2 =  0x1
p.setCollisionFilterGroupMask(frame_ghost_id, -1, collisionFilterGroup2, collisionFilterMask2)

# Conclusion on p.setCollisionFilterGroupMask(id, (body_id) -1, collisionFilterGroup, collisionFilterMask):
# Raytest works with collisionFilterMask = 0x1 ONLY
# EVERYTHING is BINARY
# DIFFERENT GROUP IDs == NO COLLISION!
# However, groups need to be assigned at the beginning!

# enableCollision = 1
# p.setCollisionFilterPair(planeId, cubeId, -1, -1, enableCollision)

count = 0
while 1:
    cx = p.readUserDebugParameter(dp_joint_pos_x)
    cy = p.readUserDebugParameter(dp_joint_pos_y)
    cz = p.readUserDebugParameter(dp_joint_pos_z)
    crr = p.readUserDebugParameter(dp_joint_pos_rr)
    crp = p.readUserDebugParameter(dp_joint_pos_rp)
    cry = p.readUserDebugParameter(dp_joint_pos_ry)

    kkp = p.readUserDebugParameter(dp_joint_pos_kp)
    kko = p.readUserDebugParameter(dp_joint_pos_ko)

    desiredPositions = [cx,cy,cz,crr,crp,cry]
    desiredVelocities = [0.0,0.0,0.0,0.0,0.0,0.0]
    cmd = ctrl_osc.compute(arm.getUUid(), arm.getMotorIndices(), desiredPositions, desiredVelocities, kkp, kko)


    if (count % 10 == 0):
        count = 0
        state = p.getLinkState(arm.getUUid(), 7)
        pos = state[0]
        orn = state[1]
        orn_des_bullet = p.getQuaternionFromEuler([crr, crp, cry])
        p.resetBasePositionAndOrientation(frame_ghost_id, [0, 0, 0.7], orn_des_bullet)
        p.resetBasePositionAndOrientation(frame_ghost_id_2, [0, 0.2, 1], orn)

    arm.setCommand(cmd)
    mouseEvents = p.getMouseEvents()
    for e in mouseEvents:
        if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
            mouseX = e[1]
            mouseY = e[2]
            rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
            rayInfo = p.rayTestBatch([rayFrom], [rayTo])#, collisionFilterMask=1)
            # print("rayInfo = " + str(rayInfo))
            # p.addUserDebugLine(rayFrom,rayTo,[1,0,0],3)
            for l in range(len(rayInfo)):
                hit = rayInfo[l]
                objectUid = hit[0]
                print("rayInfo: " + str(l) + " = " + str(objectUid))
                # print(objectUid)
                if (objectUid == frame_ghost_id):
                    print("yesss")

    p.stepSimulation()
    count = count + 1

    # time.sleep(1/500) # TODO DLW
