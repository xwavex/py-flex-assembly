import os, inspect

import pybullet as p
import time
import math
import numpy as np

import gym_flexassembly.data as data

from gym_flexassembly.constraints import frame


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

# p.connect(p.GUI)
p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
#don't create a ground plane, to allow for gaps etc
# p.resetSimulation()
#p.createCollisionShape(p.GEOM_PLANE)
#p.createMultiBody(0,0)
#p.resetDebugVisualizerCamera(5,75,-26,[0,0,1]);
# p.resetDebugVisualizerCamera(15, -346, -16, [-15, 0, 1])

p.setGravity(0,0,-9.81)

import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# table_squareId = p.loadURDF("table_square/table_square.urdf")
teddy = p.loadURDF("teddy_vhacd.urdf")
p.resetBasePositionAndOrientation(teddy, [0, 0.2, 1], [0,0,0,1])



first_frame1 = frame.Frame(p, "first_frame1")
first_frame1.resetPositionAndOrientation([1, 0, 3], [0,0,0,1])
# first_frame1.setVisibility(0,False)
# first_frame1.setVisibility(1,True)
# first_frame1.setVisibility(2,True)
# first_frame1.setVisibility(3,True)
# first_frame1.setVisibility(4,True)
# first_frame1.setVisibility(5,False)

first_frame2 = frame.Frame(p, "first_frame2")
first_frame2.resetPositionAndOrientation([1, 1, 3], [0,0,0,1])

first_frame3 = frame.Frame(p, "first_frame3")
first_frame3.resetPositionAndOrientation([1, 1, 2], [1,0,0,0])

# objects = p.loadMJCF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/MPL/skybox.xml")

schunk = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/schunk-egp-40/model.urdf", useFixedBase=False, flags = p.URDF_USE_INERTIA_FROM_FILE)
p.resetBasePositionAndOrientation(schunk, [0, 0, 1], [0,0,0,1])

base_text = p.addUserDebugText("base?", [0, 0, 0],
                   textColorRGB=[0, 0, 0],
                   textSize=1.0,
                   parentObjectUniqueId=schunk,
                   parentLinkIndex=3)

# base_text = p.addUserDebugText("bddddase?", [0, 0, 0],
#                    textColorRGB=[0, 0, 0],
#                    textSize=1.0,
#                    parentObjectUniqueId=schunk,
#                    parentLinkIndex=3,
#                    replaceItemUniqueId=base_text)

# p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [0.3, 0.3, 0.3], parentObjectUniqueId=schunk, parentLinkIndex=3)
# p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0.3, 0.3, 0.3], parentObjectUniqueId=schunk, parentLinkIndex=3)

p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/plane_solid.urdf", useMaximalCoordinates=True) # Brauche ich fuer die hit rays
#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# #disable tinyrenderer, software (CPU) renderer, we don't use it here
# p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

table = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/3d/table_profile_1.urdf", useFixedBase=True, flags = p.URDF_USE_INERTIA_FROM_FILE)
table_offset_world_x = -0.85
table_offset_world_y = 0
table_offset_world_z = 0
p.resetBasePositionAndOrientation(table, [table_offset_world_x, table_offset_world_y, table_offset_world_z], [0,0,0,1])

wood = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/3d/bend_wood.urdf", useFixedBase=False, flags = p.URDF_USE_INERTIA_FROM_FILE)
wood_offset_table_x = 0.88
wood_offset_table_y = 0.22
wood_offset_table_z = 0.73
p.resetBasePositionAndOrientation(wood, [table_offset_world_x + wood_offset_table_x, table_offset_world_y + wood_offset_table_y, table_offset_world_z + wood_offset_table_z], [0,0,0,1])



urdfRootPath = data.getDataPath()
# frame_ghost_id = p.loadURDF(os.path.join(urdfRootPath, "frame.urdf"), useFixedBase=True)
# p.resetBasePositionAndOrientation(frame_ghost_id, [-1, 1, 1],
#                                       [0.000000, 0.000000, 0.000000, 1.000000])
# # p.changeVisualShape(frame_ghost_id, -1, rgbaColor=[1, 1, 1, 0])
# p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
# p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
# p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
# p.addUserDebugText("frame_1", [0, 0.15, 0.15],
#                    textColorRGB=[0, 0, 0],
#                    textSize=1.0,
#                    parentObjectUniqueId=frame_ghost_id,
#                    parentLinkIndex=-1)
# p.addUserDebugLine([0, 0.05, 0.05], [0, 0.14, 0.14], [0, 0, 0], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)


# cidZ = p.createConstraint(schunk, -1, -1, -1, p.JOINT_POINT2POINT, [0, 0, 1], [0, 0, 0], [0, 0, 0])
# cidX = p.createConstraint(schunk, -1, -1, -1, p.JOINT_PRISMATIC, [1, 0, 0], [0, 0, 0], [0, 0, 0])


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

motor_indices = []
# DISABLE MOTORS
for j in range(p.getNumJoints(schunk)):
    ji = p.getJointInfo(schunk, j)
    jointType = ji[2]
    if (jointType == p.JOINT_SPHERICAL):
        targetPosition = [0, 0, 0, 1]
        p.setJointMotorControlMultiDof(schunk,j,p.POSITION_CONTROL,targetPosition, targetVelocity=[0,0,0], positionGain=0,velocityGain=1,force=[0,0,0])
    if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
        p.setJointMotorControl2(schunk,j,p.VELOCITY_CONTROL,targetVelocity=0, force=0)
    # 
    if ji[3] > -1:
        motor_indices.append(j)

# for j in range(p.getNumJoints(schunk)):
#     ji = p.getJointInfo(schunk, j)
    
#     targetPosition = [0]
#     jointType = ji[2]
#     if (jointType == p.JOINT_SPHERICAL):
#         targetPosition = [0, 0, 0, 1]
#         p.setJointMotorControlMultiDof(
#             schunk,
#             j,
#             p.TORQUE_CONTROL,
#             force=[0,0,0])
#     if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
#         p.setJointMotorControl2(schunk, j, p.TORQUE_CONTROL, targetVelocity=0, force=0)

p.setTimeStep(0.001)
# p.setRealTimeSimulation(1)

ux = p.addUserDebugLine([0, 0, 0], [0,0,0], [0.3, 0.3, 0.3], 2)
uy = p.addUserDebugLine([0, 0, 0], [0,0,0], [0.3, 0.3, 0.3], 2)
uz = p.addUserDebugLine([0, 0, 0], [0,0,0], [0.3, 0.3, 0.3], 2)
count_draw = 0


print(p.getDynamicsInfo(schunk, -1))
p.changeDynamics(schunk, 0, mass=0.0)
print(p.getDynamicsInfo(schunk, 0))
p.changeDynamics(schunk, 1, mass=0.0)
print(p.getDynamicsInfo(schunk, 1))
p.changeDynamics(schunk, 2, mass=0.0)
print(p.getDynamicsInfo(schunk, 2))
print(p.getDynamicsInfo(schunk, 3))
p.changeDynamics(schunk, 4, mass=0.0)
print(p.getDynamicsInfo(schunk, 4))
p.changeDynamics(schunk, 5, mass=0.0)
print(p.getDynamicsInfo(schunk, 5))

# print(p.getDynamicsInfo(schunk, -1))
while (1):
    numJoints = p.getNumJoints(schunk)
    sphere_js = p.getJointStateMultiDof(schunk, 3)
    euler = p.getEulerFromQuaternion([sphere_js[0][0], sphere_js[0][1], sphere_js[0][2], sphere_js[0][3]])

    # print(motor_indices)
    jointStates = p.getJointStates(schunk, motor_indices)
    q=   [jointStates[0][0],jointStates[1][0],jointStates[2][0], euler[0],euler[1],euler[2]                      , jointStates[4][0], jointStates[5][0]]
    qdot=[jointStates[0][1],jointStates[1][1],jointStates[2][1], sphere_js[1][0],sphere_js[1][1],sphere_js[1][2] , jointStates[4][1], jointStates[5][1]]
    zeroAccelerations=[0,0,0, 0,0,0 ,0,0]
    # for j in range(numJoints):
    #     q.append(jointStates[j][0])
    #     qdot.append(jointStates[j][1])
    #     zeroAccelerations.append(0)

    # pos, orn = p.getBasePositionAndOrientation(schunk)


    c = np.array(p.calculateInverseDynamics(schunk, q, qdot, zeroAccelerations))
    print(c)
    
    p.setJointMotorControl2(schunk,0,p.TORQUE_CONTROL, force=c[0])
    p.setJointMotorControl2(schunk,1,p.TORQUE_CONTROL, force=c[1])
    p.setJointMotorControl2(schunk,2,p.TORQUE_CONTROL, force=c[2])

    # p.setJointMotorControl2(schunk,1,p.TORQUE_CONTROL, force=0)
    # p.setJointMotorControl2(schunk,2,p.TORQUE_CONTROL, force=0)

    p.setJointMotorControlMultiDof(schunk,3,p.TORQUE_CONTROL,force=[c[3],c[4],c[5]])

    # print(sphere_js)
    if count_draw % 50 == 0:
        ret = p.getLinkState(schunk, 3) 
        # position_linkcom_world, world_rotation_linkcom, position_linkcom_frame, frame_rotation_linkcom, position_frame_world, world_rotation_frame, linearVelocity_linkcom_world, angularVelocity_linkcom_world
        # p.addUserDebugLine([0, 0, 0], ret[0], [0.3, 0.3, 0.3], 3, 0.02)
        ux = p.addUserDebugLine([0, 0, 0.01], [ret[0][0], 0, 0.01], [0.3, 0.3, 0.3], 3, replaceItemUniqueId=ux)
        uy = p.addUserDebugLine([ret[0][0], 0, 0.01], [ret[0][0], ret[0][1], 0.01], [0.3, 0.3, 0.3], 3, replaceItemUniqueId=uy)
        uz = p.addUserDebugLine([ret[0][0], ret[0][1], 0.01], [ret[0][0], ret[0][1], ret[0][2]], [0.3, 0.3, 0.3], 3, replaceItemUniqueId=uz)
        count_draw = 0

    # camData = p.getDebugVisualizerCamera()
    # viewMat = camData[2]
    # projMat = camData[3]
    # p.getCameraImage(256,
    #                 256,
    #                 viewMatrix=viewMat,
    #                 projectionMatrix=projMat,
    #                 renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # keys = p.getKeyboardEvents()
    # mouseEvents = p.getMouseEvents()
    # for e in mouseEvents:
    #     if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
    #         mouseX = e[1]
    #         mouseY = e[2]
    #         rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
    #         rayInfo = p.rayTest(rayFrom, rayTo)
    #         print("rayInfo = " + str(rayInfo))
    #         # p.addUserDebugLine(rayFrom,rayTo,[1,0,0],3)
    #         for l in range(len(rayInfo)):
    #             hit = rayInfo[l]
    #             objectUid = hit[0]
    #             print(objectUid)
    #             if (objectUid >= 1):
    #                 print("yesss")
    #                 # p.changeVisualShape(objectUid, -1, rgbaColor=colors[currentColor])
    p.stepSimulation()
    time.sleep(1/1000) # TODO DLW
    count_draw = count_draw + 1


# import pybullet as p
# import time

# cid = p.connect(p.SHARED_MEMORY)
# if (cid < 0):
#   p.connect(p.GUI)
# p.loadURDF("plane.urdf")
# kuka = p.loadURDF("kuka_iiwa/model.urdf")
# p.addUserDebugText("tip", [0, 0, 0.1],
#                    textColorRGB=[1, 0, 0],
#                    textSize=1.5,
#                    parentObjectUniqueId=kuka,
#                    parentLinkIndex=6)
# p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=kuka, parentLinkIndex=6)
# p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=kuka, parentLinkIndex=6)
# p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=kuka, parentLinkIndex=6)
# p.setRealTimeSimulation(0)
# angle = 0
# while (True):
#   time.sleep(0.01)
#   p.resetJointState(kuka, 2, angle)
#   p.resetJointState(kuka, 3, angle)
#   angle += 0.01
