#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3

import os, inspect

import pybullet as p
import time
import math
import numpy as np
from pyquaternion import Quaternion

import data

from constraints import frame
from constraints import constraint_manager

import trimesh

import signal

import sys
print(sys.path)

try:
    from .robots import KukaIIWA7
except (ImportError, SystemError):
    from robots import KukaIIWA7


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
p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/plane_solid.urdf", useMaximalCoordinates=True) # Brauche ich fuer die hit rays

# Table
table_id = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/3d/table_profile_1.urdf", useFixedBase=True, flags = p.URDF_USE_INERTIA_FROM_FILE)
table_offset_world_x = -0.85
table_offset_world_y = 0
table_offset_world_z = 0
p.resetBasePositionAndOrientation(table_id, [table_offset_world_x, table_offset_world_y, table_offset_world_z], [0,0,0,1])

# Workpiece to be polished
workpiece_id = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/3d/bend_wood.urdf", useFixedBase=False, flags = p.URDF_USE_INERTIA_FROM_FILE)
wood_offset_table_x = 0.88
wood_offset_table_y = 0.22
wood_offset_table_z = 0.73
wood_offset_world = [table_offset_world_x + wood_offset_table_x, table_offset_world_y + wood_offset_table_y, table_offset_world_z + wood_offset_table_z]
p.resetBasePositionAndOrientation(workpiece_id, wood_offset_world, [0,0,0,1])

# Getting my data assets
# urdfRootPath = data.getDataPath()
# frame_ghost_id = p.loadURDF(os.path.join(urdfRootPath, "frame.urdf"), useFixedBase=True)

# Enable rendering again
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

################################
### CLAMP MOCK ###
################################

# Floor SHOULD BE ALWAYS ID 0
p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/W_QS_1/W_QS_1.urdf")

################################
### KUKA MOCK ###
################################

arm = KukaIIWA7()
p.resetBasePositionAndOrientation(arm.getUUid(), [0,-0.2,0.5], [0,0,0,1])
# arm.setControlMode("JOINT_TORQUE_CONTROL")
#                           0x010
collisionFilterGroup_kuka = 0x10
#                           0x001
collisionFilterMask_kuka = 0x1
for i in range(p.getNumJoints(arm.getUUid())):
    p.setCollisionFilterGroupMask(arm.getUUid(), i-1, collisionFilterGroup_kuka, collisionFilterMask_kuka)

# p.enableJointForceTorqueSensor(arm.getUUid(), 7)
p.enableJointForceTorqueSensor(arm.getUUid(), 8) # Why 8?

arm_ft_7 = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0.6, 0.3, 0.1], parentObjectUniqueId=arm.getUUid(), parentLinkIndex=7)

##########################
### Debug GUI elements ###
##########################

# Test Frames!
first_frame1 = frame.Frame(p, "first_frame1")
first_frame1.resetPositionAndOrientation([0, 0.1, 1], [0,1,0,0])

first_frame2 = frame.Frame(p, "first_frame2")
first_frame2.resetPositionAndOrientation([1, 1, 3], [0,0,0,1])

# ux = p.addUserDebugLine([0, 0, 0], [0,0,0], [0.3, 0.3, 0.3], 2)
# uy = p.addUserDebugLine([0, 0, 0], [0,0,0], [0.3, 0.3, 0.3], 2)
# uz = p.addUserDebugLine([0, 0, 0], [0,0,0], [0.3, 0.3, 0.3], 2)

dp_frame_tx = p.addUserDebugParameter("tx", -10, 10, 0)
dp_frame_ty = p.addUserDebugParameter("ty", -10, 10, 0.1)
dp_frame_tz = p.addUserDebugParameter("tz",   0, 10, 1)
dp_frame_rr = p.addUserDebugParameter("rr", -3.14*2, 3.14*2, 0)
dp_frame_rp = p.addUserDebugParameter("rp", -3.14*2, 3.14*2, 3.14)
dp_frame_ry = p.addUserDebugParameter("ry", -3.14*2, 3.14*2, 0)


#################
### Main loop ###
#################

objtrimesh = trimesh.load_mesh("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/data/3d/bend_wood_y_z.obj")
objtrimesh.apply_scale(0.1)
# Need to be done to be conform with bullet loading the model
# objtrimesh.apply_transform(trimesh.transformations.euler_matrix(1.57, 0, 0, 'rxyz'))
# 
# for face_index in range(len(objtrimesh.face_normals)):
#     # print(objtrimesh.vertices[point_index])
#     p.addUserDebugLine(objtrimesh.vertices[objtrimesh.faces[face_index][0]]+wood_offset_world, objtrimesh.vertices[objtrimesh.faces[face_index][1]]+wood_offset_world, [0.3, 0.3, 0.3], 3)
#     p.addUserDebugLine(objtrimesh.vertices[objtrimesh.faces[face_index][1]]+wood_offset_world, objtrimesh.vertices[objtrimesh.faces[face_index][2]]+wood_offset_world, [0.3, 0.3, 0.3], 3)
#     p.addUserDebugLine(objtrimesh.vertices[objtrimesh.faces[face_index][2]]+wood_offset_world, objtrimesh.vertices[objtrimesh.faces[face_index][0]]+wood_offset_world, [0.3, 0.3, 0.3], 3)

#     cgx = (objtrimesh.vertices[objtrimesh.faces[face_index][0]][0] + objtrimesh.vertices[objtrimesh.faces[face_index][1]][0] + objtrimesh.vertices[objtrimesh.faces[face_index][2]][0])/3
#     cgy = (objtrimesh.vertices[objtrimesh.faces[face_index][0]][1] + objtrimesh.vertices[objtrimesh.faces[face_index][1]][1] + objtrimesh.vertices[objtrimesh.faces[face_index][2]][1])/3
#     cgz = (objtrimesh.vertices[objtrimesh.faces[face_index][0]][2] + objtrimesh.vertices[objtrimesh.faces[face_index][1]][2] + objtrimesh.vertices[objtrimesh.faces[face_index][2]][2])/3

#     p.addUserDebugLine(np.array([cgx, cgy, cgz])+wood_offset_world, objtrimesh.face_normals[face_index]*0.1+np.array([cgx, cgy, cgz])+wood_offset_world, [0.3, 0.3, 1], 3)


# MyKey1 = p.addUserData(first_frame2.getFrameId(), "MyKey1", "MyValue1")

cm = constraint_manager.ConstraintManager(p)
cm.addFrame(first_frame1)
cm.addFrame(first_frame2)

# numJoints = p.getNumJoints(arm.getUUid())

# p.setRealTimeSimulation(1)

count_draw = 0

p.stepSimulation()

once = False
while (1):
    pose_frame_tx = p.readUserDebugParameter(dp_frame_tx)
    pose_frame_ty = p.readUserDebugParameter(dp_frame_ty)
    pose_frame_tz = p.readUserDebugParameter(dp_frame_tz)
    pose_frame_rr = p.readUserDebugParameter(dp_frame_rr)
    pose_frame_rp = p.readUserDebugParameter(dp_frame_rp)
    pose_frame_ry = p.readUserDebugParameter(dp_frame_ry)

    # camData = p.getDebugVisualizerCamera()
    # viewMat = camData[2]
    # projMat = camData[3]
    # p.getCameraImage(256,
    #                 256,
    #                 viewMatrix=viewMat,
    #                 projectionMatrix=projMat,
    #                 renderer=p.ER_BULLET_HARDWARE_OPENGL)

    # p.resetBasePositionAndOrientation(first_frame1.getFrameId(), [pose_frame_tx, pose_frame_ty, pose_frame_tz], p.getQuaternionFromEuler([pose_frame_rr, pose_frame_rp, pose_frame_ry]))

    f1_pos, f1_orn = p.getBasePositionAndOrientation(first_frame1.getFrameId())
    jointPoses = p.calculateInverseKinematics(arm.getUUid(), 8, np.array(f1_pos)+np.array([0,0,pose_frame_tz]), f1_orn) # Why 8?
    maxForce = 200.0
    if not once:
        # once = True
        for jointIndex in arm.getMotorIndices():
            p.resetJointState(arm.getUUid(), jointIndex, jointPoses[jointIndex-1])
            p.setJointMotorControl2(arm.getUUid(),
                                    jointIndex,
                                    p.POSITION_CONTROL,
                                    targetPosition=jointPoses[jointIndex-1],
                                    force=maxForce)
            


    # Get force
    # print("ft7 = " + str(p.getJointState(arm.getUUid(), 7)[2][0:3]))
    # print("ft8 = " + str(p.getJointState(arm.getUUid(), 8)[2][0:3]))
    force = np.array(p.getJointState(arm.getUUid(), 8)[2][0:3])
    force = force/np.linalg.norm(force)
    
    arm_ft_7 = p.addUserDebugLine([0, 0, 0], force, [0.6, 0.3, 0.1], parentObjectUniqueId=arm.getUUid(), parentLinkIndex=7, replaceItemUniqueId = arm_ft_7)

    keys = p.getKeyboardEvents()
    mouseEvents = p.getMouseEvents()
    
    for e in mouseEvents:
        # Selection
        if ((e[0] == 2) and (e[4] & p.KEY_WAS_TRIGGERED)):
            if e[3] == 0: # Left click
                ctrl_pressed = False
                for k, v in keys.items():
                    if not (k == p.B3G_CONTROL and (v & p.KEY_WAS_TRIGGERED)):
                        ctrl_pressed = True
                if not ctrl_pressed:
                    mouseX = e[1]
                    mouseY = e[2]
                    rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
                    rayInfo = p.rayTest(rayFrom, rayTo)
                    # print("rayyyyyy " + str(rayInfo))
                    cm.handlePick(rayInfo)
            elif e[3] == 2: # Right click
                selectedFrame = cm.getSelectedFrame()
                if selectedFrame:
                    mouseX = e[1]
                    mouseY = e[2]
                    rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
                    rayInfo = p.rayTest(rayFrom, rayTo)
                    if rayInfo and len(rayInfo) > 0 and rayInfo[0][0] > 0 and selectedFrame.getFrameId() != rayInfo[0][0]:
                        v_source = np.array([0,0,1])
                        v_normal = np.array(rayInfo[0][4])

                        v_source_norm = v_source/np.linalg.norm(v_source)
                        v_normal_norm = v_normal/np.linalg.norm(v_normal)

                        cos_theta = np.dot(v_source_norm, v_normal_norm)
                        angle = math.acos(cos_theta)
                        to_normalize = np.cross(v_source, v_normal)
                        w = to_normalize/np.linalg.norm(to_normalize)
                        qqq = p.getQuaternionFromAxisAngle(w, angle)

                        qqq = Quaternion([qqq[3], qqq[0], qqq[1], qqq[2]])
                        qqq = qqq * Quaternion(axis=[0., 1., 0.], angle=3.14)

                        selectedFrame.resetPositionAndOrientation(rayInfo[0][3], [qqq[1], qqq[2], qqq[3], qqq[0]])

                        # (closest_points, distances, triangle_id) = objtrimesh.nearest.on_surface(np.array(rayInfo[0][3]).transpose())
                        # locations, index_ray, index_tri = objtrimesh.ray.intersects_location(ray_origins=ray_origins, ray_directions=ray_directions)

    # for k, v in keys.items():
    #     if (k == 113) and (v == 1):
    p.stepSimulation()
    time.sleep(1/1000) # TODO DLW
    # time.sleep(1/500)


    # count_draw = count_draw + 1

try:
    signal.pause()
except (KeyboardInterrupt, SystemExit):
    print("Shutting down...")