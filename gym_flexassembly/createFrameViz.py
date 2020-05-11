import os, inspect

import pybullet as p
import time
import math

import data

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



p.connect(p.GUI)
#don't create a ground plane, to allow for gaps etc
# p.resetSimulation()
#p.createCollisionShape(p.GEOM_PLANE)
#p.createMultiBody(0,0)
#p.resetDebugVisualizerCamera(5,75,-26,[0,0,1]);
# p.resetDebugVisualizerCamera(15, -346, -16, [-15, 0, 1])

import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", useMaximalCoordinates=True) # Brauche ich fuer die hit rays

#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# #disable tinyrenderer, software (CPU) renderer, we don't use it here
# p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

urdfRootPath = data.getDataPath()
frame_ghost_id = p.loadURDF(os.path.join(urdfRootPath, "frame_ghost.urdf"), useFixedBase=True)
p.resetBasePositionAndOrientation(frame_ghost_id, [-1, 1, 1],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
p.changeVisualShape(frame_ghost_id, -1, rgbaColor=[1, 1, 1, 0])
p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=frame_ghost_id, parentLinkIndex=-1)
p.addUserDebugText("tip", [0, 0, 0.1],
                   textColorRGB=[1, 0, 0],
                   textSize=1.5,
                   parentObjectUniqueId=frame_ghost_id,
                   parentLinkIndex=-1)





p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

while (1):
    camData = p.getDebugVisualizerCamera()
    viewMat = camData[2]
    projMat = camData[3]
    p.getCameraImage(256,
                    256,
                    viewMatrix=viewMat,
                    projectionMatrix=projMat,
                    renderer=p.ER_BULLET_HARDWARE_OPENGL)
    keys = p.getKeyboardEvents()
    mouseEvents = p.getMouseEvents()
    for e in mouseEvents:
        if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
            mouseX = e[1]
            mouseY = e[2]
            rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
            rayInfo = p.rayTest(rayFrom, rayTo)
            print("rayInfo = " + str(rayInfo))
            # p.addUserDebugLine(rayFrom,rayTo,[1,0,0],3)
            for l in range(len(rayInfo)):
                hit = rayInfo[l]
                objectUid = hit[0]
                print(objectUid)
                if (objectUid >= 1):
                    print("yesss")
                    # p.changeVisualShape(objectUid, -1, rgbaColor=colors[currentColor])
    p.stepSimulation()


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
