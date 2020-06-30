import numpy as np
import os

import math

import pybullet as pb

import tf2_ros
from geometry_msgs.msg import TransformStamped

from gym_flexassembly.constraints import frame

from pyquaternion import Quaternion

class FrameManager(object):
    """ TODO
    """

    def __init__(self, pb, frame_broadcaster=None):
        self.p = pb
        # s[i]=j
        self.frame_id_storage = {}
        # s[i]=[j,j,j,j,...]
        self.frame_constraint_storage = {}

        self.frame_dependency_order = []

        self.selectedFrame = None

        # self.fb = frame_broadcaster

    def createFrame(self, name, pos=[0,0,0], orn=[0,0,0,1], ref_id=-1, ref_link_id=-1, is_body_frame=False):
        print("CREATE FRAME " + str(name) + " ref_id=" + str(ref_id) + " ref_link_id=" + str(ref_link_id) + " is_body_frame=" + str(is_body_frame))
        tmp_frame = None
        if ref_id > -1:
            ref_name_tmp = ""
            if not is_body_frame:
                ref_name_tmp = self.frame_id_storage[ref_id].getName()
        
            tmp_frame = frame.Frame(self.p, name, fixed_base=True, ref_id=ref_id, ref_link_id=ref_link_id, ref_name=ref_name_tmp, is_body_frame=is_body_frame)
        else:
            tmp_frame = frame.Frame(self.p, name, fixed_base=True)
        
        print("CREATED WITH ID " + str(tmp_frame.getFrameId()))
        self.frame_id_storage[tmp_frame.getFrameId()]=tmp_frame
        print("createFrame with " + str(name) + " at " + str(pos) + " and " + str(orn) + " : " + str(tmp_frame.getFrameId()) + " at ref: " + str(ref_id))

        self.recalculateFrameDependency()
        tmp_frame.resetPositionAndOrientation(pos, orn)

        # if self.fb:
        #     self.fb.sendTransform(tmp_frame.getROSTransformStamped())

        # TODO avoid douplings
        self.updateFramePoses()

        return tmp_frame.getFrameId()

    def calculateFrameDependency(self, frame, dep_order):
        if not frame.isBodyFrame():
            # Assume that origin frame is already here and objects can only be positioned with reference to the origin frame for now!
            if frame.getRefId() > -1:
                self.calculateFrameDependency(self.frame_id_storage[frame.getRefId()],dep_order)

        if frame not in dep_order:
            dep_order.append(frame)

    def recalculateFrameDependency(self):
        self.frame_dependency_order = []
        for entry in self.frame_id_storage:
            self.calculateFrameDependency(self.frame_id_storage[entry], self.frame_dependency_order)

    def updateFramePoses(self):
        for frame in self.frame_dependency_order:
            pos, orn = frame.getInternalPosOrn()
            frame.resetPositionAndOrientation(pos, orn)

    def getRayFromTo(self, mouseX, mouseY):
        width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = self.p.getDebugVisualizerCamera(
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

    def handleKeyAndMouseEvents(self):
        # Perhaps this should be C/C++ extension
        keys = self.p.getKeyboardEvents()
        mouseEvents = self.p.getMouseEvents()
        
        for e in mouseEvents:
            # Selection
            if ((e[0] == 2) and (e[4] & self.p.KEY_WAS_TRIGGERED)):
                if e[3] == 0: # Left click
                    ctrl_pressed = False
                    for k, v in keys.items():
                        if not (k == self.p.B3G_CONTROL and (v & self.p.KEY_WAS_TRIGGERED)):
                            ctrl_pressed = True
                    if not ctrl_pressed:
                        mouseX = e[1]
                        mouseY = e[2]
                        rayFrom, rayTo = self.getRayFromTo(mouseX, mouseY)
                        rayInfo = self.p.rayTest(rayFrom, rayTo)
                        print("rayyyyyy " + str(rayInfo))
                        self.handlePick(rayInfo)
                # elif e[3] == 2: # Right click
                #     selectedFrame = self.getSelectedFrame()
                #     if selectedFrame:
                #         mouseX = e[1]
                #         mouseY = e[2]
                #         rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
                #         rayInfo = self.p.rayTest(rayFrom, rayTo)
                #         if rayInfo and len(rayInfo) > 0 and rayInfo[0][0] > 0 and selectedFrame.getFrameId() != rayInfo[0][0]:
                #             v_source = np.array([0,0,1])
                #             v_normal = np.array(rayInfo[0][4])

                #             v_source_norm = v_source/np.linalg.norm(v_source)
                #             v_normal_norm = v_normal/np.linalg.norm(v_normal)

                #             cos_theta = np.dot(v_source_norm, v_normal_norm)
                #             angle = math.acos(cos_theta)
                #             to_normalize = np.cross(v_source, v_normal)
                #             w = to_normalize/np.linalg.norm(to_normalize)
                #             qqq = p.getQuaternionFromAxisAngle(w, angle)

                #             qqq = Quaternion([qqq[3], qqq[0], qqq[1], qqq[2]])
                #             qqq = qqq * Quaternion(axis=[0., 1., 0.], angle=3.14)

                #             selectedFrame.resetPositionAndOrientation(rayInfo[0][3], [qqq[1], qqq[2], qqq[3], qqq[0]])

    def getSelectedFrame(self):
        return self.selectedFrame

    def handlePick(self, rayInfo):
        for info in rayInfo:
            found = False
            for frame_key in self.frame_id_storage:
                frame = self.frame_id_storage[frame_key]
                frame_id = frame.getFrameId()
                if info[0] == frame_id:
                    if self.selectedFrame == frame:
                        found = True
                        break
                    if self.selectedFrame:
                        # Deselect frame first
                        self.selectedFrame.deselect()
                    # Select new frame
                    self.selectedFrame = frame
                    self.selectedFrame.select()
                    found = True
                    break
            if not found:
                if self.selectedFrame:
                    # Deselect frame
                    self.selectedFrame.deselect()
                    self.selectedFrame = None