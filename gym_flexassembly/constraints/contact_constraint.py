import numpy as np
import os

import gym_flexassembly.data as data

import rospy
from geometry_msgs.msg import TransformStamped

from gym_flexassembly.constraints import frame

class ContactConstraint(object):
    """ TODO
    """

    def __init__(self, pb, axis, target_frame, direction=[1,1,1,1,1,1]):
        self.p = pb
        self.target_frame = target_frame
        self.axis = axis
        self.direction = direction

        self.constraint_id = -1
        
        # self.line = self.p.addUserDebugLine(self.ref_pos, self.target_pos, [1, 1, 0], 8)
        # self.frame_text_node = self.p.addUserDebugText(str(self.text), [0, 0.15, 0.15],
        #                 textColorRGB=[0, 0, 0],
        #                 textSize=1.0,
        #                 parentObjectUniqueId=self.frame_ghost_id,
        #                 parentLinkIndex=-1)

        if self.axis[0] == 1:
            # tX
            self.target_frame.setVisibility(0, False)
        if self.axis[1] == 1:
            # tY
            self.target_frame.setVisibility(1, False)
        if self.axis[2] == 1:
            # tZ
            self.target_frame.setVisibility(2, False)
        if self.axis[3] == 1:
            # rX
            self.target_frame.setVisibility(3, False)
        if self.axis[4] == 1:
            # rY
            self.target_frame.setVisibility(4, False)
        if self.axis[5] == 1:
            # rZ
            self.target_frame.setVisibility(5, False)

    def updateConstraint(self):
        pass

    def getTargetFrame(self):
        return self.target_frame

    def setId(self, cid):
        self.constraint_id = cid

    def getId(self):
        return self.constraint_id
