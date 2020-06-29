import numpy as np
import os

import gym_flexassembly.data as data

import rospy
from geometry_msgs.msg import TransformStamped

class ContactConstraint(object):
    """ TODO
    """

    def __init__(self, pb, axis, target_id, ref_id, target_link_id=-1, target_name=None, ref_name=None, bilateral=[1,1,1,1,1,1]):
        self.p = pb
        self.target_id = target_id
        self.ref_id = ref_id
        self.target_link_id = target_link_id

        self.target_pos = [0,0,0]
        self.ref_pos = [0,0,0]

        self.axis = axis
        self.bilateral = bilateral
        
        self.line = self.p.addUserDebugLine(self.ref_pos, self.target_pos, [1, 1, 0], 8)
        # self.frame_text_node = self.p.addUserDebugText(str(self.text), [0, 0.15, 0.15],
        #                 textColorRGB=[0, 0, 0],
        #                 textSize=1.0,
        #                 parentObjectUniqueId=self.frame_ghost_id,
        #                 parentLinkIndex=-1)

        self.line_length = 0.2
        if self.axis[0] == 1:
            # tX
            self.line_x = self.p.addUserDebugLine([0, 0, 0], [self.line_length, 0, 0], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)
        if self.axis[1] == 1:
            # tY
            self.line_y = self.p.addUserDebugLine([0, 0, 0], [0, self.line_length, 0], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)
        if self.axis[2] == 1:
            # tZ
            self.line_z = self.p.addUserDebugLine([0, 0, 0], [0, 0, self.line_length], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)

        if self.axis[3] == 1:
            # rX
            self.line_rx_a = self.p.addUserDebugLine([0.1, 0, 0.1], [0.1, 0.1, 0], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)
            self.line_rx_b = self.p.addUserDebugLine([0.1, 0, 0.1], [0.1, -0.1, 0], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)
        
        if self.axis[4] == 1:
            # rY
            self.line_ry_a = self.p.addUserDebugLine([0, 0.1, 0.1], [0.1, 0.1, 0], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)
            self.line_ry_b = self.p.addUserDebugLine([0, 0.1, 0.1], [-0.1, 0.1, 0], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)
        
        if self.axis[5] == 1:
            # rZ
            self.line_rz_a = self.p.addUserDebugLine([0.1, 0, 0.1], [0, 0.1, 0.1], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)
            self.line_rz_b = self.p.addUserDebugLine([0.1, 0, 0.1], [0, -0.1, 0.1], [1, 1, 0], 1, parentObjectUniqueId=self.target_id, parentLinkIndex=self.target_link_id)

    def updateConstraint(self):
        pass

    def getTargetId(self):
        return self.target_id

    def getRefId(self):
        return self.ref_id