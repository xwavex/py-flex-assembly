import numpy as np
import os

import gym_flexassembly.data as data

import rospy
from geometry_msgs.msg import TransformStamped

class MaxwellConstraint(object):
    """ TODO
    """

    def __init__(self, pb, target_id, ref_id, target_name=None, ref_name=None):
        self.p = pb
        self.target_id = target_id
        self.ref_id = ref_id

        self.target_pos = [0,0,0]
        self.ref_pos = [0,0,0]
        
        self.line1_id = self.p.addUserDebugLine(self.ref_pos, self.target_pos, [1, 1, 0], 8)
        self.line2_id = self.p.addUserDebugLine(self.ref_pos, self.target_pos, [1, 1, 0], 8)
        self.line3_id = self.p.addUserDebugLine(self.ref_pos, self.target_pos, [1, 1, 0], 8)
        self.line4_id = self.p.addUserDebugLine(self.ref_pos, self.target_pos, [1, 1, 0], 8)
        self.line5_id = self.p.addUserDebugLine(self.ref_pos, self.target_pos, [1, 1, 0], 8)
        # self.frame_text_node = self.p.addUserDebugText(str(self.text), [0, 0.15, 0.15],
        #                 textColorRGB=[0, 0, 0],
        #                 textSize=1.0,
        #                 parentObjectUniqueId=self.frame_ghost_id,
        #                 parentLinkIndex=-1)

    def updateConstraint(self):
        self.target_pos, orn = self.p.getBasePositionAndOrientation(self.target_id)
        self.ref_pos, orn = self.p.getBasePositionAndOrientation(self.ref_id)

        # dist
        dist_step = (np.array(self.target_pos) - np.array(self.ref_pos)) / 5
        temp_end = np.array(self.ref_pos) + dist_step
        upstep = 0.1
        temp_end[2] =  temp_end[2] + upstep
        self.line1_id = self.p.addUserDebugLine(self.ref_pos, temp_end, [1, 1, 0], 8, replaceItemUniqueId=self.line1_id)

        start = temp_end
        temp_end = temp_end + dist_step
        self.line2_id = self.p.addUserDebugLine(start, [temp_end[0], temp_end[1], temp_end[2] + 0.1], [1, 1, 0], 8, replaceItemUniqueId=self.line2_id)

        temp_end = temp_end + dist_step
        self.line3_id = self.p.addUserDebugLine(self.ref_pos, [temp_end[0], temp_end[1], temp_end[2] + 0.1], [1, 1, 0], 8, replaceItemUniqueId=self.line3_id)

        temp_end = temp_end + dist_step
        self.line4_id = self.p.addUserDebugLine(self.ref_pos, [temp_end[0], temp_end[1], temp_end[2] + 0.1], [1, 1, 0], 8, replaceItemUniqueId=self.line4_id)
        self.line5_id = self.p.addUserDebugLine(self.ref_pos, self.target_pos, [1, 1, 0], 8, replaceItemUniqueId=self.line5_id)

    def getTargetId(self):
        return self.target_id

    def getRefId(self):
        return self.ref_id