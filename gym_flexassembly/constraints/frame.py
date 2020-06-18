import numpy as np
import os

import gym_flexassembly.data as data

class Frame(object):
    """ TODO
    """

    def __init__(self, pb, text, fixed_base=True, ref_id=-1):
        self.p = pb
        self.text = text
        self.ref_id = ref_id
        urdfRootPath = data.getDataPath()
        self.frame_ghost_id = self.p.loadURDF(os.path.join(urdfRootPath, "frame_full.urdf"), useFixedBase=fixed_base)
        self.p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=self.frame_ghost_id, parentLinkIndex=-1)
        self.p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=self.frame_ghost_id, parentLinkIndex=-1)
        self.p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=self.frame_ghost_id, parentLinkIndex=-1)
        self.frame_text_node = self.p.addUserDebugText(str(self.text), [0, 0.15, 0.15],
                        textColorRGB=[0, 0, 0],
                        textSize=1.0,
                        parentObjectUniqueId=self.frame_ghost_id,
                        parentLinkIndex=-1)
        self.p.addUserDebugLine([0, 0.05, 0.05], [0, 0.14, 0.14], [0, 0, 0], parentObjectUniqueId=self.frame_ghost_id, parentLinkIndex=-1)

        self.deselectedTransparency = 0.5
        self.selectedTransparency = 1
        self.transparency = self.deselectedTransparency
        self.visibility = [True, True, True, True, True, True]

        self.updateVisual()

        # Add collision disabling stuff! Binary, ray only works on 0x1
        collisionFilterGroup = 0x100
        collisionFilterMask =  0x1
        for i in range(self.p.getNumJoints(self.frame_ghost_id)):
            self.p.setCollisionFilterGroupMask(self.frame_ghost_id, i-1, collisionFilterGroup, collisionFilterMask)

        # Conclusion on p.setCollisionFilterGroupMask(id, (body_id) -1, collisionFilterGroup, collisionFilterMask):
        # Raytest works with collisionFilterMask = 0x1 ONLY
        # EVERYTHING is BINARY
        # DIFFERENT GROUP IDs == NO COLLISION!
        # However, groups need to be assigned at the beginning!

        # enableCollision = 1
        # p.setCollisionFilterPair(planeId, cubeId, -1, -1, enableCollision)

        self.internal_pos = [0,0,0]
        self.internal_orn = [0,0,0,1]

    def updateVisual(self):
        self.p.changeVisualShape(self.frame_ghost_id, 0, rgbaColor=[1, 0, 0, (self.transparency if self.visibility[0] else 0)]) # X
        self.p.changeVisualShape(self.frame_ghost_id, 2, rgbaColor=[0, 1, 0, (self.transparency if self.visibility[1] else 0)]) # Y
        self.p.changeVisualShape(self.frame_ghost_id, 4, rgbaColor=[0, 0, 1, (self.transparency if self.visibility[2] else 0)]) # Z
        self.p.changeVisualShape(self.frame_ghost_id, 1, rgbaColor=[1, 0, 0, (self.transparency if self.visibility[3] else 0)]) # RotX
        self.p.changeVisualShape(self.frame_ghost_id, 3, rgbaColor=[0, 1, 0, (self.transparency if self.visibility[4] else 0)]) # RotY
        self.p.changeVisualShape(self.frame_ghost_id, 5, rgbaColor=[0, 0, 1, (self.transparency if self.visibility[5] else 0)]) # RotZ

    def select(self):
        self.transparency = self.selectedTransparency
        self.updateVisual()

    def deselect(self):
        self.transparency = self.deselectedTransparency
        self.updateVisual()

    def setVisibility(self, index, visible):
        if index == 0:
            self.visibility[index] = visible
            self.p.changeVisualShape(self.frame_ghost_id, 0, rgbaColor=[1, 0, 0, (self.transparency if visible else 0)]) # X
        elif index == 1:
            self.visibility[index] = visible
            self.p.changeVisualShape(self.frame_ghost_id, 2, rgbaColor=[0, 1, 0, (self.transparency if visible else 0)]) # Y
        elif index == 2:
            self.visibility[index] = visible
            self.p.changeVisualShape(self.frame_ghost_id, 4, rgbaColor=[0, 0, 1, (self.transparency if visible else 0)]) # Z
        elif index == 3:
            self.visibility[index] = visible
            self.p.changeVisualShape(self.frame_ghost_id, 1, rgbaColor=[1, 0, 0, (self.transparency if visible else 0)]) # RotX
        elif index == 4:
            self.visibility[index] = visible
            self.p.changeVisualShape(self.frame_ghost_id, 3, rgbaColor=[0, 1, 0, (self.transparency if visible else 0)]) # RotY
        elif index == 5:
            self.visibility[index] = visible
            self.p.changeVisualShape(self.frame_ghost_id, 5, rgbaColor=[0, 0, 1, (self.transparency if visible else 0)]) # RotZ

    def getFrameId(self):
        return self.frame_ghost_id

    def setFrameText(self, text):
        self.text = text
        self.frame_text_node = self.p.addUserDebugText(str(self.text), [0, 0.15, 0.15],
                        textColorRGB=[0, 0, 0],
                        textSize=1.0,
                        parentObjectUniqueId=frame_ghost_id,
                        parentLinkIndex=-1,
                        replaceItemUniqueId = self.frame_text_node)

    def resetPositionAndOrientation(self, pos, orn):
        self.internal_pos = pos
        self.internal_orn = orn

        if self.ref_id > -1:
            rpos, rorn = self.p.getBasePositionAndOrientation(self.ref_id)
            npos, norn = self.p.multiplyTransforms(rpos,rorn,pos,orn)
            self.p.resetBasePositionAndOrientation(self.frame_ghost_id, npos, norn)
        else:
            self.p.resetBasePositionAndOrientation(self.frame_ghost_id, pos, orn)

    def getInternalPosOrn(self):
        return self.internal_pos, self.internal_orn

    def getRefId(self):
        return self.ref_id