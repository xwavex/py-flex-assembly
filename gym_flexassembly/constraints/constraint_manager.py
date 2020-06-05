import numpy as np

from . import frame

class ConstraintManager(object):
    """ TODO
    """

    def __init__(self, pb):
        self.p = pb
        self.frames = []
        self.selectedFrame = None

    def addFrame(self, frame):
        self.frames.append(frame)
        return len(self.frames) - 1 # Index

    def getSelectedFrame(self):
        return self.selectedFrame

    def handlePick(self, rayInfo):
        for info in rayInfo:
            found = False
            for frame in self.frames:
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

