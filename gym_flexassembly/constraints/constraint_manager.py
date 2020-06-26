import numpy as np

from gym_flexassembly.constraints import frame

class ConstraintManager(object):
    """ TODO
    """

    def __init__(self, pb):
        self.p = pb
        # s[i]=j
        self.frame_id_storage = {}
        # s[i]=[j,j,j,j,...]
        self.frame_constraint_storage = {}

        self.frame_dependency_order = []

        self.selectedFrame = None

