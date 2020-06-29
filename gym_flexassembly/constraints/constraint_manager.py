import numpy as np

from gym_flexassembly.constraints import frame

from gym_flexassembly.constraints.maxwell_constraint import MaxwellConstraint

# from gym_flexassembly.constraints.contact_constraint import ContactConstraint

class ConstraintManager(object):
    """ TODO
    """

    def __init__(self, pb):
        self.p = pb
        self.constraints = []

    def addMaxwellConstraint(self, target_id, ref_id):
        c = MaxwellConstraint(self.p, target_id, ref_id)
        self.constraints.append(c)

        # d = ContactConstraint(self.p, [1,1,1,1,1,1], target_id, ref_id, target_link_id=-1, target_name=None, ref_name=None, bilateral=[1,1,1,1,1,1])

    def updateConstraints(self):
        for e in self.constraints:
            e.updateConstraint()