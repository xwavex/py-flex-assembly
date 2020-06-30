import numpy as np

from gym_flexassembly.constraints import frame

from gym_flexassembly.constraints.maxwell_constraint import MaxwellConstraint

from gym_flexassembly.constraints.contact_constraint import ContactConstraint

class ConstraintManager(object):
    """ TODO
    """

    def __init__(self, pb):
        self.p = pb
        self.constraints = []

    def addMaxwellConstraint(self, target_id, ref_id):
        c = MaxwellConstraint(self.p, target_id, ref_id)
        self.constraints.append(c)

    def addContactConstraint(self, target_frame, axis=[1,1,1,1,1,1], bilateral=[1,1,1,1,1,1]):
        d = ContactConstraint(self.p, axis, target_frame, bilateral=bilateral)
        self.constraints.append(d)

    def updateConstraints(self):
        for e in self.constraints:
            e.updateConstraint()