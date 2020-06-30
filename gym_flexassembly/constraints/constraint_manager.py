import numpy as np

from gym_flexassembly.constraints import frame

from gym_flexassembly.constraints.maxwell_constraint import MaxwellConstraint

from gym_flexassembly.constraints.contact_constraint import ContactConstraint

class ConstraintManager(object):
    """ TODO
    """

    def __init__(self, pb):
        self.p = pb
        self.constraints_map = {}
        self.constraint_id_counter = 0

    def addMaxwellConstraint(self, target_id, ref_id):
        c = MaxwellConstraint(self.p, target_id, ref_id)
        self.constraint_id_counter = self.constraint_id_counter + 1
        c.setId(self.constraint_id_counter)
        self.constraints_map[self.constraint_id_counter] = c
        return c.getId()

    def addContactConstraint(self, target_frame, axis=[1,1,1,1,1,1], direction=[1,1,1,1,1,1]):
        d = ContactConstraint(self.p, axis, target_frame, direction=direction)
        self.constraint_id_counter = self.constraint_id_counter + 1
        d.setId(self.constraint_id_counter)
        self.constraints_map[self.constraint_id_counter] = d
        return d.getId()

    def updateConstraints(self):
        for key in self.constraints_map:
            e = self.constraints_map[key]
            e.updateConstraint()