import numpy as np

from gym_flexassembly.utils import transformations

class ForceConstraint(object):
    """ TODO
    """

    def __init__(
        self,
        pb,
        kp=1,
        ko=None,
        kv=None,
        ki=0,
        vmax=None,
        ctrlr_dof=None,
        null_controllers=None,
        use_g=True,
        use_C=False,
        orientation_algorithm=0,
    ):

        self._pb = pb

        self.offset_zeros = np.zeros(3)

        self.kp = kp
        self.ko = kp if ko is None else ko
        # TODO: find the appropriate default critical damping value
        # when using different position and orientation gains
        self.kv = np.sqrt(self.kp + self.ko) if kv is None else kv
        self.ki = ki
        self.null_controllers = null_controllers
        self.use_g = use_g
        self.use_C = use_C
        self.orientation_algorithm = orientation_algorithm

        if self.ki != 0:
            self.integrated_error = np.zeros(6)

        # if ctrlr_dof is None:
        #     ctrlr_dof = [True, True, True, False, False, False]
        ctrlr_dof = [True, True, True, True, True, True]
        self.ctrlr_dof = np.copy(ctrlr_dof)
        self.n_ctrlr_dof = np.sum(self.ctrlr_dof)

        self.task_space_gains = np.array([self.kp] * 3 + [self.ko] * 3)
        self.lamb = self.task_space_gains / self.kv

        self.vmax = vmax
        if vmax is not None:
            # precalculate gains used in velocity limiting
            self.sat_gain_xyz = vmax[0] / self.kp * self.kv
            self.sat_gain_abg = vmax[1] / self.ko * self.kv
            self.scale_xyz = vmax[0] / self.kp * self.kv
            self.scale_abg = vmax[1] / self.ko * self.kv

        self.ZEROS_SIX = np.zeros(6)
        self.IDENTITY_N_JOINTS = np.eye(7)
