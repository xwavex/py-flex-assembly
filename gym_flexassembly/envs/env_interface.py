import os, inspect

# UTILITY IMPORTS
import math
import numpy as np
import random
import time

# PYBULLET IMPORTS
import pybullet as p
import pybullet_data

# GYM IMPORTS
import gym
from gym import error, spaces, utils
from gym.utils import seeding

# DEPLOYMENT IMPORTS
from pkg_resources import parse_version

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data

# CONSTRAINTS MANAGER IMPORTS
from gym_flexassembly.constraints import frame
from gym_flexassembly.constraints import constraint_manager
from gym_flexassembly.constraints import frame_manager

class EnvInterface(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self, gui=True, ros_frame_broadcaster=None):
        self._client_id = -1;

        self._timeStep = 1.0 / 1000.0

        self._run = False

        self._p = p

        self._gui = gui

        self._cm = None
        self._fm = None

        self._fb = ros_frame_broadcaster

        if self._gui:
            self._client_id = self._p.connect(self._p.GUI_SERVER, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
        else:
            self._client_id = self._p.connect(self._p.SHARED_MEMORY_SERVER)

        self._p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        self._p.resetSimulation()
        self._p.setTimeStep(self._timeStep)
        self._p.setGravity(0, 0, -9.81)

        # Floor SHOULD BE ALWAYS ID 0
        self._p.loadURDF(os.path.join(flexassembly_data.getDataPath(), "plane_solid.urdf"), useMaximalCoordinates=True) # Brauche ich fuer die hit rays


        # self._p.setRealTimeSimulation(1)

        # self._p.stepSimulation()

        self.setup_manager()

    def setup_manager(self):
        self._cm = constraint_manager.ConstraintManager(self._p)
        self._fm = frame_manager.FrameManager(self._p, self._fb)

    def getFrameManager(self):
        return self._fm

    def getConstraintManager(self):
        return self._cm

    def get_client_id(self):
        return self._client_id

    def __del__(self):
        self._p.disconnect()

    def set_running(self, run):
        self._run = run

    def handle_input_events(self):
        self._fm.handleKeyAndMouseEvents()

    def step_sim(self):
        if self._run:
            self._p.stepSimulation()
        if self._gui:
            time.sleep(self._timeStep)

    def updateConstraints(self):
        if self._cm:
            self._cm.updateConstraints()