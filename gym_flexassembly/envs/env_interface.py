import os, inspect

# UTILITY IMPORTS
import math
import numpy as np
import random
import threading
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
        self.ros_loaded = False
        # (OPTIONAL) ROS IMPORTS
        try:
            import rospy
            from std_msgs.msg import Header
            from geometry_msgs.msg import Pose
            from sensor_msgs.msg import Image
            import cv_bridge

            global rospy, cv_bridge, Pose, Image, Header

            rospy.init_node('env', anonymous=True)

            self.ros_loaded = True
        except ImportError:
            self.ros_loaded = False
        
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
        self._p.loadURDF(os.path.join(flexassembly_data.getDataPath(), "objects/plane_solid.urdf"), useMaximalCoordinates=True) # Brauche ich fuer die hit rays


        # self._p.setRealTimeSimulation(1)

        # self._p.stepSimulation()

        dv = 2.0
        self.dp_joint_pos_0 = p.addUserDebugParameter("j0", -dv, dv, 0)
        self.dp_joint_pos_1 = p.addUserDebugParameter("j1", -dv, dv, 0)
        self.dp_joint_pos_2 = p.addUserDebugParameter("j2", -dv, dv, 0)
        self.dp_joint_pos_3 = p.addUserDebugParameter("j3", -dv, dv, 0)
        self.dp_joint_pos_4 = p.addUserDebugParameter("j4", -dv, dv, 0)
        self.dp_joint_pos_5 = p.addUserDebugParameter("j5", -dv, dv, 0)
        self.dp_joint_pos_6 = p.addUserDebugParameter("j6", -dv, dv, 0)

        self._cameras = {}

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
        for camera in self._cameras:
            self.remove_camera(camera)

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

    def getDebugJointCommands(self):
        joint_pos_0 = self._p.readUserDebugParameter(self.dp_joint_pos_0)
        joint_pos_1 = self._p.readUserDebugParameter(self.dp_joint_pos_1)
        joint_pos_2 = self._p.readUserDebugParameter(self.dp_joint_pos_2)
        joint_pos_3 = self._p.readUserDebugParameter(self.dp_joint_pos_3)
        joint_pos_4 = self._p.readUserDebugParameter(self.dp_joint_pos_4)
        joint_pos_5 = self._p.readUserDebugParameter(self.dp_joint_pos_5)
        joint_pos_6 = self._p.readUserDebugParameter(self.dp_joint_pos_6)
        return [joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5,joint_pos_6]

    def remove_camera(self, name):
        if name not in self._cameras:
            return

        self._cameras[name]['stop_flag'] = True
        self._cameras[name]['thread'].join()
        del self._cameras[name]
    
    def add_camera(self, settings, name):
        if name in self._cameras:
            raise ValueError('Camera[%s] already exists!' % name)

        self._cameras[name] = {}
        self._cameras[name]['settings'] = settings
        self._cameras[name]['stop_flag'] = False
        def thread_func():
            pub_depth = rospy.Publisher('/camera/' + name + '/depth/image_raw', Image, queue_size=10)
            pub_color = rospy.Publisher('/camera/' + name + '/color/image_raw', Image, queue_size=10)

            bridge = cv_bridge.CvBridge()

            rate = rospy.Rate(settings['framerate'])
            while not self._cameras[name]['stop_flag']:
                # get a new camera image
                view_matrix = self._p.computeViewMatrix(settings['pos'],
                                                        settings['target_pos'],
                                                        settings['up'])
                projection_matrix = self._p.computeProjectionMatrixFOV(settings['fov'],
                                                                       settings['width'] / settings['height'],
                                                                       settings['near'],
                                                                       settings['far'])
                try: 
                    _, _, rgba, depth_buffer, _ = self._p.getCameraImage(settings['width'],
                                                                         settings['height'],
                                                                         view_matrix,
                                                                         projection_matrix,
                                                                         shadow=True,
                                                                         renderer=self._p.ER_BULLET_HARDWARE_OPENGL)
                except Exception as x:
                    # lost connection to pybullet server so quit
                    break

                near = settings['near']
                far = settings['far']
                depth = far * near / (far - (far - near) * depth_buffer)
                rgb = rgba[:, :, :3]

                # create a header for the messages
                header = Header()
                header.stamp = rospy.Time.now()

                try:
                    img_depth = bridge.cv2_to_imgmsg(depth, 'passthrough')
                    img_color = bridge.cv2_to_imgmsg(rgb, 'passthrough')
                except cv_bridge.CvBridgeError as e:
                    print('Could not convert CV image to ROS msg: ' + str(e))

                # publish the images
                img_depth.header = header
                img_depth.header.frame_id = 'depth_image'
                pub_depth.publish(img_depth)

                img_color.header = header
                img_color.header.frame_id = 'color_image'
                pub_color.publish(img_color)

                rate.sleep()

            pub_color.unregister()
            pub_depth.unregister()

        self._cameras[name]['thread'] = threading.Thread(target=thread_func, daemon=True)
        self._cameras[name]['thread'].start()
