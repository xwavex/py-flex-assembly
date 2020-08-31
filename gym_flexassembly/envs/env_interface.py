import os, inspect

# UTILITY IMPORTS
import math
import numpy as np
import random
import threading
import time
import signal
import sys

# PYBULLET IMPORTS
# PyBullet
import pybullet as p
# PyBullet Planning
from pybullet_planning.interfaces.robots.body import get_bodies
from pybullet_planning.interfaces.robots import get_movable_joints

# import pybullet_utils.bullet_client as bc
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

    def __init__(self, gui=True, ros_frame_broadcaster=None, direct=False, use_real_interface=True, hz=250.0):
        ''' Initialize the base class for the environments.
            >>> Using use_real_interface the ros interfaces are exposed that the real system will utilize.
        '''
        self._use_real_interface = use_real_interface
        self._robot_map = {}
        self._node_name = "env"

        # self._hz = 1000.0
        self._hz = hz
        self._timeStep = 1.0 / self._hz

        if self._use_real_interface:
            # load (optional) ROS imports if the real interface should be mirrored
            try:
                import rospy
                from std_msgs.msg import Header
                from geometry_msgs.msg import Pose
                from sensor_msgs.msg import Image
                import cv_bridge

                # not dure if we really need this
                global rospy, cv_bridge, Pose, Image, Header

                # create ros node
                rospy.init_node(self._node_name, anonymous=False)

                self.rate = rospy.Rate(self._hz) # 250hz
            except ImportError:
                print("ERROR IMPORTING ros 1", file=sys.stderr)
        
        self._client_id = -1

        self._run = False

        self._p = p

        self._gui = gui
        self._direct = direct

        self._cm = None
        self._fm = None

        self._fb = ros_frame_broadcaster

        if self._gui:
            self._client_id = self._p.connect(self._p.GUI_SERVER, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
        else:
            if self._direct:
                self._client_id = self._p.connect(self._p.DIRECT)
            else:
                self._client_id = self._p.connect(self._p.SHARED_MEMORY_SERVER)
        print("Initialized main pybullet instance with id",self._client_id)

        self._p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        self._p.resetSimulation()
        self._p.setGravity(0, 0, -9.81)
        self._p.setTimeStep(self._timeStep)

        # Floor SHOULD BE ALWAYS ID 0 because hitrays need this to work properly
        self._p.loadURDF(os.path.join(flexassembly_data.getDataPath(), "objects/plane_solid.urdf"), useMaximalCoordinates=True)


        # self._p.setRealTimeSimulation(1)
        # self._p.stepSimulation()

        self._cameras = {}

        # Managers are still WIP!
        self.setup_manager()

    def env_observation(self):
        ''' Provide the observation of the scenario.
        '''
        return self.observation_internal()

    def observation_internal(self):
        ''' Provide the observation from the derived class.
        '''
        pass

    def env_loop(self):
        ''' Call this one time to enter the main loop.
        '''
        if self._use_real_interface:
            while not rospy.is_shutdown():
                # p.stepSimulation() # Only use this is we are not triggered externally...
                self.env_step()
                # if self._gui:
                #    # time.sleep(self._timeStep)
                self.rate.sleep()
        else:
            while(1):
                self.env_step()
                # if self._gui:
                time.sleep(self._timeStep)

        try:
            signal.pause()
        except (KeyboardInterrupt, SystemExit):
            print("Shutting down...")

    def env_step(self):
        ''' This function manages one step.
        '''
        self.step_internal()

        if self._run:
            self._p.stepSimulation()

    def step_internal(self):
        ''' This function needs overridden by the derived class.
        '''
        pass

    def env_reset(self):
        ''' This function manages the reset process.
        '''
        self._p.resetSimulation()
        # TODO overrride the server?
        self._robot_map = {}
        # TODO trigger a reset message
        self._p.setTimeStep(self._timeStep)
        self._p.setGravity(0, 0, -9.81)

        # reset the fuction overridden by the derived class
        self.reset_internal()

        if self._use_real_interface:
            # Upload the loaded robots to parameter server
            rospy.set_param("robot_map", self.getRobotMap())
            print("\n############################################")
            print("### Initializing DigitalTwinFlexAssembly ###")
            for k,v in rospy.get_param("robot_map").items():
                print("\t> robot",k,"with id",v)
            print("\n")

            self.setupRosCommunication()

    def reset_internal(self):
        ''' This function needs overridden by the derived class.
        '''
        pass

    def setupRosCommunication(self):
        ''' This function sets up all the relevant communications
            for the real interface using the ROS transport.
        '''
        try:
            import rospy
            from sensor_msgs.msg import JointState
            from geometry_msgs.msg import Pose
            from geometry_msgs.msg import Point
            from geometry_msgs.msg import Quaternion
            from geometry_msgs.msg import PoseWithCovariance
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

            # Msgs
            from cosima_world_state.msg import ObjectOfInterest, ObjectOfInterestArray
            # Srvs
            from cosima_world_state.srv import RequestTrajectory, RequestTrajectoryResponse
            from cosima_world_state.srv import RequestObjectsOfInterest, RequestObjectsOfInterestResponse
            from cosima_world_state.srv import RequestJointState, RequestJointStateResponse

            # Setup a service to retrieve the objects i.e. clamp poses.
            self.service_get_object_poses = rospy.Service(str(self._node_name)+'/get_object_poses', RequestObjectsOfInterest, self.service_get_object_poses_func)
            print("\n\t> Initialized object poses service\n\t(RequestObjectsOfInterest/Response) on " + str(self._node_name) + "/get_object_poses\n")

            # Setup service to retrieve the robot joint states.
            self.service_joints_state = rospy.Service(str(self._node_name)+'/get_robot_joints_state', RequestJointState, self.service_joints_state_func)
            print("\n\t> Initialized robot joints state service\n\t(RequestJointState/Response) on " + str(self._node_name) + "/get_robot_joints_state\n")
        except ImportError:
            print("ERROR IMPORTING ros 2", file=sys.stderr)

    def service_get_object_poses_func(self, req):
        ''' Service to retrieve the objects i.e. clamp poses.
        '''
        ret = RequestObjectsOfInterestResponse()
        # Is it possible to do cuncurrency in pybullet?
        for body in get_bodies():
            if not self.is_robot_id(body):
                ret_obj = ObjectOfInterest()
                ret_obj.header.stamp = rospy.Time.now()
                ret_obj.name = str(body)
                ret_obj.type = str(p.getBodyInfo(body)[1]) # TODO or also the link [0]?

                ret_obj.pose = PoseWithCovariance()
                # ret_obj.pose.covariance = # TODO

                pos, orn = p.getBasePositionAndOrientation(body)
                # print("Pose of " + str(body) + " name " + str(p.getBodyInfo(body)) + " > ",pose)
                ret_obj.pose.pose.position.x = pos[0]
                ret_obj.pose.pose.position.y = pos[1]
                ret_obj.pose.pose.position.z = pos[2]

                ret_obj.pose.pose.orientation.x = orn[0]
                ret_obj.pose.pose.orientation.y = orn[1]
                ret_obj.pose.pose.orientation.z = orn[2]
                ret_obj.pose.pose.orientation.w = orn[3]

                ret.objects.append(ret_obj)
        return ret
    
    def is_robot_id(self, id):
        ''' Check if the robot name is present in the map.
        '''
        return id in self.getRobotMap().values()

    def service_joints_state_func(self, req):
        ''' Service to retrieve the robot joint states.
        '''
        data = p.getJointStates(int(req.robot_id), get_movable_joints(int(req.robot_id)))
        msg = None
        if data:
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            for d in data:
                msg.name.append(str(int(req.robot_id)))
                msg.position.append(d[0])
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
        return msg

    def getRobotMap(self):
        return self._robot_map

    def getRobotIdByName(self, name):
        return self._robot_map[name]

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

    def updateConstraints(self):
        if self._cm:
            self._cm.updateConstraints()

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
