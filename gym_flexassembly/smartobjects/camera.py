import os, inspect

import pybullet as p
import numpy as np
import copy
import math

import sys
import threading

from time import perf_counter

class Camera:
    def __init__(self, pybullet, settings, name, model_id, use_real_interface=True):
        self._settings = settings
        self._name = name
        self._model_id = model_id
        self._use_real_interface = use_real_interface
        self._terminate_thread = False
        self._p = pybullet

        self._depth = []
        self._rgb = []

        self._lock = threading.Lock()

        self._new_data_available = False

        self._thread = None

        self.setPose(self._settings['pos'], self._settings['orn'])

        self._start_time = perf_counter()

        # self._wait_frame_rate = 1.0 / self._settings['framerate']
        self._wait_frame_rate = 1.0 / 5.0

        self._view_matrix = self._p.computeViewMatrix(self._settings['pos'],
                                                    self._settings['target_pos'],
                                                    self._settings['up'])
        self._projection_matrix = self._p.computeProjectionMatrixFOV(self._settings['fov'],
                                                                    self._settings['width'] / self._settings['height'],
                                                                    self._settings['near'],
                                                                    self._settings['far'])
        self._near = self._settings['near']
        self._far = self._settings['far']

        if self._use_real_interface:
            # load (optional) ROS imports if the real interface should be mirrored
            try:
                import rospy
                from std_msgs.msg import Header
                from geometry_msgs.msg import Pose
                from sensor_msgs.msg import Image
                import cv_bridge

                # not sure if we really need this
                global rospy, cv_bridge, Pose, Image, Header

                # print(self._p.isNumpyEnabled())

                # create ros publisher
                self._pub_depth = rospy.Publisher('~camera/' + self._name + '/depth/image_raw', Image, queue_size=1)
                self._pub_color = rospy.Publisher('~camera/' + self._name + '/color/image_raw', Image, queue_size=1)
                # instantiate the cv bridge
                self._bridge = cv_bridge.CvBridge()

                self._rate = rospy.Rate(self._settings['framerate'])

                print("\n\t> Initialized camera " + str(self._name) + " depth image \n\t(sensor_msgs.Image) on ~camera/"+str(self._name)+"/depth/image_raw publisher\n")

                print("\n\t> Initialized camera " + str(self._name) + " color image \n\t(sensor_msgs.Image) on ~camera/"+str(self._name)+"/color/image_raw publisher\n")

                def thread_func():
                    while not self._terminate_thread:

                        if self._new_data_available:
                            # create a header for the messages
                            header = Header()
                            header.stamp = rospy.Time.now()

                            self._lock.acquire()
                            try:
                                img_depth = self._bridge.cv2_to_imgmsg(self._depth, 'passthrough')
                                img_color = self._bridge.cv2_to_imgmsg(self._rgb, 'passthrough')
                            except cv_bridge.CvBridgeError as e:
                                print('Could not convert CV image to ROS msg: ' + str(e))
                            self._lock.release()

                            # publish the images
                            img_depth.header = header
                            img_depth.header.frame_id = 'depth_image'
                            self._pub_depth.publish(img_depth)

                            img_color.header = header
                            img_color.header.frame_id = 'color_image'
                            self._pub_color.publish(img_color)

                            self._new_data_available = False

                        self._rate.sleep()

                # Start the ROS publishing thread
                self._thread = threading.Thread(target=thread_func, daemon=True)
                self._thread.start()
            
            except ImportError:
                print("ERROR IMPORTING ros camera", file=sys.stderr)

    def update(self):
        if not self._terminate_thread:
            if (perf_counter() - self._start_time) < self._wait_frame_rate:
                return
            self._start_time = perf_counter()
            # get a new camera image
            _, _, rgba, depth_buffer, _ = self._p.getCameraImage(self._settings['width'],
                                                                self._settings['height'],
                                                                self._view_matrix,
                                                                self._projection_matrix,
                                                                shadow=True,
                                                                renderer=self._p.ER_BULLET_HARDWARE_OPENGL)
            self._lock.acquire()
            self._depth = self._far * self._near / (self._far - (self._far - self._near) * depth_buffer)
            self._rgb = rgba[:, :, :3]
            self._lock.release()

            self._new_data_available = True

    def getModelId(self):
        return self._model_id

    def setPose(self, pos, orn):
        self._settings['pos'] = pos
        self._settings['orn'] = orn
        p.resetBasePositionAndOrientation(self._model_id, self._settings['pos'], self._settings['orn'])

    def getPose(self):
        # TODO this does not support moving cameras yet
        return self._settings['pos'], self._settings['orn']

    def reset(self):
        pass

    def getUUid(self):
        return self._model_id

    def getObservation(self):
        observation = []
        return observation

    def terminate(self):
        self._terminate_thread = True
        self._thread.join()

        self._pub_color.unregister()
        self._pub_depth.unregister()

    def __del__(self):
        self.terminate()

__all__ = ['Camera']