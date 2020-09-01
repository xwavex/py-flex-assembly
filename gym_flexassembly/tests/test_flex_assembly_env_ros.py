#!/usr/bin/python3
import os

from gym_flexassembly.envs.flex_assembly_env import FlexAssemblyEnv
import time

import signal

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

# PyBullet
import pybullet as p
# PyBullet Planning
from pybullet_planning.interfaces.robots.body import get_bodies
from pybullet_planning.interfaces.robots import get_movable_joints

import numpy as np

class DigitalTwinFlexAssembly(object):
    def __init__(self, node_name):
        # Load the flex assembly environment
        # global environment
        self.environment = FlexAssemblyEnv(stepping=False)

        # Disable realtime
        p.setRealTimeSimulation(0)
        # Chose step width
        p.setTimeStep(0.001)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # Init ROS node
        rospy.init_node(node_name, anonymous=False)
        
        rate = rospy.Rate(250) # 250hz

        # Upload robots to parameter server
        rospy.set_param("robot_map", self.environment.getRobotMap())

        print("\n############################################")
        print("### Initializing DigitalTwinFlexAssembly ###")
        for k,v in rospy.get_param("robot_map").items():
            print("\t> robot",k,"with id",v)
        print("\n")

        run = True
        self.environment.set_running(run)

        # Setup a service to retrieve the objects i.e. clamp poses.
        self.service_get_object_poses = rospy.Service(str(node_name)+'/get_object_poses', RequestObjectsOfInterest, self.service_get_object_poses_func)
        print("\n\t> Initialized object poses service\n\t(RequestObjectsOfInterest/Response) on " + str(node_name) + "/get_object_poses\n")

        # Setup service to retrieve the robot joint states.
        self.service_joints_state = rospy.Service(str(node_name)+'/get_robot_joints_state', RequestJointState, self.service_joints_state_func)
        print("\n\t> Initialized robot joints state service\n\t(RequestJointState/Response) on " + str(node_name) + "/get_robot_joints_state\n")

        print("\n############################################\n")

        # ######################### TODO Plugins
        # Gripper plugin
        gripper_1 = Prismatic2FingerGripperPlugin(self.environment.getRobotMap()[""], finger_1_joint_name, finger_2_joint_name)

        while not rospy.is_shutdown():
            # if run:
                # # Publish (debug) robot joint states
                # jp = self.environment.getDebugJointCommands()
                # if jp:
                #     # length = len(jp)
                #     msg = JointState()
                #     msg.header.stamp = rospy.Time.now()
                #     for en in read_data_arr:
                #         msg.name.append("")
                #         msg.position.append(en)
                #         msg.velocity.append(0.0)
                #         msg.effort.append(0.0)

            # p.stepSimulation() # Only use this is we are not triggered externally...
            gripper_1.update()

            rate.sleep()
        try:
            signal.pause()
        except (KeyboardInterrupt, SystemExit):
            print("Shutting down...")

    def is_robot_id(self, id):
        return id in self.environment.getRobotMap().values()

    def service_joints_state_func(self, req):
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

    def service_get_object_poses_func(self, req):
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

if __name__ == "__main__":
    dt = DigitalTwinFlexAssembly('dt')