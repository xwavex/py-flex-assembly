"""This is the planning component.
:Author:
    `Michael Wojtynek <mwojtynek@cor-lab.de>`
    `Dennis Leroy Wigand <dwigand@cor-lab.de>`
"""

import pybullet as p
import pybullet as pi
import pybullet_data
import pybullet_planning as planning
import numpy as np
import time
import os
from pybullet_planning.utils import INF
from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion, get_distance_fn, get_extend_fn
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info
from pybullet_planning.interfaces.robots import get_movable_joints, set_joint_positions, get_link_pose, get_custom_limits, get_joint_limits, body_from_name, get_body_name, get_joint_names, get_joint_positions, get_joint_q_index, get_joint
import pybullet_planning.interfaces.robots.joint as jt
from pybullet_planning.interfaces.env_manager import create_box

from pybullet_planning.interfaces.robots.body import get_bodies

from pybullet_planning import set_joint_positions
# TODO clean up the imports

# from pybullet_planning.utils import CLIENT, set_client

# LOAD SCENARIO IMPORT
from gym_flexassembly.envs.flex_assembly_env import FlexAssemblyEnv

from gym_flexassembly.planning import FlexPlanning

import os
import sys
import signal

# ROS import
import rospy
# ROS types imports
# Msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cosima_world_state.msg import ObjectOfInterest, ObjectOfInterestArray
from sensor_msgs.msg import JointState
# Srvs
from cosima_world_state.srv import RequestObjectsOfInterest, RequestObjectsOfInterestResponse
from cosima_world_state.srv import RequestTrajectory, RequestTrajectoryResponse
from cosima_world_state.srv import RequestJointState, RequestJointStateResponse

from klampt.model import trajectory

class FlexPlanningROS(object):
    def __init__(self, name="flex_planning_ros"):
        # Load the scenario
        self.environment = FlexAssemblyEnv(stepping=False, gui=False, direct=True, use_real_interface=False, static=True) # For debuging visualization turn gui=True on.
        p.setRealTimeSimulation(0)
        p.setTimeStep(0.001)

        # Init ROS node
        rospy.init_node(name, anonymous=False)
        self.rate = rospy.Rate(1000) # 1000hz

        # Upload robots to parameter server
        self.robotMap = rospy.get_param("robot_map")

        # Load planner
        self.planner = FlexPlanning(p, list(self.robotMap.values())[0])
        self.service_planner = rospy.Service(name+'/plan', RequestTrajectory, self.plan_fnc)

        # Create service client to retrieve the joint angles and robot base pose from the digital twin
        print("Waiting for service env/get_object_poses...")
        rospy.wait_for_service("env/get_object_poses")
        self.client_get_object_poses = rospy.ServiceProxy("env/get_object_poses", RequestObjectsOfInterest)
        print(" > Initialized client for env/get_object_poses")

        # Create service client to retrieve the clamp poses from the digital twin
        print("Waiting for service env/get_robot_joints_state...")
        rospy.wait_for_service("env/get_robot_joints_state")
        self.client_get_robot_joints_state = rospy.ServiceProxy("env/get_robot_joints_state", RequestJointState)
        print(" > Initialized client for env/get_robot_joints_state")

        # Create publisher for trajectory
        self.pub_traj = rospy.Publisher(name+'/traj_setpoints', JointTrajectoryPoint, queue_size=1)
        print(" > Initialized publisher on " + name + "/traj_setpoints")

        print("\n > Initialized FlexPlanningROS with\n")
        print("\t planning service on: "+name+"/plan\n")
        print("\nRunning... (Ctrl-c to exit)\n")

        while not rospy.is_shutdown():
            p.stepSimulation()
            self.rate.sleep()
        print("Shutting down...")

    def plan_fnc(self, req):
        # Retrieve the object i.e. clamp poses from the digital twin
        try:
            object_poses = self.client_get_object_poses()
            for op in object_poses.objects:
                self.planner.updateObjectPoses(int(op.name), [op.pose.pose.position.x, op.pose.pose.position.y, op.pose.pose.position.z], [op.pose.pose.orientation.x, op.pose.pose.orientation.y, op.pose.pose.orientation.z, op.pose.pose.orientation.w])
        except rospy.ServiceException as e:
            print("Service call env/get_object_poses failed: %s"%e)

        # Retrieve the object i.e. clamp poses from the digital twin
        for robot_id in self.robotMap.values():
            try:
                robot_joints_state = self.client_get_robot_joints_state(str(robot_id))
                # set robot joint configuration
                self.planner.updateRobotConfiguration(robot_id, robot_joints_state.state.position)

            except rospy.ServiceException as e:
                print("Service call env/get_robot_joints_state for robot_id " + str(robot_id) + " failed: %s"%e)

        print("Planning to ", [req.goal.position.x, req.goal.position.y, req.goal.position.z])
        path = self.planner.calculatePath([req.goal.position.x, req.goal.position.y, req.goal.position.z], [req.goal.orientation.x,req.goal.orientation.y,req.goal.orientation.z,req.goal.orientation.w])
        if not path:
            print("No path returned!", file=sys.stderr)
            return None

        # print("\n\nPATH:",path)

        milestones=[]
        for entry in path:
            milestones.append(list(entry))

        print("\n\nPATH:",milestones)

        

        traj = trajectory.Trajectory(milestones=milestones)
        traj_timed = trajectory.path_to_trajectory(traj,vmax=2,amax=4)
        traj = traj_timed
        dt = 0.01  #approximately a 100Hz control loop
        t0 = time.time()
        while True:
            t = time.time()-t0
            if t > traj.endTime():
                break
            qklampt = traj.eval(t)
            dqklampt = traj.eval(t)
            qrobot = self.convert_klampt_config(qklampt)
            dqrobot = self.convert_klampt_velocity(dqklampt)
            # SEND
            jt_tmp = JointTrajectoryPoint(positions=qrobot, velocities=dqrobot)
            self.pub_traj.publish(jt_tmp)
            # 
            time.sleep(dt)


        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.header.frame_id = "world" # TODO?

        # Get all joints
        num_involved_joints = len(list(self.planner.getInvolvedRobotJointNames()))
        for j in self.planner.getInvolvedRobotJointNames():
            jt.joint_names.append(str(j))

        jt.points = []
        for entry in path:
            # jt.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*num_involved_joints, time_from_start=rospy.Duration(0.0)))
            jt.points.append(JointTrajectoryPoint(positions=entry, velocities=[0]*num_involved_joints)) # TODO time_from_start
            
        return jt

    def convert_klampt_config(self, q):
        """Converts klampt config to my robot's config, e.g., extract DOFs,
        convert units, account for joint offsets.

        Right now, does a straight pass-through.
        """
        return q

    def convert_klampt_velocity(self, dq):
        """Converts klampt velocity to my robot's velocity, e.g., extract DOFs,
        convert units.

        Right now, does a straight pass-through.
        """
        return dq

if __name__ == "__main__":
    topic="flex_planning_ros"

    if len(sys.argv) == 2:
        tmp = str(sys.argv[1])
        if tmp == "":
            print("Invalid component name specified! Should not be empty.", file=sys.stderr)
            sys.exit(1)
        if '-' in tmp:
            print("Invalid component name specified! Should not contain \'-\'.", file=sys.stderr)
            sys.exit(1)
        if tmp[:1].isdigit():
            print("Invalid component name specified! Should not start with a number.", file=sys.stderr)
            sys.exit(1)

        if tmp.startswith("/"):
            print("Invalid component name specified! Should not start with \'/\'.", file=sys.stderr)
            sys.exit(1)

        topic = tmp
    elif len(sys.argv) > 2:
        print("Invalid arguments!", file=sys.stderr)
        print("Usage: python3 -m gym_flexassembly.planning.flex_planning_ros [name]\n")
        sys.exit(1)

    FlexPlanningROS(name=topic)