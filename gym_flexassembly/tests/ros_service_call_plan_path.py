#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3
import os

import time

import signal

import sys

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cosima_world_state.srv import RequestTrajectory, RequestTrajectoryResponse


import numpy as np

if len(sys.argv) == 4:
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
else:
    sys.exit(1)

rospy.wait_for_service('service_planner')
try:
    add_two_ints = rospy.ServiceProxy('service_planner', RequestTrajectory)
    goal = Pose()
    goal.position.x = x
    goal.position.y = y
    goal.position.z = z

    goal.orientation.x = 0
    goal.orientation.y = 1
    goal.orientation.x = 0
    goal.orientation.w = 0
    resp1 = add_two_ints(goal)
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)