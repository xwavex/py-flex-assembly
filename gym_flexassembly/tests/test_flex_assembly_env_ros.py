#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3
import os

from gym_flexassembly.envs.flex_assembly_env import FlexAssemblyEnv
import time

import signal

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cosima_world_state.srv import RequestTrajectory, RequestTrajectoryResponse

import pybullet as p

from gym_flexassembly.planning import FlexPlanning

import numpy as np

def plan_fnc(req):
    global planner
    path = planner.calculatePath([req.goal.position.x, req.goal.position.y, req.goal.position.z], [req.goal.orientation.x,req.goal.orientation.y,req.goal.orientation.z,req.goal.orientation.w])
    print("Planned to ", [req.goal.position.x, req.goal.position.y, req.goal.position.z])
    return JointTrajectory()

def main():
    # Load the flex assembly environment
    # global environment
    environment = FlexAssemblyEnv(stepping=False)

    # Disable realtime
    p.setRealTimeSimulation(0)
    # Chose step width
    p.setTimeStep(0.001)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Upload robots to parameter server
    rospy.set_param("robot_map", environment.getRobotMap())

    print("\n##########################")
    print("### INITIALIZED ROBOTS ###")
    for k,v in rospy.get_param("robot_map").items():
        print("    > robot",k,"with id",v)
    print("##########################\n")

    run = True
    environment.set_running(run)

    pub_debug_joint_commands = rospy.Publisher('debug_joint_commands', JointState, queue_size=1)

    # Init ROS node
    rospy.init_node('test_flex_assembly_env_ros', anonymous=False)
    
    rate = rospy.Rate(1000) # 1000hz

    ### Load plugins ###
    # Load planner
    global planner
    planner = FlexPlanning(p, list(environment.getRobotMap().values())[0], environment.get_shadow_client_id())
    service_planner = rospy.Service('service_planner', RequestTrajectory, plan_fnc)

    while not rospy.is_shutdown():
        if run:
            read_data_arr = environment.getDebugJointCommands()
            if read_data_arr:
                length = len(read_data_arr)
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                for en in read_data_arr:
                    msg.name.append("")
                    msg.position.append(en)
                    msg.velocity.append(0.0)
                    msg.effort.append(0.0)
                pub_debug_joint_commands.publish(msg)
        # Process Plugins
        # 
        # 
        p.stepSimulation();

        rate.sleep()
    try:
        signal.pause()
    except (KeyboardInterrupt, SystemExit):
        print("Shutting down...")

if __name__ == "__main__":
    main()