#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3
import os

from gym_flexassembly.envs.flex_assembly_env import FlexAssemblyEnv
import time

import signal

import rospy
from sensor_msgs.msg import JointState

import pybullet as p

from gym_flexassembly.planning import FlexPlanning

import numpy as np

def main():
    # Load the flex assembly environment
    environment = FlexAssemblyEnv(stepping=False)

    # Disable realtime
    p.setRealTimeSimulation(0)
    # Chose step width
    p.setTimeStep(0.001)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    run = True
    environment.set_running(run)

    pub_debug_joint_commands = rospy.Publisher('debug_joint_commands', JointState, queue_size=1)

    # Init ROS node
    rospy.init_node('test_flex_assembly_env_ros', anonymous=False)
    
    rate = rospy.Rate(1000) # 1000hz

    ### Load plugins ###
    # Load planner
    planner = FlexPlanning(p, environment.getRobots()[0])

    linkWorld = p.getLinkState(environment.getRobots()[0], 6, computeForwardKinematics=1)

    print("linkWorld = ", linkWorld)
    linkWorldPosition = np.array(linkWorld[0])
    print("linkWorldPosition = ", linkWorldPosition)
    linkWorldPosition[0] = linkWorldPosition[0] + 0.05
    print("linkWorldPosition (new) = ", linkWorldPosition)

    orn = p.getQuaternionFromEuler([0.,-1.57,0.]) # TODO
    path = planner.calculatePath(linkWorldPosition, linkWorld[1])

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