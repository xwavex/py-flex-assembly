#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3
import os

from gym_flexassembly.envs.flex_assembly_env import FlexAssemblyEnv
import time

import signal

# import rospy

import pybullet as p

def main():
    environment = FlexAssemblyEnv(stepping=False)

    # Disable realtime
    p.setRealTimeSimulation(0)
    # Chose step width
    p.setTimeStep(0.001)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)


    environment.set_running(True)
    while(1):
        environment.step([])

    # done = False
    # while (not done):

    #     action = []
    #     for motorId in motorsIds:
    #         action.append(environment._p.readUserDebugParameter(motorId))

    #     state, reward, done, info = environment.step2(action)
    #     obs = environment.getExtendedObservation()

    try:
        signal.pause()
    except (KeyboardInterrupt, SystemExit):
        print("Shutting down...")


if __name__ == "__main__":
    main()