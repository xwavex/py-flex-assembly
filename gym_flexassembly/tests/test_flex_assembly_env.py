#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3
import os

from gym_flexassembly.envs.flex_assembly_env import FlexAssemblyEnv
import time

import signal

import rospy



def main():

    environment = FlexAssemblyEnv()

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