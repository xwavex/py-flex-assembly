#!/home/dwigand/code/cogimon/CoSimA/pyBullet/vPyBullet/bin/python3
import os

from gym_flexassembly.envs.flex_polishing_env import FlexPolishingEnv
import time

import signal


def main():

    environment = FlexPolishingEnv()

    # motorsIds = []
    # #motorsIds.append(environment._p.addUserDebugParameter("posX",0.4,0.75,0.537))
    # #motorsIds.append(environment._p.addUserDebugParameter("posY",-.22,.3,0.0))
    # #motorsIds.append(environment._p.addUserDebugParameter("posZ",0.1,1,0.2))
    # #motorsIds.append(environment._p.addUserDebugParameter("yaw",-3.14,3.14,0))
    # #motorsIds.append(environment._p.addUserDebugParameter("fingerAngle",0,0.3,.3))

    # dv = 0.01
    # motorsIds.append(environment._p.addUserDebugParameter("posX", -dv, dv, 0))
    # motorsIds.append(environment._p.addUserDebugParameter("posY", -dv, dv, 0))
    # motorsIds.append(environment._p.addUserDebugParameter("posZ", -dv, dv, 0))
    # motorsIds.append(environment._p.addUserDebugParameter("yaw", -dv, dv, 0))
    # motorsIds.append(environment._p.addUserDebugParameter("fingerAngle", 0, 0.3, .3))

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