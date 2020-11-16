""" Simulating a 2 finger prismatic gripper.
    Connect to a spawned model and supply the names of the two finger links.
:Author:
  `Dennis Leroy Wigand <dwigand@cor-lab.de>`
"""

import os, inspect

import pybullet as p
import numpy as np
import copy
import math
import sys
import threading

class Prismatic2FingerGripperPlugin:
    def __init__(self, connect_to_loaded_model_id, name, finger_1_joint_name, finger_2_joint_name, use_real_interface=True, closing_vel=-0.1, opening_vel=0.1, max_force=800.0):
        self._use_real_interface = use_real_interface
        self._name = name
        self._connect_to_loaded_model_id = connect_to_loaded_model_id
        self._finger_1_joint_name = finger_1_joint_name
        self._finger_2_joint_name = finger_2_joint_name
        self._finger_1_joint_index = -1
        self._finger_2_joint_index = -1

        # upper and lower joint limits
        self._closing_vel = closing_vel
        self._opening_vel = opening_vel
        self._max_force = max_force

        self.convergence_threshold = 0.001 # TODO

        self._finger_1_joint_state = []
        self._finger_2_joint_state = []

        self._convergence_event = threading.Event()
        self._lock = threading.Lock()

        # (OPTIONAL) ROS IMPORTS
        if self._use_real_interface:
            try:
                import rospy
                from std_msgs.msg import Header
                from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
                # not sure if we really need this
                global rospy, Empty, EmptyResponse, Trigger, TriggerResponse, Header

                # Setup a service to open the gripper.
                self.service_open = rospy.Service("/"+str(self._name)+"/open_gripper", Empty, self.open_gripper)
                print("\n\t> Initialized gripper "+str(self._name)+" open service\n\t(Empty/Response) on /" + str(self._name) + "/open_gripper\n")

                # Setup a service to close the gripper.
                self.service_close = rospy.Service("/"+str(self._name)+"/close_gripper", Empty, self.close_gripper)
                print("\n\t> Initialized gripper "+str(self._name)+" close service\n\t(Empty/Response) on /" + str(self._name) + "/close_gripper\n")

                # Setup a service to check the convergence status of the gripper.
                self.service_converged = rospy.Service("/"+str(self._name)+"/converged_gripper", Trigger, self.converged_gripper)
                print("\n\t> Initialized gripper "+str(self._name)+" converged service\n\t(Trigger/Response) on /" + str(self._name) + "/converged_gripper\n")

            except ImportError:
                print("ERROR IMPORTING ros gripper connected to " + str(connect_to_loaded_model_id), file=sys.stderr)

        self.reset()

    def converged_gripper(self, req):
        ret = False
        self._lock.acquire()
        ret = math.fabs(self._finger_1_joint_state[1]) < self.convergence_threshold and math.fabs(self._finger_2_joint_state[1]) < self.convergence_threshold
        self._lock.release()
        if ret:
            return TriggerResponse(
                success=True,
                message="Converged"
            )
        return TriggerResponse(
            success=False,
            message="Moving"
        )
    
    def open_gripper(self, req):
        self._convergence_event.clear()
        self.open()
        event_is_set = self._convergence_event.wait(10.0)
        if not event_is_set:
            return None
        return []

    def close_gripper(self, req):
        self._convergence_event.clear()
        self.close()
        event_is_set = self._convergence_event.wait(10.0)
        if not event_is_set:
            return None
        return []

    def reset(self):
        self._convergence_event.clear()
        self._already_converged = False
        self._finger_1_joint_state = []
        self._finger_2_joint_state = []
        # Find indices associated with links names
        total_joints = p.getNumJoints(self._connect_to_loaded_model_id)
        self.motorNames = []
        self.motorIndices = []
        self.zeroForces = []
        for i in range(total_joints):
            jointInfo = p.getJointInfo(self._connect_to_loaded_model_id, i)
            if str(self._finger_1_joint_name) == (str(jointInfo[1].decode('UTF-8'))):
                self._finger_1_joint_index = i
            elif str(self._finger_2_joint_name) == (str(jointInfo[1].decode('UTF-8'))):
                self._finger_2_joint_index = i
        
        if self._finger_1_joint_index == -1 or self._finger_2_joint_index == -1:
            # TODO error message
            return False


        self.cmd = self._opening_vel

        # Take care of the gripper mechanism reset
        p.setJointMotorControl2(bodyIndex=self._connect_to_loaded_model_id, jointIndex=self._finger_1_joint_index, controlMode=p.VELOCITY_CONTROL, targetPosition=0.0, targetVelocity=self.cmd, force=self._max_force)
        p.setJointMotorControl2(bodyIndex=self._connect_to_loaded_model_id, jointIndex=self._finger_2_joint_index, controlMode=p.VELOCITY_CONTROL, targetPosition=0.0, targetVelocity=self.cmd, force=self._max_force)

    def update(self):
        # TODO calculate convergence based on velocity and not position
        self._lock.acquire() # TODO perhaps we should spin here and not block the whole simulation update indirectly...
        self._finger_1_joint_state = p.getJointState(self._connect_to_loaded_model_id, self._finger_1_joint_index)
        self._finger_2_joint_state = p.getJointState(self._connect_to_loaded_model_id, self._finger_2_joint_index)
        self._lock.release()

        if math.fabs(self._finger_1_joint_state[1]) < self.convergence_threshold and math.fabs(self._finger_2_joint_state[1]) < self.convergence_threshold:
            if not self._already_converged:
                # Converged: set event
                self._already_converged = True
                if self._use_real_interface:
                    self._convergence_event.set()
            else:
                if self._already_converged:
                    # Moving again
                    self._already_converged = False

        p.setJointMotorControl2(bodyIndex=self._connect_to_loaded_model_id, jointIndex=self._finger_1_joint_index, controlMode=p.VELOCITY_CONTROL, targetPosition=0.0, targetVelocity=self.cmd, force=self._max_force)
        p.setJointMotorControl2(bodyIndex=self._connect_to_loaded_model_id, jointIndex=self._finger_2_joint_index, controlMode=p.VELOCITY_CONTROL, targetPosition=0.0, targetVelocity=self.cmd, force=self._max_force)

    def close(self):
        self.cmd = self._closing_vel

    def open(self):
        self.cmd = self._opening_vel

__all__ = ['Prismatic2FingerGripperPlugin']