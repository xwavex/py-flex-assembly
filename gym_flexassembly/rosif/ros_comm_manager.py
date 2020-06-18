import numpy as np

import pybullet as p

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import os

from cosima_world_state.srv import AddFrame, AddFrameResponse
from cosima_world_state.srv import AddConstraint, AddConstraintResponse
from cosima_world_state.srv import LaunchSim, LaunchSimResponse
from cosima_world_state.srv import AddObjectURDF, AddObjectURDFResponse
from cosima_world_state.srv import BiBo, BiBoResponse

# import threading

import signal

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data

from gym_flexassembly.envs.env_interface import EnvInterface

# FLEX ASSEMBLY SMARTOBJECTS IMPORTS
from gym_flexassembly.smartobjects.spring_clamp import SpringClamp
        

class ROSCommManager(object):
    """ TODO
    """

    # # Static dict to track instances
    # manager_instances_ = {}
    # manager_instances_lock_ = threading.Lock()

    def __init__(self, sim_id=-1, env=None):
        self._p = p
        # if the sim id is -1 then there is not yet a simulation running
        self._sim_id = sim_id
        self._env = env

        print("ROSCommManager starting")
        rospy.init_node('ROSCommManager', anonymous=False)

        service_launch_sim = rospy.Service('launch_sim', LaunchSim, self.launch_sim)

        service_add_object_urdf = rospy.Service('add_object_urdf', AddObjectURDF, self.add_object_urdf)

        service_add_object_urdf = rospy.Service('add_smart_object', AddObjectURDF, self.add_smart_object)

        service_set_auto_stepping = rospy.Service('set_auto_stepping', BiBo, self.set_auto_stepping)

        service_add_constraint = rospy.Service('add_constraint', AddConstraint, self.add_constraint)

        service_add_frame = rospy.Service('add_frame', AddFrame, self.add_frame)

        # with ROSCommManager.manager_instances_lock_:
        #     ROSCommManager.manager_instances_[self._sim_id] = self
        print("ROSCommManager started")

        rate = rospy.Rate(1000) # 1000hz
        while not rospy.is_shutdown():
            if self._env != None:
                self._env.handle_input_events()
                # self._env.step_sim()
                self._p.stepSimulation()
            rate.sleep()


    def launch_sim(self, req):
        """ Initialize the simulator or connect to an existing one.
        """
        if self._sim_id == -1:
            print("ROSCommManager launching simulator")
            self._env = EnvInterface()
            self._sim_id = self._env.get_client_id()
        print("ROSCommManager connected to simulator " + str(self._sim_id))
        # TODO check if self._env in this case is not None!
        return self._sim_id

    def add_object_urdf(self, req):
        """ Add an object to the simulator via urdf
        """
        # Disable rendering
        # self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 0)
        # object
        print("Trying to add object with fixed base " + str(req.fixed_base))
        object_id = self._p.loadURDF(req.urdf_file_name, useFixedBase=req.fixed_base, flags = self._p.URDF_USE_INERTIA_FROM_FILE)
        self._p.resetBasePositionAndOrientation(object_id, [req.frame_pose.position.x, req.frame_pose.position.y, req.frame_pose.position.z], [req.frame_pose.orientation.x,req.frame_pose.orientation.y,req.frame_pose.orientation.z,req.frame_pose.orientation.w])
        # Enable rendering again
        # self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

        # table_offset_world_x = -0.85
        # table_offset_world_y = 0
        # table_offset_world_z = 0
        
        # workpiece_1_offset_table_x = 0.60
        # workpiece_1_offset_table_y = 0.20
        # workpiece_1_offset_table_z = 0.75
        # workpiece_1_offset_world = [table_offset_world_x + workpiece_1_offset_table_x, table_offset_world_y + workpiece_1_offset_table_y, table_offset_world_z + workpiece_1_offset_table_z]
        # workpiece_1 = SpringClamp(pos=workpiece_1_offset_world)

    def add_smart_object(self, req):
        """ Add a smart object to the simulator
        """
        # Disable rendering
        # self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 0)
        # object

        if req.urdf_file_name == "SpringClamp":
            print("Trying to add smart object " + str(req.urdf_file_name))
            sObject = SpringClamp(pos=[req.frame_pose.position.x, req.frame_pose.position.y, req.frame_pose.position.z], orn=[req.frame_pose.orientation.x,req.frame_pose.orientation.y,req.frame_pose.orientation.z,req.frame_pose.orientation.w])
        # Enable rendering again
        # self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

        # TODO we need a manager that takes care of storing this thing!!!

        return sObject.getModelId()

    def set_auto_stepping(self, req):
        """ Activate or deactivate auto_stepping
        """
        if self._env == None:
            return False
        
        self._env.set_running(req.input)
        return True

    # def add_multibody:
    #     pass

    # def get_objects(self, req):
    #     """ Get all objects from the simulator
    #     """
    #     pass

    def add_constraint(self, req):
        print("ROSCommManager AddConstraintResponse: " + str(req.anchor_id))
        return AddConstraintResponse(req.anchor_id)

    def add_frame(self, req):
        if self._env == None:
            return -1337
        
        myid = self._env.getFrameManager().createFrame(req.frame_name, pos=[req.frame_pose.position.x,req.frame_pose.position.y,req.frame_pose.position.z], orn=[req.frame_pose.orientation.x,req.frame_pose.orientation.y,req.frame_pose.orientation.z,req.frame_pose.orientation.w], ref_id=req.ref_frame_id)
        print("ROSCommManager AddFrameResponse: name: " + str(req.frame_name) + ", ref id: " + str(req.ref_frame_id) + ", myid: " + str(myid))
        return myid

    # def get_frames(self, req):
    #     pass

    # def __del__(self):
    #     with ROSCommManager.manager_instances_lock_:
    #         ROSCommManager.manager_instances_.pop(self._sim_id)

if __name__ == "__main__":
    r = ROSCommManager()
    try:
        print("ROSCommManager waiting for CTRL-C")
        signal.pause()
    except (KeyboardInterrupt, SystemExit):
        print("Shutting down...")