import numpy as np

import pybullet as p

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import tf2_ros
from geometry_msgs.msg import TransformStamped

import os

from cosima_world_state.srv import AddFrame, AddFrameResponse
from cosima_world_state.srv import AddConstraint, AddConstraintResponse
from cosima_world_state.srv import LaunchSim, LaunchSimResponse
from cosima_world_state.srv import AddObjectURDF, AddObjectURDFResponse
from cosima_world_state.srv import BiBo, BiBoResponse

from cosima_world_state.msg import MultiBodyIds
# from cosima_world_state.srv import AddMultiBody, AddMultiBodyResponse

# import threading

import signal

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data

from gym_flexassembly.envs.env_interface import EnvInterface

# FLEX ASSEMBLY SMARTOBJECTS IMPORTS
from gym_flexassembly.smartobjects.spring_clamp import SpringClamp
from gym_flexassembly.robots.kuka_iiwa import KukaIIWA, KukaIIWA7, KukaIIWA14

from gym_flexassembly.constraints.maxwell_constraint import MaxwellConstraint


class ROSCommManager(object):
    """ DLW TODO
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

        # service_add_multibody = rospy.Service('add_multibody', AddMultiBody, self.add_multibody)

        service_add_object_urdf = rospy.Service('add_smart_object', AddObjectURDF, self.add_smart_object)

        service_set_auto_stepping = rospy.Service('set_auto_stepping', BiBo, self.set_auto_stepping)

        service_add_contact_constraint = rospy.Service('add_contact_constraint', AddConstraint, self.add_contact_constraint)

        service_add_frame = rospy.Service('add_frame', AddFrame, self.add_frame)

        service_add_body_frame = rospy.Service('add_body_frame', AddFrame, self.add_body_frame)

        service_run_simulation = rospy.Service('run_simulation', BiBo, self.run_simulation)

        self.debugcount = 0
        self.debugc = None

        # FRAME BROADCASTER
        self._fb = tf2_ros.TransformBroadcaster()

        # with ROSCommManager.manager_instances_lock_:
        #     ROSCommManager.manager_instances_[self._sim_id] = self
        print("ROSCommManager started")

        self.run = False

        rate = rospy.Rate(1000) # 1000hz
        while not rospy.is_shutdown():
            if self._env != None and self.run:
                self._env.handle_input_events()

                self._env.getFrameManager().updateFramePoses()

                self._env.updateConstraints()
                # self._env.step_sim()
                self._p.stepSimulation()
            rate.sleep()

    def run_simulation(self, req):
        """ Trigger to continuously run the simulator
        """
        print("Run the simulator continuously!")
        self.run = True
        return self.run

    def launch_sim(self, req):
        """ Initialize the simulator or connect to an existing one.
        """
        if self._sim_id == -1:
            print("ROSCommManager launching simulator")
            self._env = EnvInterface(ros_frame_broadcaster=self._fb)
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
        
        ret = MultiBodyIds()
        ret.id = object_id
        ret.joint_ids = []
        ret.joint_names = []

        # Disable motors
        for j in range(self._p.getNumJoints(object_id)):
            ji = self._p.getJointInfo(object_id, j)
            jointType = ji[2]
            ret.joint_ids.append(int(ji[0]))
            ret.joint_names.append(str(ji[1].decode('UTF-8')))
            if (jointType == self._p.JOINT_SPHERICAL):
                # print("Joint "+str(j)+" as JOINT_SPHERICAL")
                self._p.setJointMotorControlMultiDof(object_id, j, self._p.POSITION_CONTROL, targetPosition=[0, 0, 0, 1], targetVelocity=[0,0,0], positionGain=0, velocityGain=1, force=[0,0,0])
                self._p.setJointMotorControlMultiDof(object_id, j, self._p.TORQUE_CONTROL, force=[0,0,0])
            elif (jointType==self._p.JOINT_PRISMATIC or jointType==self._p.JOINT_REVOLUTE):
                # print("Joint "+str(j)+" as JOINT_PRISMATIC or JOINT_REVOLUTE")
                self._p.setJointMotorControl2(object_id, j, self._p.VELOCITY_CONTROL, targetVelocity=0, force=0)
                self._p.setJointMotorControl2(object_id, j, self._p.TORQUE_CONTROL, force=0.0)
            else:
                print("Joint Type " + str(jointType) + " is not yet supported!")
        
        # Enable rendering again
        # self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

        # Set collision properly
        # Collision
        #                      0x10
        collisionFilterGroup = 0x10
        #                      0x11
        collisionFilterMask =  0x11

        p.setCollisionFilterGroupMask(object_id, -1, collisionFilterGroup, collisionFilterMask)
        for i in range(p.getNumJoints(object_id)):
            p.setCollisionFilterGroupMask(object_id, i, collisionFilterGroup, collisionFilterMask)

        return ret

    # def add_multibody(self, req):
    #     """ Add an object to the simulator
    #     """
    #     visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
    #                                 fileName=req.visual_mesh_file_name,
    #                                 rgbaColor=[1, 1, 1, 1],
    #                                 specularColor=[0.4, .4, 0],
    #                                 visualFramePosition=[0,0,0],
    #                                 meshScale=[1,1,1])
    #     collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
    #                                       fileName=req.collision_mesh_file_name,
    #                                       collisionFramePosition=[0,0,0],
    #                                       meshScale=[1,1,1])

    #     schunk = p.createMultiBody(baseMass=0.6,
    #                     baseInertialFramePosition=[0.00078059, -0.00070996, 0.04726637],
    #                     baseCollisionShapeIndex=collisionShapeId,
    #                     baseVisualShapeIndex=visualShapeId,
    #                     basePosition=[0,0,0.5],
    #                     useMaximalCoordinates=False

    #     object_id = self._p.loadURDF(req.urdf_file_name, useFixedBase=req.fixed_base, flags = self._p.URDF_USE_INERTIA_FROM_FILE)
    #     self._p.resetBasePositionAndOrientation(object_id, [req.frame_pose.position.x, req.frame_pose.position.y, req.frame_pose.position.z], [req.frame_pose.orientation.x,req.frame_pose.orientation.y,req.frame_pose.orientation.z,req.frame_pose.orientation.w])
        

    def add_smart_object(self, req):
        """ Add a smart object to the simulator
        """
        # Disable rendering
        # self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 0)
        # object

        ret = MultiBodyIds()
        if req.urdf_file_name == "SpringClamp":
            print("Trying to add SpringClamp as smart object " + str(req.urdf_file_name))
            sObject = SpringClamp(pos=[req.frame_pose.position.x, req.frame_pose.position.y, req.frame_pose.position.z], orn=[req.frame_pose.orientation.x,req.frame_pose.orientation.y,req.frame_pose.orientation.z,req.frame_pose.orientation.w])
            ret.id = sObject.getModelId()
        elif req.urdf_file_name == "KukaIIWA7":
            print("Trying to add KukaIIWA7 as smart object " + str(req.urdf_file_name))
            sObject = KukaIIWA7()
            p.resetBasePositionAndOrientation(sObject.getUUid(), [req.frame_pose.position.x, req.frame_pose.position.y, req.frame_pose.position.z], [req.frame_pose.orientation.x,req.frame_pose.orientation.y,req.frame_pose.orientation.z,req.frame_pose.orientation.w])
            # Collision
            #                      0x10
            collisionFilterGroup = 0x10
            #                      0x11
            collisionFilterMask =  0x11

            self._p.setCollisionFilterGroupMask(sObject.getUUid(), -1, collisionFilterGroup, collisionFilterMask)
            for i in range(self._p.getNumJoints(sObject.getUUid())):
                self._p.setCollisionFilterGroupMask(sObject.getUUid(), i, collisionFilterGroup, collisionFilterMask)
            ret.id = sObject.getUUid()
        
        # Enable rendering again
        # self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING, 1)

        # TODO we need a manager that takes care of storing this thing!!!
        
        print("Added id for smart body " + str(ret.id))
        return ret

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

    def add_contact_constraint(self, req):
        # find frame with id
        f = self._env.getFrameManager.getFrameById(req.target_id)
        c = self._env.getConstraintManager().addContactConstraint(f, axis=[req.axis_tx,req.axis_ty,req.axis_tz,req.axis_rx,req.axis_ry,req.axis_rz], direction=[req.direction_tx,req.direction_ty,req.direction_tz,req.direction_rx,req.direction_ry,req.direction_rz])
        print("ROSCommManager add_contact_constraint (AddConstraintResponse) " + str(c.getId()))
        return c.getId()

    def add_frame(self, req):
        if self._env == None:
            return -1337
        
        f = self._env.getFrameManager().createFrame(req.frame_name, pos=[req.frame_pose.position.x,req.frame_pose.position.y,req.frame_pose.position.z], orn=[req.frame_pose.orientation.x,req.frame_pose.orientation.y,req.frame_pose.orientation.z,req.frame_pose.orientation.w], ref_id=req.ref_frame_id)
        print("ROSCommManager add_frame (AddFrameResponse) name: " + str(req.frame_name) + ", ref id: " + str(req.ref_frame_id) + ", f: " + str(f.getFrameId()))

        self.debugcount = self.debugcount + 1
        if self.debugcount == 3:
            self._env.getConstraintManager().addMaxwellConstraint(f.getFrameId(), req.ref_frame_id)
        
        return f.getFrameId()

    def add_body_frame(self, req):
        if self._env == None:
            return -1337
        
        f = self._env.getFrameManager().createFrame(req.frame_name, ref_id=req.ref_frame_id, ref_link_id=req.ref_frame_link_id, is_body_frame=True)
        print("ROSCommManager add_body_frame (AddFrameResponse) name: " + str(req.frame_name) + ", ref id: " + str(req.ref_frame_id) + ", myid: " + str(f.getFrameId()))     
        return f.getFrameId()

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