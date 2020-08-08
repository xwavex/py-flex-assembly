#!/usr/bin/python

"""This test plays with multibody creation and different constraints.
:Author:
  `Dennis Leroy Wigand <dwigand@cor-lab.de>`
"""

import pybullet as p
import time
import numpy as np
import math

import os

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data
print("flexassembly_data.getDataPath() = " + str(flexassembly_data.getDataPath()))

# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI)
p.setGravity(0, 0, 0)
p.setRealTimeSimulation(0)
p.setTimeStep(0.001)

p.setAdditionalSearchPath(flexassembly_data.getDataPath())

# Floor SHOULD BE ALWAYS ID 0
p.loadURDF("objects/plane_solid.urdf", useMaximalCoordinates=True) # Brauche ich fuer die hit rays

#################
### MULTIBODY ###
#################

#the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="robots/schunk-egp-40/meshes/visual/obj/schunk_low_baked_small.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=[0,0,0],
                                    meshScale=[1,1,1])
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="robots/schunk-egp-40/meshes/visual/obj/schunk_low_baked_small.obj",
                                          collisionFramePosition=[0,0,0],
                                          meshScale=[1,1,1])


schunk = p.createMultiBody(baseMass=0.6,
                        baseInertialFramePosition=[0.00078059, -0.00070996, 0.04726637],
                        baseCollisionShapeIndex=collisionShapeId,
                        baseVisualShapeIndex=visualShapeId,
                        basePosition=[0,0,0.5],
                        useMaximalCoordinates=False
                        # # links
                        # linkMasses=[0.6],
                        # linkCollisionShapeIndices=[collisionShapeId],
                        # linkVisualShapeIndices=[visualShapeId],
                        # linkPositions=[[0,0,0.4]],
                        # linkOrientations=[[0,0,0,1]],
                        # linkInertialFramePositions=[[0.00078059, -0.00070996, 0.04726637]],
                        # linkInertialFrameOrientations=[[0,0,0,1]],
                        # linkParentIndices=[0],
                        # linkJointTypes=[p.JOINT_REVOLUTE],
                        # linkJointAxis=[[1,0,0]]
)
p.changeDynamics(schunk, -1, mass=0.6)

# c_x = p.createConstraint(schunk,
#                     -1,
#                     -1,
#                     -1,
#                     jointType=p.JOINT_PRISMATIC,
#                     jointAxis=[1, 0, 0],
#                     parentFramePosition=[0, 0, 0],
#                     childFramePosition=[0, 0, 0])
# p.changeConstraint(c_x, maxForce=50)

c_y = p.createConstraint(schunk,
                    -1,
                    -1,
                    -1,
                    jointType=p.JOINT_PRISMATIC,
                    jointAxis=[0, 1, 0],
                    parentFramePosition=[0, 0, 0],
                    childFramePosition=[0, 0, 0])
p.changeConstraint(c_y, maxForce=0)

c_z = p.createConstraint(schunk,
                    -1,
                    -1,
                    -1,
                    jointType=p.JOINT_POINT2POINT,
                    jointAxis=[0, 0, 1],
                    parentFramePosition=[0, 0, 0],
                    childFramePosition=[0, 0, 0])
p.changeConstraint(c_z, maxForce=50)

# p.changeDynamics(schunk, -1, localInertiaDiagnoal=[])

# <mass value="0.6" />
#             <inertia ixx="0.0011357" ixy="-0.00000024461" ixz="0.000014912" iyy="0.0012832" iyz="0.0000085651" izz="0.00066545" />
# <mass value="0.04" />
#             <origin rpy="0 0 0" xyz="0 0 0" />
#             <inertia ixx="0.0000013454" ixy="0" ixz="0" iyy="0.00000061207" iyz="0" izz="0.00000097333" />
#  <mass value="0.04" />
#             <origin rpy="0 0 0" xyz="0 0 0" />
#             <inertia ixx="0.0000013454" ixy="0" ixz="0" iyy="0.00000061207" iyz="0" izz="0.00000097333" />

p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], parentObjectUniqueId=schunk, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], parentObjectUniqueId=schunk, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], parentObjectUniqueId=schunk, parentLinkIndex=-1)


# # Create a constraint to keep the fingers centered
# c = p.createConstraint(schunk,
#                     0,
#                     schunk,
#                     1,
#                     jointType=p.JOINT_GEAR,
#                     jointAxis=[1, 0, 0],
#                     parentFramePosition=[0, 0, 0],
#                     childFramePosition=[0, 0, 0])
# p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

dp_joint_pos_0 = p.addUserDebugParameter("force Z", -10, 10, 9.81*0.6+2*0.04)
kpp = p.addUserDebugParameter("kp", 0, 500, 10)
kdd = p.addUserDebugParameter("kd", 0, 30, 0.1)

while 1:
    joint_pos_0 = p.readUserDebugParameter(dp_joint_pos_0)
    kp = p.readUserDebugParameter(kpp)
    kd = p.readUserDebugParameter(kdd)

    pos, orn = p.getBasePositionAndOrientation(schunk)
    # _base_mass_urdf = p.getDynamicsInfo(schunk, -1)
    # p.changeDynamics(schunk, -1, mass=[0.6])
    # print(p.getDynamicsInfo(schunk, -1))
    # print(_base_mass_urdf)
    # print(p.getDynamicsInfo(schunk, 0))
    # print(p.getDynamicsInfo(schunk, 1))
    vpos, vorn = p.getBaseVelocity(schunk)

    pError = kp*([0,0,0.5]-np.array(pos)) - kd*np.array(vpos)
    # pError[2] = 2*(0.5-np.array(pos)[2]) - 0.1*np.array(vpos)[2]
    pError = np.array(pError)
    pError = pError + np.array([0,0,joint_pos_0])

    # p.applyExternalForce(schunk, -1, np.array(pError), pos, flags=p.WORLD_FRAME)

    p.changeConstraint(c_z, jointChildPivot=[0,0,0.5], maxForce=1000)

    p.stepSimulation()

    time.sleep(1/1000) # TODO DLW
