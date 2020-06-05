import pybullet as p
import time

p.connect(p.GUI)

jointTypeNames = [
    "JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"
]

sphereRadius = 0.05
# colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
colBoxId = p.createCollisionShape(p.GEOM_BOX, collisionFramePosition=[0,0,0],
                                  halfExtents=[sphereRadius, sphereRadius, sphereRadius])
print("colBoxId = " + str(colBoxId))

mass = 0
visualShapeId = -1

link_Masses = [1]
linkCollisionShapeIndices = [colBoxId]
linkVisualShapeIndices = [-1]
linkPositions = [[0, 0, 0]]
linkOrientations = [[0, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
indices = [colBoxId]
jointTypes = [p.JOINT_SPHERICAL]
axis = [[0, 0, 1]]

basePosition = [0,0,2]
baseOrientation = [0,0,0,1]

ob = p.createMultiBody(mass,
        colBoxId,
        visualShapeId,
        basePosition,
        baseOrientation,
        linkMasses=link_Masses,
        linkCollisionShapeIndices=linkCollisionShapeIndices,
        linkVisualShapeIndices=linkVisualShapeIndices,
        linkPositions=linkPositions,
        linkOrientations=linkOrientations,
        linkInertialFramePositions=linkInertialFramePositions,
        linkInertialFrameOrientations=linkInertialFrameOrientations,
        linkParentIndices=indices,
        linkJointTypes=jointTypes,
        linkJointAxis=axis)

                                         
p.setJointMotorControlMultiDof(ob,
                                         0,
                                         controlMode=p.POSITION_CONTROL,
                                         targetPosition=[0,0,0,1], force=[10,10,10])
                                         

print("ob=",ob)



if 0:
  spherical = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/pyCompliantInteractionPlanning/gym_flexassembly/motion_constraint_tests/spherical.urdf", useFixedBase=True)
  print("spherical=",spherical)
  
  for j in range(p.getNumJoints(spherical)):
    print("joint j=",j)
    ji = p.getJointInfo(spherical, j)
    print("ji[2]=",ji[2])
    print("joint[", j, "].type=", jointTypeNames[ji[2]])
    
    p.setJointMotorControlMultiDof(spherical,
                                         j,
                                         controlMode=p.POSITION_CONTROL,
                                         targetPosition=[0,0,0,1], force=[100,100,100])

while (1):
  p.stepSimulation()
  time.sleep(1./240.)
  