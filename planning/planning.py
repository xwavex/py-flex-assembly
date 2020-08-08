import pybullet as p
import pybullet as pi
import pybullet_data
import pybullet_planning as planning
import numpy as np
import time
import os
from pybullet_planning.utils import INF
from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion, get_distance_fn, get_extend_fn
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info
from pybullet_planning.interfaces.robots import get_movable_joints, set_joint_positions, get_link_pose, get_custom_limits, get_joint_limits, body_from_name, get_body_name, get_joint_names, get_joint_positions, get_joint_q_index, get_joint
import pybullet_planning.interfaces.robots.joint as jt
from pybullet_planning.interfaces.env_manager import create_box

### Nullspace ###
useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions
### Nullspace ###

HERE = os.path.dirname(__file__)
IIWA_ROBOT_URDF = os.path.join(HERE, 'kuka-iiwa-7-bullet', 'model_schunk.urdf')
#IIWA_ROBOT_URDF = os.path.join(HERE, 'kuka-iiwa-7-egp-40', 'model.urdf')
iiwapos = [0.0, 0.0, 0.0]

#p.connect(p.GUI_SERVER)
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#robot = p.loadURDF("kuka_iiwa/model.urdf", iiwapos[0], iiwapos[1], iiwapos[2])
robot = p.loadURDF(IIWA_ROBOT_URDF, iiwapos[0], iiwapos[1], iiwapos[2])
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

start_conf = [-1.570, 0, 1.570, 0, 0, 0, 0]
end_conf = [0, 0, 3.14, 0, 0, 0, 0]
conf = [1.57, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
dump_world()
#get_joint_q_index(robot, get_joint('iiwa7_link_ee'))

startPosition = [0.25, -0.25, 0.5]
goalPosition = [0.5, 0.6, 0.5]
orn = p.getQuaternionFromEuler([0.,-1.57,0.])

# Nullspace
# https://github.com/erwincoumans/xArm-Python-SDK/blob/master/example/wrapper/xarm7/xarm_sim.py
goalJntPos = p.calculateInverseKinematics(robot, 6, goalPosition, orn, ll, ul, jr, rp, maxNumIterations=5)

strtJntPos = p.calculateInverseKinematics(robot, 6, startPosition)
#goalJntPos = p.calculateInverseKinematics(robot, 6, goalPosition)

#block = create_box(0.059, 20., 0.089)
block_x = 0.2
block_y = 0.2
block_z = 1.5
#set_pose(block, Pose(Point(x=block_x, y=block_y, z=block_z), Euler(yaw=np.pi/2)))
block= create_box(block_x, block_y, block_z)
set_pose(block, Pose(Point(x=block_x, y=block_y, z=block_z), Euler(yaw=np.pi/2)))
#block = set_pose(block, Pose(Point(x=0, y=0, z=block_z), Euler(yaw=np.pi/2)))

set_joint_positions(robot, jt.get_movable_joints(robot), strtJntPos)
#path = planning.plan_lazy_prm(start_conf, end_conf, sample_fn, extend_fn, collision_fn, num_samples=200)
#planning.motion_planners.rrt_connect.birrt(q1, q2, distance_fn, sample_fn, extend_fn, collision_fn, RRT_RESTARTS, RRT_SMOOTHING)
#planning.plan_lazy_prm(start_conf, end_conf, sample_fn, extend_fn, collision_fn, num_samples=200, max_degree=10, weights=None, p_norm=2, max_distance=INF, approximate_eps=0.0, max_cost=INF, max_time=INF, max_paths=INF)
print("Joint Limits: " )
print(get_joint_limits(robot, 1))
 
'''
custom_limits={}
movable_joints = get_movable_joints(robot)
lower_limits, upper_limits = get_custom_limits(robot, movable_joints, custom_limits)

ik_joints = planning.get_movable_joints(robot)
arm_joints = planning.get_joint_names(robot, ik_joints)
print(str(arm_joints))

print(get_body_name(robot))

def get_custom_limits_from_name(robot, joint_limits):
    return {joint_from_name(robot, joint): limits
            for joint, limits in joint_limits.items()}

custom_limits = get_custom_limits_from_name(robot, {'lbr_iiwa_joint_1':(-2.96, 2.96), 'lbr_iiwa_joint_2':(-2.96, 2.96), 'lbr_iiwa_joint_3':(-2.96, 2.96), 'lbr_iiwa_joint_4':(-2.96, 2.96), 'lbr_iiwa_joint_5':(-2.96, 2.96),'lbr_iiwa_joint_6':(-2.96, 2.96), 'lbr_iiwa_joint_7':(-2.96, 2.96)})
print(custom_limits)


print("Start")
start_conf = get_joint_positions(robot, jt.get_joints(robot))
print(start_conf)
#sample_fn = get_sample_fn(get_body_name(robot), jt.get_joints(robot), custom_limits=custom_limits)
sample_fn = get_sample_fn(robot, jt.get_movable_joints(robot), custom_limits=custom_limits)
print("Sample: ", sample_fn)
jnt = 2
print("Type: ", type(jnt))
print("Name: ",jt.get_joint_name(robot, jnt))
print("Joint Info: ", jt.get_joint_info(robot, 2))
print("Is Circula: ", jt.is_circular(robot, 2))
'''
#distance_fn = get_distance_fn(robot, jt.get_joints(robot))
#extend_fn = get_extend_fn(robot, jt.get_joints(robot))
#birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn)
path = planning.plan_joint_motion(robot, jt.get_movable_joints(robot), goalJntPos, obstacles=[block])
print("Path: ", path)

if path is None:
    print('no plan found')
else:
    print('a motion plan is found! Press enter to start simulating!')

time_step = 0.03
while(1):
  if path is not None:
    for conf in path:
        planning.set_joint_positions(robot, jt.get_movable_joints(robot), conf)
        planning.wait_for_duration(time_step)

# adjusting this number will adjust the simulation speed
# time_step = 0.03
# for conf in path:
#    planning.set_joint_positions(robot, jt.get_joints(robot), conf)
#    planning.wait_for_duration(time_step)
# p.disconnect()