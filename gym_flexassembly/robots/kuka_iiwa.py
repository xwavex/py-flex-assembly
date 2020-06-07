import os, inspect
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# parentdir = os.path.dirname(os.path.dirname(currentdir))
# os.sys.path.insert(0, parentdir)

import pybullet as p
import numpy as np
import copy
import math

# KDL SOLVER IMPORTS
# import PyKDL

# FLEX ASSEMBLY DATA IMPORTS
from gym_flexassembly import data as flexassembly_data

class KukaIIWA:
    def __init__(self, pos=[0,0,0.07], orn=[0,0,0,1], urdfRootPath=flexassembly_data.getDataPath(), timeStep=0.001, variant='7'):
        self.variant = variant
        self._pos = pos
        self._orn = orn
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep
        self.maxVelocity = 0.35
        self.maxForce = 200.0
        self.kukaEndEffectorIndex = 6
        self.kukaGripperIndex = 7
        #lower limits for null space
        self.ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05] # TODO find right values!
        #upper limits for null space
        self.ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05] # TODO find right values!


        # Init
        self.cur_q = [0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539]
        self.cur_qdot = [0,0,0,0,0,0,0]
        self.cur_rF = [0,0,0,0,0,0,0]
        self.cur_eF = [0,0,0,0,0,0,0]

        self.reset()

        # TODO 1) add collision stuff
        # TODO 2) fix getInertiaMatrix, getGravityVector
        # TODO 3) add jacobian
        # TODO 4) add jacobian dot

    def reset(self):
        if self.variant == '14':
            self.kukaUid = p.loadURDF(os.path.join(self.urdfRootPath, "kuka-iiwa-7/model.urdf"), useFixedBase=True)
            # flags=p.URDF_USE_INERTIA_FROM_FILE
        else:
            self.kukaUid = p.loadURDF(os.path.join(self.urdfRootPath, "kuka-iiwa-7/model.urdf"), useFixedBase=True)

        p.resetBasePositionAndOrientation(self.kukaUid, self._pos, self._orn)

        self.numJoints = p.getNumJoints(self.kukaUid)
        # for jointIndex in range(self.numJoints):
        # Reset joints to the initial configuration
        # p.resetJointState(self.kukaUid, jointIndex, self.cur_q[jointIndex])
        # p.setJointMotorControl2(self.kukaUid,
        #                         jointIndex,
        #                         p.POSITION_CONTROL,
        #                         targetPosition=self.cur_q[jointIndex],
        #                         force=self.maxForce)

        self.motorNames = []
        self.motorIndices = []
        self.zeroForces = []
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(self.kukaUid, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                print("motorname " + str(jointInfo[1]) + ", index " + str(i))
                self.motorNames.append(str(jointInfo[1]))
                self.motorIndices.append(i)
                self.zeroForces.append(0.0)
            else:
                print("ignored joint " + str(jointInfo[1]) + ", index " + str(i))

        for i in range(len(self.motorIndices)):
            p.resetJointState(self.kukaUid, self.motorIndices[i], self.cur_q[i])

        self.controlMode = "JOINT_IMPEDANCE_CONTROL"
        self.setControlMode(self.controlMode)

    def getDoFSize(self):
        return len(self.motorIndices)

    def getMotorIndices(self):
        return self.motorIndices

    def getObservationDimension(self):
        return len(self.getObservation())

    def getUUid(self):
        return self.kukaUid

    def getObservation(self):
        observation = []
        # state = p.getLinkState(self.kukaUid, self.kukaGripperIndex)
        # p.getJointStates()
        # pos = state[0]
        # orn = state[1]
        # euler = p.getEulerFromQuaternion(orn)

        # observation.extend(list(pos)) # extend just adds into the same list
        # observation.extend(list(euler))

        # JOINT STATES
        joint_states = p.getJointStates(self.kukaUid, self.motorIndices)
        for j in range(len(joint_states)):
            js = joint_states[j]
            self.cur_q[j] = js[0]
            self.cur_qdot[j] = js[1]
            self.cur_rF[j] = js[2]
            self.cur_eF[j] = js[3]

        observation.append(list(self.cur_q))
        observation.append(list(self.cur_qdot))

        return observation

    def getInertiaMatrix(self):
        # Controller with MassMatrix
        # https://github.com/bulletphysics/bullet3/blob/aec9968e281faca7bc56bc05ccaf0ef29d82d062/examples/pybullet/examples/pdControllerStable.py
        # https://github.com/bulletphysics/bullet3/blob/0aaae872451a69d0c93b0c8ed818667de4ad5653/examples/pybullet/gym/pybullet_utils/pd_controller_stable.py
        
        # of EEF
        dyn = p.getDynamicsInfo(self.kukaUid, -1) # TODO -1 seems to be wrong! Because it refersa to the base!
        print("dyn = " + str(dyn))
        mass = dyn[0]
        friction = dyn[1]
        localInertiaDiagonal = dyn[2]
        pass

    def getGravityVector(self):
        pass

    def setControlMode(self, controlMode):
        # TODO use ENUM not String here
        if controlMode == "JOINT_IMPEDANCE_CONTROL":
            # TODO set kp and kd
            p.setJointMotorControlArray(self.kukaUid,
                                        self.motorIndices,
                                        p.POSITION_CONTROL,
                                        targetPositions=self.cur_q)
            # TODO PERHAPS ADD MAX FORCES!
            self.controlMode = controlMode
        if controlMode == "JOINT_TORQUE_CONTROL":
            # # Disable the motors first
            # Disable the motors for torque control:
            p.setJointMotorControlArray(self.kukaUid,
                                        self.motorIndices,
                                        p.VELOCITY_CONTROL,
                                        forces=self.zeroForces)
            self.controlMode = controlMode

    def setCommand(self, motorCommands):
        if self.controlMode == "JOINT_TORQUE_CONTROL":
            # # Use Torque control in the loop
            # # Set the Joint Torques:
            p.setJointMotorControlArray(self.kukaUid,
                                        self.motorIndices,
                                        p.TORQUE_CONTROL,
                                        forces=motorCommands)
            # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_dynamics.py


    # def applyAction(self, motorCommands):
    #     if (self.useInverseKinematics):
    #         dx = motorCommands[0]
    #         dy = motorCommands[1]
    #         dz = motorCommands[2]
    #         da = motorCommands[3]
    #         fingerAngle = motorCommands[4]

    #         state = p.getLinkState(self.kukaUid, self.kukaEndEffectorIndex)
    #         actualEndEffectorPos = state[0]
    #         #print("pos[2] (getLinkState(kukaEndEffectorIndex)")
    #         #print(actualEndEffectorPos[2])

    #         self.endEffectorPos[0] = self.endEffectorPos[0] + dx
    #         if (self.endEffectorPos[0] > 0.65):
    #             self.endEffectorPos[0] = 0.65
    #         if (self.endEffectorPos[0] < 0.50):
    #             self.endEffectorPos[0] = 0.50
    #         self.endEffectorPos[1] = self.endEffectorPos[1] + dy
    #         if (self.endEffectorPos[1] < -0.17):
    #             self.endEffectorPos[1] = -0.17
    #         if (self.endEffectorPos[1] > 0.22):
    #             self.endEffectorPos[1] = 0.22
    #         #print ("self.endEffectorPos[2]")
    #         #print (self.endEffectorPos[2])
    #         #print("actualEndEffectorPos[2]")
    #         #print(actualEndEffectorPos[2])
    #         #if (dz<0 or actualEndEffectorPos[2]<0.5):
    #         self.endEffectorPos[2] = self.endEffectorPos[2] + dz
    #         self.endEffectorAngle = self.endEffectorAngle + da
    #         pos = self.endEffectorPos
    #         orn = p.getQuaternionFromEuler([0, -math.pi, 0])  # -math.pi,yaw])
    #         if (self.useNullSpace == 1):
    #             if (self.useOrientation == 1):
    #                 jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos,
    #                                                 orn, self.ll, self.ul, self.jr, self.rp)
    #             else:
    #                 jointPoses = p.calculateInverseKinematics(self.kukaUid,
    #                                                         self.kukaEndEffectorIndex,
    #                                                         pos,
    #                                                         lowerLimits=self.ll,
    #                                                         upperLimits=self.ul,
    #                                                         jointRanges=self.jr,
    #                                                         restPoses=self.rp)
    #         else:
    #             if (self.useOrientation == 1):
    #                 jointPoses = p.calculateInverseKinematics(self.kukaUid,
    #                                                         self.kukaEndEffectorIndex,
    #                                                         pos,
    #                                                         orn,
    #                                                         jointDamping=self.jd)
    #             else:
    #                 jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos)
    #         #print("jointPoses")
    #         #print(jointPoses)
    #         #print("self.kukaEndEffectorIndex")
    #         #print(self.kukaEndEffectorIndex)
    #         if (self.useSimulation):
    #             for i in range(self.kukaEndEffectorIndex + 1):
    #                 #print(i)
    #                 p.setJointMotorControl2(bodyUniqueId=self.kukaUid,
    #                                         jointIndex=i,
    #                                         controlMode=p.POSITION_CONTROL,
    #                                         targetPosition=jointPoses[i],
    #                                         targetVelocity=0,
    #                                         force=self.maxForce,
    #                                         maxVelocity=self.maxVelocity,
    #                                         positionGain=0.3,
    #                                         velocityGain=1)
    #         else:
    #             #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
    #             for i in range(self.numJoints):
    #                 p.resetJointState(self.kukaUid, i, jointPoses[i])
    #         #fingers
    #         p.setJointMotorControl2(self.kukaUid,
    #                                 7,
    #                                 p.POSITION_CONTROL,
    #                                 targetPosition=self.endEffectorAngle,
    #                                 force=self.maxForce)
    #         p.setJointMotorControl2(self.kukaUid,
    #                                 8,
    #                                 p.POSITION_CONTROL,
    #                                 targetPosition=-fingerAngle,
    #                                 force=self.fingerAForce)
    #         p.setJointMotorControl2(self.kukaUid,
    #                                 11,
    #                                 p.POSITION_CONTROL,
    #                                 targetPosition=fingerAngle,
    #                                 force=self.fingerBForce)

    #         p.setJointMotorControl2(self.kukaUid,
    #                                 10,
    #                                 p.POSITION_CONTROL,
    #                                 targetPosition=0,
    #                                 force=self.fingerTipForce)
    #         p.setJointMotorControl2(self.kukaUid,
    #                                 13,
    #                                 p.POSITION_CONTROL,
    #                                 targetPosition=0,
    #                                 force=self.fingerTipForce)
    #     else:
    #         for action in range(len(motorCommands)):
    #             motor = self.motorIndices[action]
    #             p.setJointMotorControl2(self.kukaUid,
    #                                     motor,
    #                                     p.POSITION_CONTROL,
    #                                     targetPosition=motorCommands[action],
    #                                     force=self.maxForce)

    # def getPartialDerivativeHybrid(self, bs_J_ee, joint_idx, column_idx):
    #     ''' Inspired by http://docs.ros.org/kinetic/api/orocos_kdl/html/chainjnttojacdotsolver_8cpp_source.html
    #     const Twist& ChainJntToJacDotSolver::getPartialDerivativeHybrid(const KDL::Jacobian& bs_J_ee,
    #                                                               const unsigned int& joint_idx,
    #                                                               const unsigned int& column_idx)
    #     '''
    #     j = joint_idx
    #     i = column_idx

    #     jac_j_ = bs_J_ee[:,j]
    #     jac_i_ = bs_J_ee[:,i]

    #     # SetToZero(t_djdq_);
    #     # TWIST vel and rot
    #     t_djdq_ = []

    #     if j < i:
    #         # P_{\Delta}({}_{bs}J^{j})  ref (20)
    #         t_djdq_.append(list(jac_j_.rot * jac_i_.vel)) # TODO
    #         t_djdq_.rot = jac_j_.rot * jac_i_.rot
    #     elif j > i:
    #         # M_{\Delta}({}_{bs}J^{j})  ref (23)
    #         SetToZero(t_djdq_.rot)
    #         t_djdq_.vel = -jac_j_.vel * jac_i_.rot
    #     elif j == i:
    #         #ref (40)
    #         SetToZero(t_djdq_.rot)
    #         t_djdq_.vel = jac_i_.rot * jac_i_.vel
        
    #     return t_djdq_

    # def getJacDot(self, qdot, jac, motor_indicies):
    #     segmentNr = len(motor_indicies)

    #     # Initialize Jacobian to zero since only segmentNr columns are computed
    #     Jdot = np.zeros([6, segmentNr])

    #     # First compute the jacobian in the Hybrid representation
    #     jac_ = jac

    #     # Let's compute Jdot in the corresponding representation
    #     k = 0
    #     jac_dot_k_ = np.zeros([6]) # TODO???

    #     for i in range(segmentNr):
    #         # Only increase joint nr if the segment has a joint, assume that fixed are already removed from the motor list
    #         for j in range(segmentNr):
    #             # Column J is the sum of all partial derivatives ref (41)
    #             jac_dot_k_ += getPartialDerivative(jac_, j, k) * qdot[j];
                
    #         k = k + 1
    #         jdot.setColumn(k, jac_dot_k_);
    #         jac_dot_k_ = np.zeros([6]) # TODO???

    #     return Jdot

class KukaIIWA7(KukaIIWA):
    def __init__(self, urdfRootPath=flexassembly_data.getDataPath(), timeStep=0.001):
        KukaIIWA.__init__(self, urdfRootPath=urdfRootPath, timeStep=timeStep, variant='7')

class KukaIIWA14(KukaIIWA):
    def __init__(self, urdfRootPath=flexassembly_data.getDataPath(), timeStep=0.001):
        KukaIIWA.__init__(self, urdfRootPath=urdfRootPath, timeStep=timeStep, variant='14')

__all__ = ['KukaIIWA7', 'KukaIIWA14']