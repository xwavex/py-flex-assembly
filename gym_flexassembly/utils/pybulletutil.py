import math

import numpy

import pybullet as p

def make_inertia_pybullet_conform(mass, ixx, ixy, ixz, iyy, iyz, izz, inertialFrame_pos, inertialFrame_quat):
    ''' TODO!
    '''
    linkInertiaBasis = numpy.zeros(shape=(3,3))
    if ixy == 0.0 and ixz == 0.0 and iyz == 0.0:
        principalInertiaX = ixx
        principalInertiaY = iyy
        principalInertiaZ = izz
    else:
        principalInertiaX = ixx
        inertiaTensor = numpy.array([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])
        threshold = 1.0e-6
        numIterations = 30

        linkInertiaBasis = diagonalize(inertiaTensor, threshold, numIterations);
        principalInertiaX = inertiaTensor[0][0];
        principalInertiaY = inertiaTensor[1][1];
        principalInertiaZ = inertiaTensor[2][2];

    if principalInertiaX < 0 or principalInertiaX > (principalInertiaY + principalInertiaZ) or principalInertiaY < 0 or principalInertiaY > (principalInertiaX + principalInertiaZ) or principalInertiaZ < 0 or principalInertiaZ > (principalInertiaX + principalInertiaY):
        print("Bad inertia tensor properties, setting inertia to zero")
        principalInertiaX = 0.0;
        principalInertiaY = 0.0;
        principalInertiaZ = 0.0;
        linkInertiaBasis = numpy.zeros(shape=(3,3))

    localInertiaDiagonal = numpy.array([principalInertiaX, principalInertiaY, principalInertiaZ])
    
    linkLocalFrame_base = p.getMatrixFromQuaternion(inertialFrame_quat) * linkInertiaBasis
    
    return localInertiaDiagonal, inertialFrame_pos, linkLocalFrame_base


def diagonalize(matrix, threshold, maxSteps):
    ''' Taken from pybullet repo
    ''' 
    rot = numpy.zeros(shape=(3,3))
    step = maxSteps
    for t_step in range(maxSteps):
        # find off-diagonal element [p][q] with largest magnitude
        p = 0
        q = 1
        r = 2
        max = math.fabs(matrix[0][1]);
        v = math.fabs(matrix[0][2]);
        if v > max:
            q = 2
            r = 1
            max = v
        v = math.fabs(matrix[1][2]);
        if v > max:
            p = 1
            q = 2
            r = 0
            max = v

        t = threshold * (math.fabs(matrix[0][0]) + math.fabs(matrix[1][1]) + math.fabs(matrix[2][2]))
        if max <= t:
            if max <= 2.2204460492503131e-16 * t:
                return numpy.zeros(shape=(3,3))
            step = 1

        # compute Jacobi rotation J which leads to a zero for element [p][q]
        mpq = matrix[p][q]
        theta = (matrix[q][q] - matrix[p][p]) / (2 * mpq)
        theta2 = theta * theta
        cos = 0
        sin = 0
        if theta2 * theta2 < (10 / 2.2204460492503131e-16):
            if theta >= 0:
                t = 1 / (theta + math.sqrt(1 + theta2))
            else:
                1 / (theta - math.sqrt(1 + theta2))
            cos = 1 / math.sqrt(1 + t * t)
            sin = cos * t
        else:
            # approximation for large theta-value, i.e., a nearly diagonal matrix
            t = 1 / (theta * (2 + 0.5 / theta2))
            cos = 1 - 0.5 * t * t
            sin = cos * t

        # apply rotation to matrix (this = J^T * this * J)
        matrix[p][q] = matrix[q][p] = 0
        matrix[p][p] -= t * mpq
        matrix[q][q] += t * mpq
        mrp = matrix[r][p]
        mrq = matrix[r][q]
        matrix[r][p] = matrix[p][r] = cos * mrp - sin * mrq
        matrix[r][q] = matrix[q][r] = cos * mrq + sin * mrp

        # apply rotation to rot (rot = rot * J)
        for i in range(3):
            mrp = rot[i][p]
            mrq = rot[i][q]
            rot[i][p] = cos * mrp - sin * mrq
            rot[i][q] = cos * mrq + sin * mrp
        
        step = step - 1
    return rot