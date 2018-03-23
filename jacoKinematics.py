import numpy as np
from numpy.linalg import *
from vrepHelpers import *
from mathHelpers import *
from scipy.linalg import expm, sinm, cosm
import jacoData

__jaco_screw = jacoData.jacoScrewMatrix()
__jaco_zero_M = jacoData.getJacoZeroPose():


def jaco_FK(thetas):
	return evalT(__jaco_screw, thetas, __jaco_zero_M)

def jaco_IK(endPose, theta_init = None):
	return findIK(endPose, __jaco_screw, __jaco_zero_M_, theta_init=None)