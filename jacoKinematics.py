import numpy as np
from numpy.linalg import *
# from vrepHelpers import *
from mathHelpers import *
from scipy.linalg import expm, sinm, cosm
from jacoData import JacoScrewMatrix, getJacoZeroPose

__jaco_screw = JacoScrewMatrix()
__jaco_zero_M = getJacoZeroPose()


def jaco_FK(thetas):
	return evalT(__jaco_screw, thetas, __jaco_zero_M)

def jaco_IK(endPose, theta_init = None):
	theta, err = findIK(endPose, __jaco_screw, __jaco_zero_M, theta=None)
	if err > 0.01:
		theta, err = findIK(endPose, __jaco_screw, __jaco_zero_M, theta=None, max_iter=500)
		if err > 0.01:
			return None
	return theta