import numpy as np
from numpy.linalg import *
import vrep
from vrepHelpers import *
from mathHelpers import *
from scipy.linalg import expm, sinm, cosm
from jacoData import JacoScrewMatrix, getJacoZeroPose
import time

def static_vars(**kwargs):
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate


__jaco_screw = JacoScrewMatrix()
__jaco_zero_M = getJacoZeroPose()

def jaco_FK(thetas):
    thetas = np.reshape(thetas, (6,1)) 
    return evalT(__jaco_screw, thetas, __jaco_zero_M)

def jaco_IK(endPose, theta_init = None):
    endPose = np.reshape(endPose, (4,4)) 
    theta, err = findIK(endPose, __jaco_screw, __jaco_zero_M, theta=theta_init, max_err = 0.01)
    if err > 0.01:
        return None
    return theta

def jaco_move_theta(clientID, thetas, delay=0.1,p=False):
    thetas = np.reshape(thetas, (6,))
    jointHands = getJoiHands(clientID,'Jaco')
    for i in range(0, 6):
            setJoiTargPos(clientID,jointHands[i],thetas[i] + np.pi)
        # if print:
        #     print("Joint " + str(i+1) + " Moved by " + str(rad2deg(thetas[i])) + " Degrees")
        # if delay > 0:
        #     time.sleep(delay)
    time.sleep(0.05*np.linalg.norm(thetas))


@static_vars(__old_thetas=None)
def jaco_move_pose(clientID, Pose, delay=0.1,p=False):
    thetas = jaco_IK(Pose, theta_init=jaco_move_pose.__old_thetas)
    if thetas is None:
        return False
    jaco_move_pose.__old_thetas = thetas
    jaco_move_theta(clientID, thetas, delay=delay, p=p)
    return True