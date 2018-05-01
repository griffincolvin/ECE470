import numpy as np
from numpy.linalg import *
import vrep
from vrepHelpers import *
from mathHelpers import *
from scipy.linalg import expm, sinm, cosm
from jacoData import JacoScrewMatrix, getJacoZeroPose
from ece470_lib import *
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
    for i in reversed(range(0, 6)):
            setJoiTargPos(clientID,jointHands[i],thetas[i] + np.pi)
        # if print:
        #     print("Joint " + str(i+1) + " Moved by " + str(rad2deg(thetas[i])) + " Degrees")
        # if delay > 0:
        #     time.sleep(delay)
    time.sleep(delay)


@static_vars(__old_thetas=np.zeros(6,1))
def jaco_move_pose(clientID, Pose, delay=0.0,p=False):
    thetas = jaco_IK(Pose, theta_init=jaco_move_pose.__old_thetas)
    if thetas is None:
        return False
    delay2 = __old_thetas - thetas
    delay2 = np.norm(delay) * 0.02
    jaco_move_pose.__old_thetas = thetas
    jaco_move_theta(clientID, thetas, delay=delay2)
    return True

@static_vars(__old_pose = np.identity(4))
def jaco_move_pose_interplote(clientID, new_pose, spacing=0.1, delay=0):
    dist = np.linalg.norm(fromPose(__old_pose)[1] - fromPose(new_pose))
    for int_pose in matrix_linspace(__old_pose, new_pose,1+int(dist/spacing) ,to_end=True):
        if not jaco_move_pose(clientID, int_pose, delay=delay):
            __old_pose = int_pose
            return False
    __old_pose = new_pose
    return True
