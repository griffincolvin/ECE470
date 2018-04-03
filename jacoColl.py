import vrep
import numpy as np
import jacoKinematics as jk
import time
from jacoData import *
from vrepHelpers import *
from mathHelpers import *


def fromPose(T):
    """
    Returns a rotation matrix and position vector from a 4x4 HCT matrix
    :param T: The 4x4 HCT matrix as either python lists or numpy array
    :returns: a tuple with the first element being a 3x3 numpy array representing the rotation matrix
              and the second element being a 3x1 numpy array position vector
    """
    T = np.asarray(T)
    return (T[:3,:3], T[:3, 3:4])

# Close all open connections (just in case)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

configs = []
for i in range(1,30):
    configs.append(np.random.rand(6,1)*np.pi)

print(configs)
jacoScrew = JacoScrewMatrix()
jacoM = getJacoZeroPose()
jacoRadii = getJacoSpheres()


res,goalFrame = vrep.simxGetObjectHandle(clientID, 'goalFrame', vrep.simx_opmode_blocking)
jointHands = getJoiHands(clientID,'Jaco')
res,jacoFrame = vrep.simxGetObjectHandle(clientID, 'Jaco',vrep.simx_opmode_blocking)
res,sphere = vrep.simxGetObjectHandle(clientID,'Sphere',vrep.simx_opmode_blocking)
res,sphere0 = vrep.simxGetObjectHandle(clientID,'Sphere0',vrep.simx_opmode_blocking)
res,sphere1 = vrep.simxGetObjectHandle(clientID,'Sphere1',vrep.simx_opmode_blocking)
res,sphere2 = vrep.simxGetObjectHandle(clientID,'Sphere2',vrep.simx_opmode_blocking)
res,sphere3 = vrep.simxGetObjectHandle(clientID,'Sphere3',vrep.simx_opmode_blocking)
res,sphere4 = vrep.simxGetObjectHandle(clientID,'Sphere4',vrep.simx_opmode_blocking)
res,sphere5 = vrep.simxGetObjectHandle(clientID,'Sphere5',vrep.simx_opmode_blocking)
res,sphere6 = vrep.simxGetObjectHandle(clientID,'Sphere6',vrep.simx_opmode_blocking)
res,sphere7 = vrep.simxGetObjectHandle(clientID,'Sphere7',vrep.simx_opmode_blocking)
res,sphere8 = vrep.simxGetObjectHandle(clientID,'Sphere8',vrep.simx_opmode_blocking)

spheres_st = [np.atleast_2d(sphere6),np.atleast_2d(sphere7),np.atleast_2d(sphere8)]

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
res,sp_o = vrep.simxGetObjectPosition(clientID,sphere,jacoFrame,vrep.simx_opmode_blocking)
res,sp0_o = vrep.simxGetObjectPosition(clientID,sphere0,jacoFrame,vrep.simx_opmode_blocking)
res,sp1_o = vrep.simxGetObjectPosition(clientID,sphere1,jacoFrame,vrep.simx_opmode_blocking)
res,sp2_o = vrep.simxGetObjectPosition(clientID,sphere2,jacoFrame,vrep.simx_opmode_blocking)
res,sp3_o = vrep.simxGetObjectPosition(clientID,sphere3,jacoFrame,vrep.simx_opmode_blocking)
res,sp4_o = vrep.simxGetObjectPosition(clientID,sphere4,jacoFrame,vrep.simx_opmode_blocking)
res,sp5_o = vrep.simxGetObjectPosition(clientID,sphere5,jacoFrame,vrep.simx_opmode_blocking)

spheres_o = [np.atleast_2d(sp_o).T,np.atleast_2d(sp0_o).T,np.atleast_2d(sp1_o).T,np.atleast_2d(sp2_o).T,np.atleast_2d(sp3_o).T,np.atleast_2d(sp4_o).T,np.atleast_2d(sp5_o).T]
th1_o = getJoiPos(clientID,jointHands[0])
th2_o = getJoiPos(clientID,jointHands[1])
th3_o = getJoiPos(clientID,jointHands[2])
th4_o = getJoiPos(clientID,jointHands[3])
th5_o = getJoiPos(clientID,jointHands[4])
th6_o = getJoiPos(clientID,jointHands[5])

thetas_o = np.array([th1_o,th2_o,th3_o,th4_o,th5_o,th6_o])

for pos in configs:

    coll = checkColl(jacoScrew,jacoM,pos,jacoRadii,spheres_o,spheres_st)
    goalT = jk.jaco_FK(pos)
    goalR,goalPos = fromPose(goalT)
    goalEuler = rot2eul(goalR)
    vrep.simxSetObjectOrientation(clientID, goalFrame,jacoFrame, goalEuler, vrep.simx_opmode_blocking)
    vrep.simxSetObjectPosition(clientID, goalFrame,jacoFrame, goalPos, vrep.simx_opmode_blocking)

    for i in range(0, 6):
        setJoiTargPos(clientID,jointHands[i],pos[i] + thetas_o[i])
        # print("Joint " + str(i+1) + " Moved by " + str(rad2deg(thetas[i])) + " Degrees")
        time.sleep(2)
        

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)





