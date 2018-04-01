import vrep
import time
import numpy as np
import jacoIK
from mathHelpers import *
from vrepHelpers import *

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


res,jointHands1 = vrep.simxGetObjectHandle(clientID,'Jaco_joint1',vrep.simx_opmode_blocking)
res,jointHands2 = vrep.simxGetObjectHandle(clientID,'Jaco_joint2',vrep.simx_opmode_blocking)
res,jointHands3 = vrep.simxGetObjectHandle(clientID,'Jaco_joint3',vrep.simx_opmode_blocking)
res,jointHands4 = vrep.simxGetObjectHandle(clientID,'Jaco_joint4',vrep.simx_opmode_blocking)
res,jointHands5 = vrep.simxGetObjectHandle(clientID,'Jaco_joint5',vrep.simx_opmode_blocking)
res,jointHands6 = vrep.simxGetObjectHandle(clientID,'Jaco_joint6',vrep.simx_opmode_blocking)
res,goalFrame = vrep.simxGetObjectHandle(clientID, 'goalFrame', vrep.simx_opmode_blocking)
res,jacoFrame = vrep.simxGetObjectHandle(clientID, "Jaco",vrep.simx_opmode_blocking)
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(1)
jointHands = getJoiHands(clientID,'Jaco')
thetas = jacoIK.joint_vars
poses = jacoIK.poses

# Set the desired value of the first joint variable

for i in range(len(thetas[0])):
    T = poses[i]
    goalr = np.array([[T[0,0], T[0,1], T[0,2]], [T[1,0], T[1,1], T[1,2]], [T[2,0], T[2,1], T[2,2]]])
    goalr = rot2eul(goalr)
    goalp = np.array( [ T[0,3],T[1,3],T[2,3] ] )
    # time.sleep(0.1)

    vrep.simxSetObjectOrientation(clientID, goalFrame,jacoFrame, goalr, vrep.simx_opmode_blocking)
    vrep.simxSetObjectPosition(clientID, goalFrame,jacoFrame, goalp, vrep.simx_opmode_blocking)

    joi1_o = getJoiPos(clientID,jointHands[0])
    joi2_o = getJoiPos(clientID,jointHands[1])
    joi3_o = getJoiPos(clientID,jointHands[2])
    joi4_o = getJoiPos(clientID,jointHands[3])
    joi5_o = getJoiPos(clientID,jointHands[4])
    joi6_o = getJoiPos(clientID,jointHands[5])

    print("joint 1: " + str(joi1_o))
    joiPos_o = np.array([joi1_o,joi2_o,joi3_o,joi4_o,joi5_o,joi6_o])

    for k in range(0, 6):
        if k == 0:
            setJoiTargPos(clientID,jointHands[k],thetas[k][i] )
            print("joint 1: " + str(joi1_o))

        setJoiTargPos(clientID,jointHands[k],thetas[k][i])

        # print("Joint " + str(k+1) + " Moved by " + str((thetas[k][i])) + " Degrees")
        # time.sleep(0.1)
   
time.sleep(20)
# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
origin = np.atleast_2d([-0.5, 0.5, 0.1]).transpose()
xend =   np.atleast_2d([0.5, 0.5, 0.1]).transpose()
yend =   np.atleast_2d([-0.5, 0.5, 0.6]).transpose()