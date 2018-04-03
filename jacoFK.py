import vrep
import time
import numpy as np
import jacoKinematics as jk
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


# Get user input for desired joint angles    
print( 'Enter choice of 6 joint variables in degrees: ')
thetas = [float(x) for x in input().split()]
for i in range(0,6):
    thetas[i] = deg2rad(thetas[i])

res,goalFrame = vrep.simxGetObjectHandle(clientID, 'goalFrame', vrep.simx_opmode_blocking)
# res,worldFrame = vrep.simxGetObjectHandle(clientID, 'worldFrame', vrep.simx_opmode_blocking)
jointHands = getJoiHands(clientID,'Jaco')
res,jacoFrame = vrep.simxGetObjectHandle(clientID, "Jaco",vrep.simx_opmode_blocking)


goalT = jk.jaco_FK(thetas)
goalR,goalPos = fromPose(goalT)
goalEuler = rot2eul(goalR)

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
# Set goal frame dummy to estimated location
vrep.simxSetObjectOrientation(clientID, goalFrame,jacoFrame, goalEuler, vrep.simx_opmode_blocking)
vrep.simxSetObjectPosition(clientID, goalFrame,jacoFrame, goalPos, vrep.simx_opmode_blocking)


# Move arm to exact location using the angles entered
joi1_o = getJoiPos(clientID,jointHands[0])
joi2_o = getJoiPos(clientID,jointHands[1])
joi3_o = getJoiPos(clientID,jointHands[2])
joi4_o = getJoiPos(clientID,jointHands[3])
joi5_o = getJoiPos(clientID,jointHands[4])
joi6_o = getJoiPos(clientID,jointHands[5])
joiPos_o = np.array([joi1_o,joi2_o,joi3_o,joi4_o,joi5_o,joi6_o])

for i in range(0, 6):
    setJoiTargPos(clientID,jointHands[i],thetas[i] + joiPos_o[i])
    print("Joint " + str(i+1) + " Moved by " + str(rad2deg(thetas[i])) + " Degrees")
    time.sleep(0.5)


res,goalEuls = vrep.simxGetObjectOrientation(clientID,goalFrame,jacoFrame,vrep.simx_opmode_blocking)
printEuls(goalEuls,'Goal')
res,newEuls = vrep.simxGetObjectOrientation(clientID,jointHands[5],jacoFrame,vrep.simx_opmode_blocking)
printEuls(newEuls,'new')

res,goalPos = vrep.simxGetObjectPosition(clientID,goalFrame,jacoFrame,vrep.simx_opmode_blocking)
printPos(goalPos,'Goal')
res,newPos = vrep.simxGetObjectPosition(clientID,jointHands[5],jacoFrame,vrep.simx_opmode_blocking)
printPos(newPos,'new')



print('Error in Orientation: '+ str(np.linalg.norm(np.subtract(np.array(newEuls),np.array(goalEuls)))))
print('Error in Position: '+ str(np.linalg.norm(np.subtract(np.array(newPos),np.array(goalPos)))))
time.sleep(5)


# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)