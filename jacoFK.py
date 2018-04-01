import vrep
import time
import numpy as np
from vrepHelpers import *
from mathHelpers import *
from jacoFK_JGN import fKin

# fuck u 
# Close all open connections (just in case)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


# Get user input for desired joint angles    
print( 'Enter choice of 6 joint variables in degrees: ')
thetas = [int(x) for x in input().split()]
for i in range(0,6):
    thetas[i] = deg2rad(thetas[i])

res,goalFrame = vrep.simxGetObjectHandle(clientID, 'goalFrame', vrep.simx_opmode_blocking)
res,worldFrame = vrep.simxGetObjectHandle(clientID, 'worldFrame', vrep.simx_opmode_blocking)
jointHands = getJoiHands(clientID,'Jaco')

res,jacoFrame = vrep.simxGetObjectHandle(clientID, "Jaco",vrep.simx_opmode_blocking)


goaleuler, goalp = fKin(thetas)

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
# Set goal frame dummy to estimated location
vrep.simxSetObjectOrientation(clientID, goalFrame,jacoFrame, goaleuler, vrep.simx_opmode_blocking)
vrep.simxSetObjectPosition(clientID, goalFrame,jacoFrame, goalp, vrep.simx_opmode_blocking)
printEuls(goaleuler, 'Goal')
printPos(goalp,'Goal')

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
    time.sleep(3)

res,newori = vrep.simxGetObjectOrientation(clientID,jointHands[5],jacoFrame,vrep.simx_opmode_blocking)
printEuls(newori,'new')
res,newpos = vrep.simxGetObjectPosition(clientID,jointHands[5],jacoFrame,vrep.simx_opmode_blocking)
printPos(newpos,'new')
time.sleep(5)




# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)