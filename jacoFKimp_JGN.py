import vrep
import time
import numpy as np
from templateFunctions import *
from jacoFK_JGN import fKin

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# Get user input for desired joint angles    
print( 'Enter choice of 6 joint variables in degrees: ')
thetas = [int(x) for x in input().split()]

res,jacoFrame = vrep.simxGetObjectHandle(clientID, "Jaco",vrep.simx_opmode_blocking)
res,jh1 = vrep.simxGetObjectHandle(clientID,'Jaco_joint1',vrep.simx_opmode_blocking)
res,jh2 = vrep.simxGetObjectHandle(clientID,'Jaco_joint2',vrep.simx_opmode_blocking)
res,jh3 = vrep.simxGetObjectHandle(clientID,'Jaco_joint3',vrep.simx_opmode_blocking)
res,jh4 = vrep.simxGetObjectHandle(clientID,'Jaco_joint4',vrep.simx_opmode_blocking)
res,jh5 = vrep.simxGetObjectHandle(clientID,'Jaco_joint5',vrep.simx_opmode_blocking)
res,jh6 = vrep.simxGetObjectHandle(clientID,'Jaco_joint6',vrep.simx_opmode_blocking)
res,goalFrame = vrep.simxGetObjectHandle(clientID, 'goalFrame', vrep.simx_opmode_blocking)
res,worldFrame = vrep.simxGetObjectHandle(clientID, 'worldFrame', vrep.simx_opmode_blocking)
jointHands = np.array([jh1,jh2,jh3,jh4,jh5,jh6])

res,jaco_pos = vrep.simxGetObjectPosition(clientID, jacoFrame,-1, vrep.simx_opmode_blocking)
res,jaco_eul = vrep.simxGetObjectOrientation(clientID, jacoFrame,-1, vrep.simx_opmode_blocking)
jaco_rot = eul2rot(np.array([jaco_eul[0], jaco_eul[1], jaco_eul[2]]))


goaleuler, goalp = fKin(thetas)

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

vrep.simxSetObjectOrientation(clientID, goalFrame,worldFrame, goaleuler, vrep.simx_opmode_blocking)
vrep.simxSetObjectPosition(clientID, goalFrame,worldFrame,goalp, vrep.simx_opmode_blocking)

# Test the estimated frame position with hard coded angles
time.sleep(2)
res,joint0 = vrep.simxGetJointPosition(clientID, jointHands[0],vrep.simx_opmode_blocking)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, jointHands[0], joint0 + deg2rad(thetas[0]),vrep.simx_opmode_oneshot)

time.sleep(2)
res,joint1 = vrep.simxGetJointPosition(clientID, jointHands[1],vrep.simx_opmode_blocking)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, jointHands[1], joint1 + deg2rad(thetas[1]),vrep.simx_opmode_oneshot)

time.sleep(2)
res,joint2 = vrep.simxGetJointPosition(clientID, jointHands[2],vrep.simx_opmode_blocking)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, jointHands[2], joint2 + deg2rad(thetas[2]),vrep.simx_opmode_oneshot)

time.sleep(2)
res,joint3 = vrep.simxGetJointPosition(clientID, jointHands[3],vrep.simx_opmode_blocking)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, jointHands[3], joint3 + deg2rad(thetas[3]),vrep.simx_opmode_oneshot)

time.sleep(2)
res,joint4 = vrep.simxGetJointPosition(clientID, jointHands[4],vrep.simx_opmode_blocking)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, jointHands[4], joint4 + deg2rad(thetas[4]),vrep.simx_opmode_oneshot)

time.sleep(2)
res,joint5 = vrep.simxGetJointPosition(clientID, jointHands[5],vrep.simx_opmode_blocking)
time.sleep(1)
vrep.simxSetJointTargetPosition(clientID, jointHands[5], joint5 + deg2rad(thetas[5]),vrep.simx_opmode_oneshot)
    

res,joi6realorient = vrep.simxGetObjectOrientation(clientID, jointHands[5], -1, vrep.simx_opmode_blocking)
res,joi6realpos = vrep.simxGetObjectPosition(clientID, jointHands[5], -1, vrep.simx_opmode_blocking)
time.sleep(2)
print("Predicted Position")
print("Expected X value: " + str(goalp[0]))
print("Expected Y value: " + str(goalp[1]))
print("Expected Z value: " + str(goalp[2]))
print("")
print("Actual Position")
print("Actual X value: " + str(joi6realpos[0]))
print("Actual Y value: " + str(joi6realpos[1]))
print("Actual Z value: " + str(joi6realpos[2]))
print("")
print("Predicted Orientation")
print("Expected alpha value: " + str(rad2deg(goaleuler[0])))
print("Expected beta value: " + str(rad2deg(goaleuler[1])))
print("Expected gamma value: " + str(rad2deg(goaleuler[2])))
print("")
print("Actual Orientation")
print("Actual alpha value: " + str(rad2deg(joi6realorient[0])))
print("Actual beta value: " + str(rad2deg(joi6realorient[1])))
print("Actual gamma value: " + str(rad2deg(joi6realorient[2])))


# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)