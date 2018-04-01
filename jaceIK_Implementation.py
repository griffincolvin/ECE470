import vrep
import time
import numpy as np

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



# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(1)

res,theta1 = vrep.simxGetJointPosition(clientID, jointHands1, vrep.simx_opmode_blocking)
res,theta2 = vrep.simxGetJointPosition(clientID, jointHands2, vrep.simx_opmode_blocking)
res,theta3 = vrep.simxGetJointPosition(clientID, jointHands3, vrep.simx_opmode_blocking)
res,theta4 = vrep.simxGetJointPosition(clientID, jointHands4, vrep.simx_opmode_blocking)
res,theta5 = vrep.simxGetJointPosition(clientID, jointHands5, vrep.simx_opmode_blocking)
res,theta6 = vrep.simxGetJointPosition(clientID, jointHands6, vrep.simx_opmode_blocking)

thetas = 
# Set the desired value of the first joint variable
if False: '''
vrep.simxSetJointTargetPosition(clientID, jointHands1, theta1 +(np.pi), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, jointHands2, theta2 +(np.pi/4), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, jointHands3, theta3 +(np.pi/4), vrep.simx_opmode_oneshot)
time.sleep(8)
vrep.simxSetJointTargetPosition(clientID, jointHands3, theta3 +(np.pi), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, jointHands6, theta6 +(np.pi), vrep.simx_opmode_oneshot)
time.sleep(8)
vrep.simxSetJointTargetPosition(clientID, jointHands3, theta3 -(np.pi), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, jointHands6, theta6 +(np.pi), vrep.simx_opmode_oneshot)
time.sleep(8)
vrep.simxSetJointTargetPosition(clientID, jointHands3, theta3 +(np.pi), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, jointHands6, theta6 +(np.pi), vrep.simx_opmode_oneshot)
time.sleep(8)
vrep.simxSetJointTargetPosition(clientID, jointHands3, theta3 -(np.pi/2), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, jointHands6, theta6 +(np.pi), vrep.simx_opmode_oneshot)
time.sleep(8)
vrep.simxSetJointTargetPosition(clientID, jointHands4, theta4 +(np.pi/2), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, jointHands5, theta5 -(np.pi/2), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, jointHands6, theta6 +(2*np.pi), vrep.simx_opmode_oneshot)
time.sleep(6)
'''
for i in thetas
    vrep.simxSetJointTargetPosition(clientID, jointHands1, theta1 + thetas[i][0], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, jointHands2, theta2 + thetas[i][1], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, jointHands3, theta3 + thetas[i][2], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, jointHands4, theta4 + thetas[i][3], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, jointHands5, theta5 + (np.pi) + thetas[i][4], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, jointHands6, theta6 + thetas[i][5], vrep.simx_opmode_oneshot)

time.sleep(30)
# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
