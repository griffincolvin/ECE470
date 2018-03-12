#import vrep
import time
import numpy as np
from numpy.linalg import *
from templateFunctions import *
from scipy.linalg import expm, sinm, cosm

# TO DO: create function for matrix bracket and for running the robot after giving appropriate variables
# TO DO: create a frame or reference to display the predicted pose of the robot
# TO DO: create some display of error or a calculation of error once the robot has reached the frame

# Request user input for joint angles
print( 'Enter choice of 6 joint variables in degrees: ')
thetas = [int(x) for x in input().split()]
# thetas = [60, 60, 60, 60, 60, 60]
# Initializing dimensions

z_base_to_origin = 0.16316/2
z_base_to_arm1 = (0.275 - z_base_to_origin)
z_arm1_to_arm2 = 0.410
y_arm2_to_arm3 = -0.0098
z_arm2_to_arm3 = 0.2073
y_arm3_to_arm4 = y_arm2_to_arm3 + 0.0741*np.sin(np.pi/6)
z_arm3_to_arm4 = 0.0741*np.cos(np.pi/6)
y_arm4_to_arm5 = y_arm3_to_arm4 + 0.0741*np.sin(np.pi/6) 
z_arm4_to_arm5 = 0.0741*np.cos(np.pi/6)

# Initializing coordinate transformation matrix: M

R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
p = np.array([[0], [y_arm2_to_arm3 + y_arm3_to_arm4 + y_arm4_to_arm5], [z_base_to_origin + z_base_to_arm1 + z_arm1_to_arm2 + z_arm2_to_arm3 + z_arm3_to_arm4 + z_arm4_to_arm5]])

temp0 = np.concatenate((R, p), axis = 1)
M = np.concatenate((temp0 , np.array([[0, 0, 0, 1]])), axis = 0)

# initializing screw axis

w1 = np.array([0, 0, -1])
w2 = np.array([0, 1, 0])
w3 = np.array([0, -1, 0])
w4 = np.array([0, 0, -1])
w5 = np.array([0, -np.sin(np.pi/3), -np.cos(np.pi/3)])
w6 = np.array([0, 0, -1])

q1 = np.array([0, 0, z_base_to_origin])
q2 = np.array([0, 0, q1[2] + z_base_to_arm1])
q3 = np.array([0, 0, q2[2] + z_arm1_to_arm2])
q4 = np.array([0, y_arm2_to_arm3, q3[2] + z_arm2_to_arm3])
q5 = np.array([0, y_arm3_to_arm4, q4[2] + z_arm3_to_arm4])
q6 = np.array([0, y_arm4_to_arm5, q5[2] + z_arm4_to_arm5])


temp1 = np.matmul(brack3by3(w1).dot(-1),q1)
S1 = np.concatenate( (w1, temp1), axis = 0)
expm1 = expm(brack4by4(S1)*(thetas[0]*np.pi/180))

temp2 = np.matmul(brack3by3(w2).dot(-1),q2)
S2 = np.concatenate( (w2, temp2), axis = 0)
expm2 = expm(brack4by4(S2)*(thetas[1]*np.pi/180))

temp3 = np.matmul(brack3by3(w3).dot(-1),q3)
S3 = np.concatenate( (w3, temp3), axis = 0)
expm3 = expm(brack4by4(S3)*(thetas[2]*np.pi/180))

temp4 = np.matmul(brack3by3(w4).dot(-1),q4)
S4 = np.concatenate( (w4, temp4), axis = 0)
expm4 = expm(brack4by4(S4)*(thetas[3]*np.pi/180))

temp5 = np.matmul(brack3by3(w5).dot(-1),q5)
S5 = np.concatenate( (w5, temp5), axis = 0)
expm5 = expm(brack4by4(S5)*(thetas[4]*np.pi/180))

temp6 = np.matmul(brack3by3(w6).dot(-1),q6)
S6 = np.concatenate( (w6, temp6), axis = 0)
expm6 = expm(brack4by4(S6)*(thetas[5]*np.pi/180))

T = expm1 @ expm2 @ expm3 @ expm4 @ expm5 @ expm6 @ M
print(" ")
print("T = ")
print(T)

# note that the height of the concrete block has a height of 0.271, this needs to be accounted for in the pose z value
