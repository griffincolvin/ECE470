#import vrep
import time
import numpy as np
from numpy.linalg import *
from templateFunctions import *
from scipy.linalg import expm, sinm, cosm

def ForwardKinematics(thetas):


    # Initializing dimensions

    z_base_to_origin = 0.0856
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


    v1 = np.matmul(brack3by3(w1).dot(-1),q1)
    S1 = np.concatenate( (w1, v1), axis = 0)
    expm1 = expm(brack4by4(S1)*(thetas[0]*np.pi/180))

    v2 = np.matmul(brack3by3(w2).dot(-1),q2)
    S2 = np.concatenate( (w2, v2), axis = 0)
    expm2 = expm(brack4by4(S2)*(thetas[1]*np.pi/180))

    v3 = np.matmul(brack3by3(w3).dot(-1),q3)
    S3 = np.concatenate( (w3, v3), axis = 0)
    expm3 = expm(brack4by4(S3)*(thetas[2]*np.pi/180))

    v4 = np.matmul(brack3by3(w4).dot(-1),q4)
    S4 = np.concatenate( (w4, v4), axis = 0)
    expm4 = expm(brack4by4(S4)*(thetas[3]*np.pi/180))

    v5 = np.matmul(brack3by3(w5).dot(-1),q5)
    S5 = np.concatenate( (w5, v5), axis = 0)
    expm5 = expm(brack4by4(S5)*(thetas[4]*np.pi/180))

    v6 = np.matmul(brack3by3(w6).dot(-1),q6)
    S6 = np.concatenate( (w6, v6), axis = 0)
    expm6 = expm(brack4by4(S6)*(thetas[5]*np.pi/180))

    T = expm1 @ expm2 @ expm3 @ expm4 @ expm5 @ expm6 @ M
    print(" ")
    print("T = ")
    print(T)
    R = np.array([[T[0,0], T[0,1], T[0,2]], [T[1,0], T[1,1], T[1,2]], [T[2,0], T[2,1], T[2,2]]])
    print("")
    
    print("Predicted Position")
    
    print("Expected X value: " + str(T[0,3]))
    print("Expected Y value: " + str(T[1,3]))
    print("Expected Z value: " + str(T[2,3] + 0.271) )

    print("")
    
    print("Predicted Orientation")
    
    print("Expected alpha value: " + str(rotationMatrixToEulerAngles(R)[0] * (180/np.pi)))
    print("Expected beta value: " + str(rotationMatrixToEulerAngles(R)[1]* (180/np.pi)))
    print("Expected gamma value: " + str(rotationMatrixToEulerAngles(R)[2]* (180/np.pi)))

    # note that the height of the concrete block has a height of 0.271, this needs to be accounted for in the pose z value

    return