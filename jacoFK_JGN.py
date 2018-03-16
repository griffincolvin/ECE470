#import vrep
import time
import numpy as np
from numpy.linalg import *
from templateFunctions import *
from scipy.linalg import expm, sinm, cosm


def fKin(thetas):


    # Initializing dimensions
    # Space frame set at Jaco base
    zo= 0.08237
    yo=0
    xo=0
    j1 = np.array([0,0,(0.078381)])
    j2 = np.array([0,0,(0.2795)])
    j3 = np.array([0,0,(0.68950)])
    j4 = np.array([-0.000032767,0.0097827,(0.8968)])
    j5 = np.array([-0.000032792,0.044032,(0.96263)])
    j6 = np.array([-0.000032792,0.11769,(0.97239)])

    # Initializing coordinate transformation matrix: M
    
    R =  eul2rot(np.array([deg2rad(70), 0, 0]))
    p = np.reshape(j6,(3,1))
    temp0 = np.concatenate((R, p), axis = 1)
    M = np.concatenate((temp0 , np.array([[0, 0, 0, 1]])), axis = 0)

    # initializing screw axis

    a1 = np.array([0, 0, -1])
    a2 = np.array([0, 1, 0])
    a3 = np.array([0, -1, 0])
    a4 = np.array([0, 0, -1])
    a5 = np.array([0, -np.sin((35)*(180/np.pi)), -np.cos((35)*(180/np.pi))])
    a6 = np.array([0, -np.cos((20)*(180/np.pi)), np.sin((20)*(180/np.pi))])

    S1 = revS(a1,j1)
    expm1 = expm(brack4by4(S1)*deg2rad(thetas[0]))
    S2 = revS(a2,j2)
    expm2 = expm(brack4by4(S2)*deg2rad(thetas[1]))
    S3 = revS(a3,j3)    
    expm3 = expm(brack4by4(S3)*deg2rad(thetas[2]))
    S4 = revS(a4,j4)
    expm4 = expm(brack4by4(S4)*deg2rad(thetas[3]))
    S5 = revS(a5,j5)
    expm5 = expm(brack4by4(S5)*deg2rad(thetas[4]))
    S6 = revS(a6,j6)
    expm6 = expm(brack4by4(S6)*deg2rad(thetas[5]))

    T = expm1 @ expm2 @ expm3 @ expm4 @ expm5 @ expm6 @ M
    print(" ")
    print("T = ")
    print(T)
    goalr = np.array([[T[0,0], T[0,1], T[0,2]], [T[1,0], T[1,1], T[1,2]], [T[2,0], T[2,1], T[2,2]]])
    goaleuler = rot2eul(goalr)
    goalp = np.array( [ T[0,3],T[1,3],T[2,3] ] )
   
    return goaleuler, goalp