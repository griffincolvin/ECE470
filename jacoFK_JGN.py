#import vrep
import time
import numpy as np
from numpy.linalg import *
from vrepHelpers import *
from mathHelpers import *
from scipy.linalg import expm, sinm, cosm


def fKin(thetas):


    # Initializing dimensions
    # Space frame set at Jaco base
    zo= +8.2370e-02
    yo=0
    xo=0
    j1 = np.array([(-3.2712e-05),(-1.7324e-05),(+1.6075e-01-zo)])
    j2 = np.array([(-3.2712e-05),(-1.7324e-05),(+2.7950e-01-zo)])
    j3 = np.array([(-3.2737e-05),(-1.7328e-05),(+6.8950e-01-zo)])
    j4 = np.array([(-3.2767e-05),(+9.7827e-03),(+8.9680e-01-zo)])
    j5 = np.array([(-3.2792e-05),(+4.4032e-02),(+9.6263e-01-zo)])
    j6 = np.array([(-3.2792e-05),(+1.1769e-01),(+9.7239e-01-zo)])

    # Initializing coordinate transformation matrix: M
    
    R =  eul2rot(np.array([70, 0, 0]))
    p = np.reshape(j6,(3,1))
    temp0 = np.concatenate((R, p), axis = 1)
    M = np.concatenate((temp0 , np.array([[0, 0, 0, 1]])), axis = 0)

    # initializing screw axis
    q1=np.reshape(j1,(3,1))
    q2=np.reshape(j2,(3,1))
    q3=np.reshape(j3,(3,1))
    q4=np.reshape(j4,(3,1))
    q5=np.reshape(j5,(3,1))
    q6=np.reshape(j6,(3,1))

    a1 = np.array([[0], [0], [-1]])
    a2 = np.array([[0], [1], [0]])
    a3 = np.array([[0], [-1], [0]])
    a4 = np.array([[0], [0], [-1]])
    a5temp = eul2rot(np.array([125,0,0]))
    a5 = np.reshape(a5temp[:,2], (3,1))
    a6temp = eul2rot(np.array([70,0,0]))
    a6 = np.reshape(a6temp[:,2], (3,1))
    print(a5)
    print(a6)
    S1 = revS(a1,q1)
    expm1 = expm(brack6(S1)*(thetas[0]))
    S2 = revS(a2,q2)
    expm2 = expm(brack6(S2)*(thetas[1]))
    S3 = revS(a3,q3)    
    expm3 = expm(brack6(S3)*(thetas[2]))
    S4 = revS(a4,q4)
    expm4 = expm(brack6(S4)*(thetas[3]))
    S5 = revS(a5,q5)
    expm5 = expm(brack6(S5)*(thetas[4]))
    S6 = revS(a6,q6)
    expm6 = expm(brack6(S6)*(thetas[5]))
   
    T = expm1 @ expm2 @ expm3 @ expm4 @ expm5 @ expm6 @ M
    print(" ")
    print("T = ")
    print(T)
    goalr = np.array([[T[0,0], T[0,1], T[0,2]], [T[1,0], T[1,1], T[1,2]], [T[2,0], T[2,1], T[2,2]]])
    goaleuler = rot2eul(goalr)
    goalp = np.array( [ T[0,3],T[1,3],T[2,3] ] )
   
    return goaleuler, goalp