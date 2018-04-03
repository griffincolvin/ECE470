import numpy as np
from numpy.linalg import *
# from vrepHelpers import *
from mathHelpers import *
from scipy.linalg import expm, sinm, cosm

def JacoScrewMatrix():
    zo= 8.2370 * (10**-2)
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

    S1 = revS(a1,q1)
    S2 = revS(a2,q2)
    S3 = revS(a3,q3)    
    S4 = revS(a4,q4)
    S5 = revS(a5,q5)
    S6 = revS(a6,q6)

    return [S1, S2, S3, S4, S5, S6]

def getJacoZeroPose():
    zo= +8.2370e-02
    j6 = np.array([(-3.2792e-05),(+1.1769e-01),(+9.7239e-01-zo)])
    R =  eul2rot(np.array([70, 0, 0]))
    p = np.reshape(j6,(3,1))
    temp0 = np.concatenate((R, p), axis = 1)
    return np.concatenate((temp0 , np.array([[0, 0, 0, 1]])), axis = 0)

def getJacoZOffset():
    return +8.2370e-02

def getJacoSpheres():

    joiRadii = np.array([.05,.05,.05,.075,.035,.035,.035,.15,.1,.05])

    return joiRadii