import numpy as np
import math
__all__ = ['brack3' , 'brack6', 'rot2eul', 'eul2rot', 'revS', 'prismS', 'deg2rad', 'rad2deg', 'adjbrack', 'printEuls', 'printPos']

def brack3( inputVector) :
    a = inputVector[0]
    b = inputVector[1]
    c = inputVector[2]
    return np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])

def brack6(vector)  :
    w = vector[:3]
    v = vector[3:]
    temp = np.concatenate((brack3(w),v), axis = 1)
    return np.concatenate( (temp, np.zeros((1,4)) ) , axis = 0)

def adjbrack(Tmatrix) :
    R = np.array([[Tmatrix[0,0], Tmatrix[0,1], Tmatrix[0,2]], [Tmatrix[1,0], Tmatrix[1,1], Tmatrix[1,2]], [Tmatrix[2,0], Tmatrix[2,1], Tmatrix[2,2]]])
    p = np.array( [ Tmatrix[0,3],Tmatrix[1,3],Tmatrix[2,3] ] )
    top = np.concatenate((R,np.zeros(3,3)) , axis = 1)
    bot = np.concatenate( (np.matmul(brack3(p),R), R), axis=1)
    return np.concatenate((top,bot), axis = 0)

def rot2eul(R) : 
    if R[0][2] < 1:
        if R[0][2] > -1:
            b = np.arcsin(R[0][2])
            a = np.arctan2(-R[1][2],R[2][2])
            g = np.arctan2(-R[0][1],R[0][0])
        else:
            b = -np.pi/2
            a = np.arctan2(R[1][0],R[1][1])
            g = 0
    else:
        b = np.pi/2
        a = np.arctan2(R[1][0],R[1][1])
        g = 0

    return np.array([a,b, g])

def eul2rot(theta) :
    a = deg2rad(theta[0])
    b = deg2rad(theta[1])
    g = deg2rad(theta[2])

    rx = np.array([[1,0,0],
                   [0,np.cos(a),-np.sin(a)],
                    [0,np.sin(a),np.cos(a)]])
    ry = np.array([[np.cos(b),0,np.sin(b)],
                   [0,1,0],
                    [-np.sin(b),0,np.cos(b)]])
    rz = np.array([[np.cos(g),-np.sin(g),0],
                   [np.sin(g),np.cos(g),0],
                    [0,0,1]])
    tempR = np.matmul(ry,rz)
    R = np.matmul(rx, tempR)

    return R

def revS(a,q) :
    w = a
    v = np.matmul(brack3(a).dot(-1),q)
    
    return np.concatenate((w,v), axis=0)

def prismS(a,q) :
    w = np.array([0],[0],[0])
    v = a
    
    return np.concatenate((w,v), axis=0)

def deg2rad(deg) :
    res = deg*(np.pi/180)

    return res

def rad2deg(rad) :
    res = rad*(180/np.pi)

    return res


def printEuls(eulvec,oldornew) :

    return print(" " + oldornew + " Orientation \n" + oldornew + " alpha value: " + str(rad2deg(eulvec[0])) + "\n" + oldornew + " beta value: " + str(rad2deg(eulvec[1]))+ "\n" + oldornew + " gamma value: " + str(rad2deg(eulvec[2])))
    
def printPos(posvec,oldornew) :

    return print(" " + oldornew + " Position \n" + oldornew + " x value: " + str(posvec[0]) + "\n" + oldornew + " y value: " + str(posvec[1])+ "\n" + oldornew + " z value: " + str(posvec[2]))
    