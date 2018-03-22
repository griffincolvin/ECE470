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

def combine_matrix(m):
    """
    Combines a list of matricies into a single numpy matrix
    Essentially stacks every row of the input list into a numpy matrix,
    then stacks all the rows vertically.
    :param m: The list of lists to convert
    """
    return np.vstack([np.hstack(row) for row in m])


def bracket(v):
    """
    Returns the 'bracket' operator of a 3x1 vector or 6x1 twist
    :param v: the 3x1 vector or 6x1 twist, can be of type list or numpy.ndarray - Must be convertible to a numpy array!
    :returns: a 3x3 or 4x4 numpy array based on the input matrix or an empty list otherwise
    """
    v = np.asarray(v)
    rtn = []
    if(v.shape == (6,1)):
        rtn =   [[ bracket(v[:3]),  v[3:]   ],
                 [ np.zeros((1,4))          ]]
        rtn = combine_matrix(rtn)
    elif(v.shape == (3,1)):
        rtn = np.zeros((3,3))
        rtn[0][1] = - v[2]
        rtn[0][2] =   v[1]
        rtn[1][2] = - v[0]
        rtn = rtn - rtn.transpose()
    return rtn

def inv_bracket(m):
    """
    Performs the inverse 'bracket' operation on a 3x3 or 4x4 matrix
    :param m: the 3x3 skew-symmetric matrix or 4x4 bracket of a twist - Must be convertible to a numpy array!
    :returns: the vector or twist representation of the input matrix or an empty list otherwise
    """
    rtn = []
    m = np.asarray(m)
    if(m.shape == (4,4)):
        rtn =   [[ inv_bracket(m[:3,:3])],
                 [ m[:3,3:]             ]]
        rtn = combine_matrix(rtn)
    elif(m.shape == (3,3)):
        m = m - m.transpose()
        rtn = np.zeros((3,1))
        rtn[2] = - m[0][1]/2
        rtn[1] =   m[0][2]/2
        rtn[0] = - m[1][2]/2
    return rtn

def adj_T(T):
    """
    Returns the adjoint transformation matrix of T
    :param T: the pose whose 6x6 adjoint matrix to return
    """
    rot, pos = fromPose(T)
    return combine_matrix([[ rot,                   np.zeros((3,3)) ],
                           [ bracket(pos).dot(rot), rot             ]])


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
    
def toTs(S, theta):
    return [expm(bracket(s) * t) for s, t in zip(S, theta)]

def evalT(S, theta, M):
    ret = np.identity(4)
    for t in toTs(S, theta):
        ret = ret.dot(t)
    return ret.dot(M)

def evalJ(S, theta):
    T = toTs(S, theta)
    J = [S[0]]
    for i in range(1, len(S)):
        col = T[0]
        for j in range(1, i):
            col = col.dot(T[j])
        J.append(adj_T(col).dot(S[i]))
    return np.hstack(J)

def findIK(endT, S, M, theta=None, max_iter=100, max_err = 0.001, mu=0.05):
    if  theta is None:
        theta = np.zeros((len(S),1))
    V = np.ones((6,1))
    while np.linalg.norm(V) > max_err and max_iter > 0:
        curr_pose = evalT(S, theta, M)
        V = inv_bracket(logm(endT.dot(inv(curr_pose))))
        J = evalJ(S, theta)
        pinv = inv(J.transpose().dot(J) + mu*np.identity(len(S))).dot(J.transpose())
        thetadot = pinv.dot(V)
        theta = theta + thetadot
        max_iter -= 1;
    return (theta, np.linalg.norm(V))