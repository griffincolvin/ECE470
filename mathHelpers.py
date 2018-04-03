import numpy as np
from numpy.linalg import inv, norm
from scipy.linalg import expm, logm
import math
__all__ = ['brack3' , 'brack6', 'rot2eul', 'eul2rot', 'revS', 'prismS', 'deg2rad', 'rad2deg', 'adjbrack', 'printEuls', 'printPos', 'findIK', 'evalT', 'toPose','checkColl']

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

def toPose(rot, pos):
    """
    Returns a 4x4 HCT matrix given by the 3x3 rotation matrix and 3x1 postion vector
    :param rot: A 3x3 Rotation Matrix
    :param pos: A 3x1 Position Vector
    :returns: A 4x4 HTC matrix as a numpy array
    """
    return np.block([[ rot, pos  ],
                     [ 0,0,0,1   ]])

def fromPose(T):
    """
    Returns a rotation matrix and position vector from a 4x4 HCT matrix
    :param T: The 4x4 HCT matrix as either python lists or numpy array
    :returns: a tuple with the first element being a 3x3 numpy array representing the rotation matrix
              and the second element being a 3x1 numpy array position vector
    """
    T = np.asarray(T)
    return (T[:3,:3], T[:3, 3:4])

def toScrew(a, q=None):
    """
    Returns the space screw of some prismatic or revolute joint as a 6x1 numpy array.
    If a q is supplied, the returned screw will be revolute; if no q, screw will be prismatic.
    Can use either python list, list of lists, or numpy array as inputs in XYZ order
    :param a: The axis of motion for a prismatic screw or axis of revolution. Should have norm 1 (not checked)
    :param q: A point passing through the axis if a revolute joint
    :returns: A 6x1 numpy matrix representing the screw axis
    """
    a = np.atleast_2d(a).reshape((3,1))
    # Revolute Screw
    if q is not None:
        q = np.atleast_2d(q).reshape((3,1))
        return np.block([[ a                 ],
                         [ bracket(q).dot(a) ]])
    # Prismatic Screw
    return np.block([[ np.zeros((3,1)) ],
                     [ a               ]])

def toTs(S, theta):
    """
    Generates a list of HCT matricies from a list of screw axes and joint variables. Not that useful for general work,
    but used by other functions. Note that numpy arrays of screw axes are not supported, only python lists of screw axes.
    Use np.hsplit(S, N) to generate a list of screw axes given a numpy array S where N is the number of joints (cols in the matrix) 
    :param S: A python list of 6x1 screw axes
    :param theta: A list/numpy array of joint vars. Should have the same number of elements as S
    :returns: A python list of 4x4 HCT matricies representing a transformation by each of the screw axes
    """
    return [expm(bracket(s) * t) for s, t in zip(S, theta)]

def evalT(S, theta, M):
    """
    Basically Forward Kinematics 
    Finds the end position of a robot based on space screw axes, joint vars and the space 'zero HCT'
    Note that numpy arrays of screw axes are not supported, only python lists of screw axes.
    Use np.hsplit(S, N) to generate a list of screw axes given a numpy array S where N is the number of joints (cols in the matrix) 
    :param S: A python list of 6x1 screw axes from the base to the manipulator
    :param theta: A python list/numpy array of joint vars in the same order as S.
    :param M: A 4x4 HCT transformation matrix representing the pose of the end effector when theta = 0 for all joint vars
    :returns: A numpy 4x4 HCT transformation matrix representing the pose of the end effector at the given joint vars
    """
    ret = np.identity(4)
    for t in toTs(S, theta):
        ret = ret.dot(t)
    return ret.dot(M)

def evalJ(S, theta):
    """
    Finds the space jacobian of a robot with given screw axes at a given joint positions:
    Note that numpy arrays of screw axes are not supported, only python lists of screw axes.
    Use np.hsplit(S, N) to generate a list of screw axes given a numpy array S where N is the number of joints (cols in the matrix)
    TODO: Improve efficeny by removing the need to recompute the transformation for each screw
    :param S: a python list of 6x1 screw axes
    :param theta: a python list/numpy array of joint vars. Should be same number of elements as S
    :returns: A 6xN matrix representing the space Jacobian of the robot with the given screw axes at the given joint vars
    """
    T = toTs(S, theta)
    J = [S[0]]
    for i in range(1, len(S)):
        col = T[0]
        for j in range(1, i):
            col = col.dot(T[j])
        J.append(adj_T(col).dot(S[i]))
    return np.hstack(J)

##
## The following code will (probably) not be included with Exam 4
##

def findIK(endT, S, M, theta=None, max_iter=100, max_err = 0.001, mu=0.05):
    """
    Basically Inverse Kinematics
    Uses Newton's method to find joint vars to reach a given pose for a given robot. Returns joint positions and 
    the error. endT, S, and M should be provided in the space frame. Stop condiditons are when the final pose is less than a given
    twist norm from the desired end pose or a maximum number of iterations are reached. 
    Note that numpy arrays of screw axes are not supported, only python lists of screw axes.
    Use np.hsplit(S, N) to generate a list of screw axes given a numpy array S where N is the number of joints (cols in the matrix) 
    TODO: Improve internal type flexibilty of input types
    :param endT: the desired end pose of the end effector
    :param S: a python list of 6x1 screw axes in the space frame
    :param M: the pose of the end effector when the robot is at the zero position
    :param theta: Optional - An initial guess of theta. If not provided, zeros are used. Should be a Nx1 numpy matrix
    :param max_iter: Optional - The maximum number of iterations of newtons method for error to fall below max_err. Default is 10
    :param max_err: Optional - The maximum error to determine the end of iterations before max_iter is reached. Default is 0.001 and should be good for PL/quizes
    :param mu: The normalizing coefficient (?) when computing the pseudo-inverse of the jacobian. Default is 0.05
    :returns: A tuple where the first element is an Nx1 numpy array of joint variables where the algorithm ended. Second 
              element is the norm of the twist required to take the found pose to the desired pose. Essentially the error that PL checks against.
    """
    if  theta is None:
        theta = np.zeros((len(S),1))
    V = np.ones((6,1))
    while np.linalg.norm(V) > max_err and max_iter > 0:
        curr_pose = evalT(S, theta, M)
        V = inv_bracket(logm(endT.dot(inv(curr_pose))))  
        J = evalJ(S, theta)
        pinv = inv(J.transpose().dot(J) + mu*np.identity(len(S))).dot(J.transpose())
        thetadot = pinv.dot(V)
        #thetadot = np.linalg.pinv(J,mu).dot(V)
        theta = theta + thetadot
        max_iter -= 1

    
    return (theta, np.linalg.norm(V))



def checkColl(S,M,thetas,radii,spheres_o,spheres_st):
    """
    Checks for collisions at given thetas using a robot's S, M, joint radii, and starting joint positions
    """
    def transform_pts(P, S, theta, M):
        P.insert(0, np.atleast_2d([[0],[0],[0]]))
        N = len(S)
        T = toTs(S, theta)
        ret = []
        T.insert(0, np.identity(4))
        T.insert(0, np.identity(4))
        for i in range(len(T)):
            temp = np.identity(4)
            for t in T[:i+1]:
                temp = temp.dot(t)
            ret.append(temp.dot(np.vstack([ P[i] , [[1]]])))
        return np.hstack(ret)[:3]

    # numSpheres = len(S) + 2 #Assumes end effector and base included

    spheres2 = transform_pts(spheres_o,S,thetas,M)
    spheres3 = np.hsplit(spheres2,8)[1:]

    for s in spheres_st:
        spheres3.append(s)
    # for r in extradii:
    #     radii.append(r)
   
    ret = 1*np.atleast_2d([[np.linalg.norm(s1-s2) < r1+r2 for s1, r1 in zip(spheres3, radii)] for s2, r2 in zip(spheres3, radii)])-np.identity(len(spheres3))
    print(ret)

    return ret
            



            
             

