from ece470_lib import *
import numpy as np
from numpy.linalg import inv, norm
import vrep
import jacoKinematics as jk
import time
from jacoData import *
from vrepHelpers import *
from mathHelpers import *


# def multi_transform(pts, S, theta):
#     Ns = 0
#     Np = 0
#     if isinstance(S, np.ndarray):
#         Ns = S.shape[1]
#         S = np.hsplit(S, Ns)
#     else:
#         Ns = len(S)
#     theta = np.asarray(theta).flatten()
#     if isinstance(pts, np.ndarray):
#         Np = pts.shape[1]
#         pts = np.hsplit(pts, Np)
#     else:
#         Np = len(pts)
#     pts = [np.vstack([p, [[1]] ]) for p in pts]
#     T = [evalT(S[:n], theta[:n]) for n in range(1,Ns+1)]
#     while Ns < Np:
#         T.insert(0, np.identity(4))
#         Ns += 1
#     return np.hstack([t.dot(p) for t, p in zip(T, pts)])[:3]

def collision_check(pts, radii, pts2, radii2, dprint=False):
    Np = 0
    Np = pts.shape[1]
    pts = np.hsplit(pts, Np)
    radii = radii.flatten()
    for p1, r1 in zip(pts, radii):
        for p2, r2 in zip(pts, radii):
            if p1 is not p2:
                if norm(p1 - p2) < r1 + r2:
                    if dprint:
                        print("self col")
                    return True
                
    pts2 = np.hsplit(pts2, pts2.shape[1])
    radii2 = radii2.flatten()
    for p1, r1 in zip(pts, radii):
        for p2, r2, in zip(pts2, radii2):
            if norm(p1 - p2) <= r1 + r2:
                if dprint:
                    print(p1)
                    print("")
                    print(p2)
                    print("obstacle col")
                return True
    return False

def random_theta(min_val=0, max_val=1, shape=(1,)):
    return (max_val - min_val)*np.random.random_sample(shape)+min_val

def collision_check_line(t_start, t_end, p_robot, r_robot, S, p_obstacle, r_obstacle, num=100):
    for theta in matrix_linspace(t_start, t_end, num, to_end=True):
        tpts = multi_transform(p_robot, S, theta)
        if collision_check(tpts, r_robot, p_obstacle, r_obstacle):
            return True
    return False

print( 'Enter choice of 6 joint variables in degrees: ')
thetas = [float(x) for x in input().split()]
for i in range(0,6):
    thetas[i] = deg2rad(thetas[i])
    # Close all open connections (just in case)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

configs = []


jointHands = getJoiHands(clientID,'Jaco')
th1_o = getJoiPos(clientID,jointHands[0])
th2_o = getJoiPos(clientID,jointHands[1])
th3_o = getJoiPos(clientID,jointHands[2])
th4_o = getJoiPos(clientID,jointHands[3])
th5_o = getJoiPos(clientID,jointHands[4])
th6_o = getJoiPos(clientID,jointHands[5])

thetas_o = np.array([th1_o,th2_o,th3_o,th4_o,th5_o,th6_o]) # thetas_start ?

jacoScrew = JacoScrewMatrix() # S ?
jacoM = getJacoZeroPose() # M ?

# i dont think this is right, we still need the radii and the position of all of the 
jacoRadii = getJacoSpheres() # r_robot & r_obstacles ????

jointHands = getJoiHands(clientID,'Jaco') # r_robot ?

print("theta_start:     " + str(thetas_o))
print("M:               " + str(jacoM))
print("Radii:           " + str(jacoRadii))
print("S:               " + str(jacoScrew))
print("jointHands:      " + str(jointHands))
## Get theta goal from the user

M = np.asarray(jacoM)
S = np.asarray(jacoScrew)
theta_goal = thetas

# Im attempting to get the parameters that are used in the path planning code to work for our values that apply to the robot and obstacles that are in the scene: PATH_SCENE.ttt, 
# these are the relevant numbers, but they need to work with the implementation below
S = np.array([[0, 0, 0, 0, 0, 0] , [0, 1, -1, 0,  -0.819152044, -0.939692621] ,  [-1, 0, 0, -1, -0.573576436, 0.342020143],  [0, -0.19713, 0.60713, -0.0097827, 0.695811061, 0.876597577], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
M = np.array([[ 1.0,  0.0,  0.0, -3.27920000e-05],[ 0.0,  3.42020143e-01, -9.39692621e-01,  1.17690000e-01],[ 0.0,  9.39692621e-01,  3.42020143e-01,  8.90020000e-01], [ 0.0,  0.0,  0.0, 1.0]])
p_robot = np.array([[0, 0, 0, 0, 0, 0], [-0.0192, -0.0192, -0.0192, 0.0098, -0.0036, 0.1054], [0.3027, 0.4027, 0.5027, 0.6934, 0.8087, 0.8908]])
r_robot = np.array([[0.05, 0.05, 0.05, 0.05, 0.05, 0.05]])
p_obstacle = np.array([[0, -0.375, -0.375, 0], [0, -0.3, -0.325, -0.325], [0.1589, 0.400,  0.85, 0.550]])
r_obstacle = np.array([[0.075, 0.05, 0.05, 0.05]])
theta_start = np.array([[np.pi], [np.pi], [np.pi], [np.pi], [np.pi], [np.pi]])

# the theta goal can me modified, i just used this for testing.
theta_goal = np.array([[np.pi + 0.5], [np.pi + 0.5], [np.pi+0.5], [np.pi+0.5], [np.pi+0.5], [np.pi+0.5]])



ts = Tree(theta_start)
te = Tree(theta_goal)
N = S.shape[1]
max_iter = 100
path_found = False
while max_iter > 0 and not path_found:
    max_iter -= 1
    r_theta = random_theta(min_val=-np.pi, max_val=np.pi, shape=theta_start.shape)
    min_theta_start = None
    b1 = False
    b2 = False
    min_dist = 100000
    for t in ts:
        if norm(t-r_theta) < min_dist:
            min_theta_start = t
            min_dist = norm(t-r_theta)
    if min_theta_start is not None and not collision_check_line(min_theta_start, r_theta, p_robot, r_robot, S, p_obstacle, r_obstacle):
        ts.insert(r_theta, min_theta_start)
        b1 = True
    min_dist = 100000
    for t in te:
        if norm(t-r_theta) < min_dist:
            min_theta_end = t
            min_dist = norm(t-r_theta)
    if min_theta_end is not None and not collision_check_line(min_theta_end, r_theta, p_robot, r_robot, S, p_obstacle, r_obstacle):
        te.insert(r_theta, min_theta_end)
        b2 = True
    path_found = b1 and b2
if max_iter is 0:
    print("Failed")

ret = [r_theta]
app = ts.parent(r_theta)
while app is not None:
    ret.insert(0, app)
    app = ts.parent(app)
ret.insert(0, theta_start)
app = te.parent(r_theta)
while app is not None:
    ret.append(app)
    app = te.parent(app)
ret.append(theta_goal)

print(np.hstack(ret))
ts.getElements()



# Dummy4 = [0  -0.0192 0.3027] 
# r4 = 0.05
# Dummy = [0  -0.0192 0.4027]
# r = 0.05
# Dummy3 = [0  -0.0192 0.5027]
# r3 = 0.05
# Dummy5 = [0  0.0098 0.6934]
# r5 = 0.05
# Dummy6 = [0  -0.0036 0.8087]
# r6 = 0.05
# Dummy2 = [0  0.1054 0.8908]
# r2 = 0.05

# Dummy1 = [0  0.00 0.1589] 
# r1 = 0.075
# Dummy8 = [-0.375 -0.300 0.400] 
# r8 = 0.05
# Dummy7 = [0.3750 -0.3250 0.85] 
# r7 = 0.05
# Dummy0 = [0 -0.3250 0.550]
# r0 = 0.05


# res,dummy = vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_blocking)
# res,dummy0 = vrep.simxGetObjectHandle(clientID,'Dummy0',vrep.simx_opmode_blocking)
# res,dummy1 = vrep.simxGetObjectHandle(clientID,'Dummy1',vrep.simx_opmode_blocking)
# res,dummy2 = vrep.simxGetObjectHandle(clientID,'Dummy2',vrep.simx_opmode_blocking)
# res,dummy3 = vrep.simxGetObjectHandle(clientID,'Dummy3',vrep.simx_opmode_blocking)
# res,dummy4 = vrep.simxGetObjectHandle(clientID,'Dummy4',vrep.simx_opmode_blocking)
# res,dummy5 = vrep.simxGetObjectHandle(clientID,'Dummy5',vrep.simx_opmode_blocking)
# res,dummy6 = vrep.simxGetObjectHandle(clientID,'Dummy6',vrep.simx_opmode_blocking)
# res,dummy7 = vrep.simxGetObjectHandle(clientID,'Dummy7',vrep.simx_opmode_blocking)
# res,dummy8 = vrep.simxGetObjectHandle(clientID,'Dummy8',vrep.simx_opmode_blocking)