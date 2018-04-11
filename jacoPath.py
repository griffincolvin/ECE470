from ece470_lib import *
import numpy as np
from numpy.linalg import inv, norm

import vrep
import jacoKinematics as jk
import time
from jacoData import *
from vrepHelpers import *
from mathHelpers import *


def multi_transform(pts, S, theta):
    Ns = 0
    Np = 0
    if isinstance(S, np.ndarray):
        Ns = S.shape[1]
        S = np.hsplit(S, Ns)
    else:
        Ns = len(S)
    theta = np.asarray(theta).flatten()
    if isinstance(pts, np.ndarray):
        Np = pts.shape[1]
        pts = np.hsplit(pts, Np)
    else:
        Np = len(pts)
    pts = [np.vstack([p, [[1]] ]) for p in pts]
    T = [evalT(S[:n], theta[:n]) for n in range(1,Ns+1)]
    while Ns < Np:
        T.insert(0, np.identity(4))
        Ns += 1
    return np.hstack([t.dot(p) for t, p in zip(T, pts)])[:3]

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

S = np.array([[ 0,  0,  1], [ 0,  1,  0], [ 0,  0,  0], [ 0,  0,  0], [ 0,  0,  0], [ 1,  0, -4]])
M = np.array([[ 1,  0,  0,  2], [ 0, -1,  0,  2], [ 0,  0, -1,  0], [ 0,  0,  0,  1]])
p_robot = np.array([[0, 0, 0, 2, 2], [0, 2, 4, 4, 2], [0, 0, 0, 0, 0]])
r_robot = np.array([[0.90, 0.90, 0.90, 0.90, 0.90]])
p_obstacle = np.array([[-4.36, 2.37, 2.63, -4.27, 0.83, 0.81, 4.22, 0.88, 4.94, -2.64, 0.65,  2.56, 3.13, 4.06, 4.27], [4.03, -1.82, -3.67, 3.91, -4.07, -0.99, -0.27, -4.87, -4.75, -3.25,  -2.93, -1.40, -3.80, -1.15, 0.67], [3.90, 0.21, -4.81, -4.57, -4.71, 4.64, -0.96, -2.13, 3.73, 3.22, -4.92,  -2.06, 4.91, -2.97, 4.73]])
r_obstacle = np.array([[1.52, 0.69, 1.92, 3.28, 2.26, 2.73, 2.19, 2.45, 4.14, 2.14, 3.68, 1.93,  2.01, 2.86, 1.47]])
theta_start = np.array([[0.48], [0.23], [1.20]])
theta_goal = np.array([[2.91], [-3.05], [-0.54]])

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

print(thetas_o)
print(jacoM)
print(jacoRadii)
print(jacoScrew)
print(jacoHands)
## Get theta goal from the user


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