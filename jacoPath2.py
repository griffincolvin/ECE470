from ece470_lib import *
import numpy as np
from numpy.linalg import inv, norm
import vrep
import jacoKinematics as jk
import time
from jacoData import *
from vrepHelpers import *
# from mathHelpers import *

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
    return (max_val - min_val) * np.random.random_sample(shape)+min_val

def collision_check_line(t_start, t_end, p_robot, r_robot, S, p_obstacle, r_obstacle, num=20):
    for theta in matrix_linspace(t_end, t_start, num, to_end=True):
        tpts = multi_transform(p_robot, S, theta)
        if collision_check(tpts, r_robot, p_obstacle, r_obstacle):
            return True
    return False

def findPath(t_start, t_goal, p_robot, r_robot, S, p_obstacle, r_obstacle, dprint=False):
    t_start = np.unwrap(t_start)
    t_goal = np.unwrap(t_goal)
    if dprint:
        print("PATH PLANNING\n start:\n{}\n end:\n{}".format(str(t_start), str(t_goal)))
    ts = Tree(t_start)
    te = Tree(t_goal)
    r_theta = t_start
    N = S.shape[1]
    max_iter = 100
    path_found = collision_check_line(t_start, t_goal, p_robot, r_robot, S, p_obstacle, r_obstacle)
    if path_found:
        te.insert(t_start, t_goal)
    
    while max_iter > 0 and not path_found:
        max_iter -= 1
        r_theta = random_theta(min_val=-np.pi, max_val=np.pi, shape=t_start.shape)
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
        if dprint:
            print("iter: "+str(max_iter))

    if max_iter is 0:
        print("Failed")
        return None

    if dprint:
        print("Path Planning Successful!")
        print(ts.getElements())
        print(te.getElements())


    # Assemble the return array
    ret = [r_theta]
    while ret[0] is not None:
        ret.insert(0, ts.parent(ret[0]))
    while ret[-1] is not None:
        ret.append(te.parent(ret[-1]))
    return np.hstack(ret[1:-1])    
    
def input_thetas(n=6):
    print("Enter choice of {} joint variables in degrees".format(n))
    input_str = input().split()
    if len(input_str) is n:
        return (np.reshape([np.pi/180 * float(t) for t in input_str], (n,1)), False)
    return (None, True)

if __name__ == '__main__':
    dprint = True
    
    # print(thetas)
    vrep.simxFinish(-1)
    # Connect to V-REP (raise exception on failure)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        raise Exception('Failed connecting to remote API server')
    # Start simulation
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

    jointHands = getJoiHands(clientID,'Jaco')
    res,jacoFrame = vrep.simxGetObjectHandle(clientID, "Jaco",vrep.simx_opmode_blocking)

    t_start = np.zeros((6,1))
    S = np.hstack(JacoScrewMatrix())
    # jacoM = getJacoZeroPose()

    p_robot = np.array([[0, 0, 0, 0, 0, 0], [-0.0192, -0.0192, -0.0192, 0.0098, -0.0036, 0.1054], [0.3027, 0.4027, 0.5027, 0.6934, 0.8087, 0.8908]])
    r_robot = np.array([[0.05, 0.05, 0.05, 0.05, 0.05, 0.05]])
    p_obstacle = np.array([[0, -0.375, -0.375, 0], [0, -0.3, -0.325, -0.325], [0.1589, 0.400,  0.85, 0.550]])
    r_obstacle = np.array([[0.075, 0.05, 0.05, 0.05]])

    t_goal, err = input_thetas()
    while not err:
        path = findPath(t_start, t_goal, p_robot, r_robot, S, p_obstacle, r_obstacle, dprint=dprint)
        for t in np.hsplit(path, path.shape[1]):
            jk.jaco_move_theta_interpolate(clientID, t, num = 20)
        t_start = t_goal
        t_goal, err = input_thetas()
        print("Next position:")

    print('Invalid input, quitting')

    time.sleep(5)

    # Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)
    # Close the connection to V-REP
    vrep.simxFinish(clientID)