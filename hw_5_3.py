
# coding: utf-8

# In[1]:


from ece470_lib import *
import numpy as np
from numpy.linalg import inv, norm


# In[2]:


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


# In[3]:


#1


# In[18]:


import numpy as np

S = np.array([[ 0, -1], [-1,  0], [ 0,  0], [ 0,  0], [ 0,  0], [-2, -2]])
M = np.array([[ 0,  0, -1,  0], [-1,  0,  0, -4], [ 0,  1,  0,  0], [ 0,  0,  0,  1]])
p_robot = np.array([[ 0,  2,  0,  0], [ 0,  0, -2, -4], [ 0,  0,  0,  0]])
r_robot = np.array([[0.90, 0.90, 0.90, 0.90]])
p_obstacle = np.array([[3.63, 3.37, -0.77, 1.26, -4.01, 3.15, 1.07, 3.54, -0.14, 2.91, -0.48,  -4.65, 0.52, -4.85, 1.68], [1.89, 3.53, -0.02, 2.66, -1.17, -4.90, 1.80, 3.37, 3.16, 2.08, 2.40,  -2.51, 1.68, 2.84, 4.93], [3.45, -4.28, 3.71, 4.77, -4.24, -1.36, 4.61, 3.37, 3.74, -3.34, 4.07,  -1.17, -4.49, 4.54, -2.45]])
r_obstacle = np.array([[1.46, 0.88, 2.13, 3.33, 2.41, 1.33, 1.06, 1.26, 2.88, 0.81, 3.34, 1.56,  1.64, 4.36, 1.54]])
theta_start = np.array([[-3.05], [3.02]])
theta_goal = np.array([[1.95], [2.51]])


# In[19]:


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
ret


# In[21]:


np.hstack(ret)


# In[22]:


#2


# In[29]:


import numpy as np

S = np.array([[ 0,  0,  1], [ 0,  1,  0], [ 0,  0,  0], [ 0,  0,  0], [ 0,  0,  0], [ 1,  0, -4]])
M = np.array([[ 1,  0,  0,  2], [ 0, -1,  0,  2], [ 0,  0, -1,  0], [ 0,  0,  0,  1]])
p_robot = np.array([[0, 0, 0, 2, 2], [0, 2, 4, 4, 2], [0, 0, 0, 0, 0]])
r_robot = np.array([[0.90, 0.90, 0.90, 0.90, 0.90]])
p_obstacle = np.array([[-4.36, 2.37, 2.63, -4.27, 0.83, 0.81, 4.22, 0.88, 4.94, -2.64, 0.65,  2.56, 3.13, 4.06, 4.27], [4.03, -1.82, -3.67, 3.91, -4.07, -0.99, -0.27, -4.87, -4.75, -3.25,  -2.93, -1.40, -3.80, -1.15, 0.67], [3.90, 0.21, -4.81, -4.57, -4.71, 4.64, -0.96, -2.13, 3.73, 3.22, -4.92,  -2.06, 4.91, -2.97, 4.73]])
r_obstacle = np.array([[1.52, 0.69, 1.92, 3.28, 2.26, 2.73, 2.19, 2.45, 4.14, 2.14, 3.68, 1.93,  2.01, 2.86, 1.47]])
theta_start = np.array([[0.48], [0.23], [1.20]])
theta_goal = np.array([[2.91], [-3.05], [-0.54]])


# In[30]:


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
np.hstack(ret)


# In[28]:


ts.getElements()

