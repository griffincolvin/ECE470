from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import jacoKinematics as jk
import vrep
import time
from vrepHelpers import *
from mathHelpers import *


def toPose(rot, pos):
    """
    Returns a 4x4 HCT matrix given by the 3x3 rotation matrix and 3x1 postion vector
    :param rot: A 3x3 Rotation Matrix
    :param pos: A 3x1 Position Vector
    :returns: A 4x4 HTC matrix as a numpy array
    """
    return np.block([[ rot, pos  ],
                     [ 0,0,0,1  ]])


class Cursor(object):
    def __init__(self, ax):
        self.ax = ax
        self.lx = ax.axhline(color='k')  # the horiz line
        self.ly = ax.axvline(color='k')  # the vert line
        self.press = False
        self.xpts = []
        self.ypts = []
        self.xtemp = []
        self.ytemp = []

        # text location in axes coords
        self.txt = ax.text(0.7, 0.9, '', transform=ax.transAxes)
        self.connect()

    def mouse_move(self, event):
        if event.inaxes:
            x = event.xdata
            y = event.ydata
            self.lx.set_ydata(y)
            self.ly.set_xdata(x)
            self.txt.set_text('x=%1.2f, y=%1.2f' % (x, y))

            if self.press:
                self.xtemp.append(x)
                self.ytemp.append(y)
                for xs, ys in zip(self.xpts, self.ypts):
                    self.ax.plot(xs, ys, c='b')
                self.ax.plot(self.xtemp, self.ytemp, c='b')
            plt.draw()

    def button_press(self, event):
        self.press = True
        self.xtemp = []
        self.ytemp = []


    def button_release(self, event):
        self.press = False
        self.xpts.append(self.xtemp)
        self.ypts.append(self.ytemp)
        self.xtemp = []
        self.ytemp = []

    def connect(self):
        plt.connect('motion_notify_event', self.mouse_move)
        plt.connect('button_press_event', self.button_press)
        plt.connect('button_release_event', self.button_release)

fig, ax = plt.subplots()
cursor = Cursor(ax)
plt.axis([0, 1, 0, 1])
plt.show()
plt.close()

#XYZ locations of the corner
origin = np.atleast_2d([.1,.1,.1]).transpose()
xend =   np.atleast_2d([.2,.2,.1]).transpose()
yend =   np.atleast_2d([.1,.1,.2]).transpose()

xvec = xend - origin;
yvec = yend - origin;

pts = [ [x*xvec + y*yvec + origin for x, y in zip(xs, ys)] for xs, ys in zip(cursor.xpts, cursor.ypts)]

fig = plt.figure()
ax = fig.gca(projection='3d')

for l in pts:
    l = np.hstack(l)
    x = l[0,:]
    y = l[1,:]
    z = l[2,:]
    ax.plot(x,y,z)
    
plt.show()

plane_norm = -np.cross(xvec.T, yvec.T).T
plane_norm = plane_norm/np.linalg.norm(plane_norm)
y_rot = np.atleast_2d([[0],[0],[1]])
x_rot = np.cross(y_rot.T, plane_norm.T).T
rot = np.hstack([x_rot, y_rot, plane_norm])

poses = [ [toPose(rot, point) for point in line] for line in pts]

theta = np.zeros((6,1))
joint_vars = []
for line in poses:
    temp = []
    for pose in line:
        theta = jk.jaco_IK(pose, theta)
        temp.append(theta)
    joint_vars.append(temp)

print(temp)

print('First move is:' + str(temp[1]))

goalT = jk.jaco_FK(temp[1])

print('First pose is:' + str(goalT))

goalThetas = temp[1]

print('First set of angles are:' + str(goalThetas))



# Close all open connections (just in case)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

res,goalFrame = vrep.simxGetObjectHandle(clientID, 'goalFrame', vrep.simx_opmode_blocking)
jointHands = getJoiHands(clientID,'Jaco')
res,jacoFrame = vrep.simxGetObjectHandle(clientID, "Jaco",vrep.simx_opmode_blocking)

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

for config in temp:

    goalT = jk.jaco_FK(config)
    # Set goal frame dummy to estimated location
    setObjPose(clientID, goalFrame, jacoFrame, goalT)
    # Move arm to estimated location
    joi1_o = getJoiPos(clientID,jointHands[0])
    joi2_o = getJoiPos(clientID,jointHands[1])
    joi3_o = getJoiPos(clientID,jointHands[2])
    joi4_o = getJoiPos(clientID,jointHands[3])
    joi5_o = getJoiPos(clientID,jointHands[4])
    joi6_o = getJoiPos(clientID,jointHands[5])
    joiPos_o = np.array([joi1_o,joi2_o,joi3_o,joi4_o,joi5_o,joi6_o])

    for i in range(0, 6):
        setJoiTargPos(clientID,jointHands[i],config[i] + joiPos_o[i])
        print("Joint " + str(i+1) + " Moved by " + str(rad2deg(config[i])) + " Degrees")
    

# print('Finished motions. Sleeping for 5sec')
# res,newori = vrep.simxGetObjectOrientation(clientID,jointHands[5],jacoFrame,vrep.simx_opmode_blocking)
# printEuls(newori,'new')
# res,newpos = vrep.simxGetObjectPosition(clientID,jointHands[5],jacoFrame,vrep.simx_opmode_blocking)
# printPos(newpos,'new')
# time.sleep(15)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)