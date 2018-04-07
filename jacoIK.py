from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import jacoKinematics as jk
import vrep
import time
from vrepHelpers import *
from mathHelpers import *


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
origin = np.atleast_2d([.5,.4,.1]).transpose()
xend =   np.atleast_2d([.5,-.4,.1]).transpose()
yend =   np.atleast_2d([.5,.4,.5]).transpose()
# # Shouldn't it be somewhere like this? if the wall is placed at the
# current location in the .ttt file in the repo
# origin = np.atleast_2d([.4,.5,0]).transpose()
# xend =   np.atleast_2d([.4,-.5,0]).transpose()
# yend =   np.atleast_2d([.4,.5,1]).transpose()

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
    
# plt.show()

plane_norm = np.cross(xvec.T, yvec.T).T
plane_norm = plane_norm/np.linalg.norm(plane_norm)
y_rot = np.atleast_2d([[0],[0],[1]])
x_rot = np.cross(y_rot.T, plane_norm.T).T
rot = np.hstack([x_rot, y_rot, plane_norm])

poses = [ [toPose(rot, point) for point in line] for line in pts]



if __name__ == "__main__":
 
    # print(thetas)
    vrep.simxFinish(-1)
    # Connect to V-REP (raise exception on failure)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        raise Exception('Failed connecting to remote API server')
    # Start simulation
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

    res,goalFrame = vrep.simxGetObjectHandle(clientID, 'goalFrame', vrep.simx_opmode_blocking)
    jointHands = getJoiHands(clientID,'Jaco')
    res,jacoFrame = vrep.simxGetObjectHandle(clientID, "Jaco",vrep.simx_opmode_blocking)

    en_drawing = np.array([[0,0,0,0.1],[0,0,0,0],[0,0,0,0],[0,0,0,0]])

    for line in poses:
        first = line[0]
        setObjPose(clientID, goalFrame, jacoFrame, first)
        jk.jaco_move_pose(clientID, first, delay=0)
        time.sleep(2)
        for T in line:
            # Set goal frame dummy to estimated location
            setObjPose(clientID, goalFrame, jacoFrame, T + en_drawing)
            jk.jaco_move_pose(clientID, T + en_drawing, delay=0)
        last = line[-1]
        setObjPose(clientID, goalFrame, jacoFrame, last)
        jk.jaco_move_pose(clientID, last, delay=0)
        time.sleep(1)



    print('Finished motions. Sleeping for 5sec')

    time.sleep(5)

    # Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)
    # Close the connection to V-REP
    vrep.simxFinish(clientID)