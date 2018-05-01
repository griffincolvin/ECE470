from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import jacoKinematics as jk
import vrep
import time
from vrepHelpers import *
from mathHelpers import *
from ece470_lib import *

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


def get_input(origin = [.5, .4, .1], x_end = [.5, -0.4, .1] , y_dir = [.5, .4, .5], DEBUG = False, max_dist = 0.01):
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

    xvec = xend - origin
    yvec = yend - origin
    yvec =(np.linalg.norm(xvec))*yvec/np.linalg.norm(yvec)

    pts = [ [x*xvec + y*yvec + origin for x, y in zip(xs, ys)] for xs, ys in zip(cursor.xpts, cursor.ypts)]

    pts2 = []
    for line in pts:
        tpts = []
        for r_start, r_end in zip(line[:-1], line[1:]):
            tpts = tpts + dist_linspace(r_start, r_end, max_dist)
        pts2.append(tpts)

    pts = pts2

    if DEBUG:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        for l in pts:
            l = np.hstack(l)
            x = l[0,:]
            y = l[1,:]
            z = l[2,:]
            ax.plot(x,y,z)
        plt.show()

    plane_norm = np.cross(xvec.T, yvec.T).T
    plane_norm = plane_norm/np.linalg.norm(plane_norm)
    y_rot = np.atleast_2d([[0],[0],[1]])
    x_rot = np.cross(y_rot.T, plane_norm.T).T
    rot = np.hstack([x_rot, y_rot, plane_norm])

    poses = []
    for line in pts:
        tp= []
        for pos in line:
            tp.append(toPose(rot, pos))
        poses.append(tp)

    if DEBUG:
        print(poses)
    return poses


def dist_linspace(m1, m2, max_dist):
    dist = np.linalg.norm(m1 - m2)
    return matrix_linspace(m1, m2,int(dist/max_dist) + 1)

if __name__ == "__main__":

    print(get_input())