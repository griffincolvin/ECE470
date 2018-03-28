from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import jacoKinematics as jk

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
origin = np.atleast_2d([1,1,1]).transpose()
xend =   np.atleast_2d([2,2,1]).transpose()
yend =   np.atleast_2d([1,1,2]).transpose()

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
