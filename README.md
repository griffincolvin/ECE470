# Implementing Basic 6 DOF Movement in V-REP

## ECE470 Project Team : @jeczajk2 @gcolvin2 @namanj2

## Table of Contents

### __1. Starting off using V-REP with Python API__

    1a. Installation of necessary software
    1b. Connecting to V-REP through Python remote API
    1c. Example scene creation and setup
    1d. Develop and run the main script
    1e. Refining script based on the simulation
    1f. References

### __2. Applying forward kinematics using V-REP__

    2a. Determining physical parameters
    2b. Initializing coordinate transformation matrix
    2c. Intitializing screw axes
    2d. Calculating homogeneous transformation matrix
    2e. Implentating forward kinematic function

### __3. Applying inverse kinematics using V-REP__

    3a. Determining DH parameters
    3b.
---

## Starting off using V-REP with Python API

### __1a. Installation of necessary software__

This instructable is to be followed using V-REP simulation software. Many different languages are supported by V-REP, but English will be the language used throughout this `README.md`. The following examples and explanations were executed on a computer running Windows 10 x64, Python 3.6, and Anaconda 5.1.

#### V-REP

Download your OS version of V-REP EDU at: http://www.coppeliarobotics.com/downloads.html <br> Rename main V-REP install folder to something easier to type like `vrep`

#### Anaconda (Python 3.6)

If not already installed on your computer, a suitable choice for Python 3.6/2.7 is Anaconda and can be found at: https://www.anaconda.com/download/

### __1b. Connecting to V-REP through Python remote API__

In order to control V-REP simulations through a terminal, three files from V-REP's installation folders are required to be pulled and put in a seperate folder.

__Files:__<br>
`vrep/programming/remoteApiBindings/python/python/vrep.py`
`vrep/programming/remoteApiBindings/python/python/vrepConst.py`
`vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib`

Download `jacoPaint_JDN.py` and place inside the same folder as the previous three files

Continue to a terminal/cmd prompt, change directory to the V-REP install path and run `vrep.exe`

__Note:__ the Console Hide setting under `Tools > User Settings` might have to be changed first before the terminal/cmd prompt will stay open on V-REP launch

Finally, change the directory to the newly created folder in preparation for Python script execution

### __1c. Example scene creation and setup__

There are two options for this step, either download and open the provided `jacoPaint.ttt` file or recreate the scene as follows:

1. Drag and drop a Kinova Robotics Jaco robot arm onto the workspace
2. Continue to right click the robot and select `Edit > Remove > Associated Child Script`
3. Drag and drop a paint nozzle tool onto the workspace
4. Select the attachment point in the Jaco arm dropdown list and `Assemble` it to the paint nozzle tool
5. Drag and drop a projector screen at a reasonable distance away from the Jaco arm while also being placed with the nozzle facing away from the screen

### __1d. Develop and run the main script__

The `jacoPaint_JGN.py` file provided is the only script necessary to complete this simulation. It is comprised of a few key sections.

#### Script setup

* This section begins with importing the needed Python packages `numpy`,`vrep`, and `time`. The connection is then made to V-REP through the `vrep.simxStart` function using system default port and connection settings.

#### Joint initalization

* All 6 joints are then initialized through the `vrep.simxGetObjectHandle` function and given a respective variable name of jointHands_x_.The simulation is then started through `vrep.simxStartSimulation`. Directly after, each joint position is gathered by calling `vrep.simxGetJointPosition` and assigning each to a variable theta_x_.

#### Making moves

* The desired joint rotations to generate the desired question mark symbol on the canvas are then implemented with specific `time.sleep()` functions inbetween in order to allow motions to be completed in full. The rotations are done by calling the `simxSetJointTargetPosition` function with the desired jointHands_x_ and theta_x_ variables. The rotation angle is specified by adding a specfic angle in radians to the initial theta.

#### Closing out

* To wrap up the script, the simulation is ended through `vrep.simxStopSimulation` and than the connection to V-REP is closed.

### __1e. Refining script based on the simulation__

From within the newly created folder from before, run `python jacoPaint_JDN.py` from within the terminal/cmd prompt. The simulation should then begin inside of the V-REP window. Based on the simulation results, one should be able to step through the script and make any necessary changes in order to achieve the desired result.

This simple simulation using the Jaco robot arm is a great demonstration of how quickly V-REP can be picked up and put to use. The supporting video for this section can be found at <https://www.youtube.com/watch?v=IwFgBY0sgR0> and was recorded using V-REP's simple in-house screen recorder. This can be easily found within the left sidebar of the GUI.

### __1f. References__

__Python V-REP Remote API Cheatsheet :__
<http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm>

__Anaconda Cheatsheet :__
<https://conda.io/docs/_downloads/conda-cheatsheet.pdf>

__Kinova Robotics :__
<http://www.kinovarobotics.com>

---

## Applying forward kinematics using V-REP

### __2a. Determining physical parameters__

---

## Applying inverse kinematics using V-REP

### Code Setup

In order to demonstrate inverse kinematics functionaltiy, an method of providing an input pose and solving for the joint variables needed to achieve that pose must be created. For our project, we chose to have the user draw an arbitrary shape on and have the robot draw the same shape on a screen in the V-REP simulated evironment. The code that obtains the user-input and solves the joint-variables is located in the 'jacoIK.py' file.

For the user input portion of the code, an empty, interactive, matplotlib frame is displayed. The user can use the cursor to draw an arbitary shape, recoreded as a sequence of lines, on the display. Once the user exits the menu, the points are transformed in space onto the known location of the screen in V-REP. From the point locations, poses are generated that point the drawing tool normal to the screen. Then for the starting point, a numerical inverse-kinematic function is used to generate the joint variables to achieve the given pose. 

The inverse kinematics function used was a numerical newton-raphson method to find a robot joint-variables to get arbitrarily close to the given pose. By starting at a given pose with a given set of joint variables, the robot jacobian is found and used to determine the direction each joint needs to move to get closer to the given pose. This is essentially the forward kinematics of the robot. Once the error is small enough, the joint variables are simply returned. Should the pose be unreachable, the algorithm will not converge and a indication is returned after a set maximum number of iterations.

If multiple poses that are nearby need to be generated, the alogirthm can be initialized with a joint-variables that produce a nearby pose. This provides faster convergence as the robot response to small changes to joint varaibles produce a nearly linear response.

### V-REP Setup

For the demo, a Jaco robot with a ?marker/felt-tip pen tool and a screen are used. The robot is placed at a known location and orientation so that its position relative to the screen can be measured. By finding the coordinates of the bottom-left of the screen as the origin and the bottom-right and top-left as the end of the x and y 'axes', the user input drawing can be easily transformed into a set of points that lie on the screen.

Once the joint angles from the previous code are known, the forward-kinematics of the robot are used to drive the robot to each location in the correct sequence to draw the input onto the screen in V-REP.

As the user has complete freedom over a large and varied input space, the program demonstrates true inverse kinematics functionality as the user input can accurately be converted into robot motion.

## Demonstrating Collision Detection

### Code Setup

A common consideration in robotics is that of collision between robot joint arms during the motion of a robot. A solution to this problem comes through the application of bounding volumes attached to the robot's joint arms. By applying simple bounding volumes, such as spheres to the robot arms, then calculating the goal position of the joints and checking for collision between the arms, we can detect where collisions will occur, if any, and protect the robots in use.

The method described above is applied in code by first taking the desired joint angles and then calculating the forward kinematics. From the forward kinematics, the pose of each joint on the robot can be calculated and from the positions of each joint arm and the locations and size of the bounding volume, possible collisions can be checked for by comparing the end position of each bounding volume and seeing if it comes within range of another bounding volume, if so, a potential collision has been detected.

### V-REP Setup

For the demo, a Jaco robot with spheres attached at the necessary reference points is places in an environment with spheres attached to make sure that collision with external objects doesnt occur. This scene is titled "JacoCD.ttt" and can be opened once V-REP is launced as per the instructions described above. Next, to achieve orientations and check for collision detection, the user should execute "python JacoCD.py" and enter in values for the 6 joint angles. If no collision is detected, the final pose of the robot will be displayed and the robot will move to the position accordingly. However, if a collision is detected, whether with the robot itself or with an external object, the user will be notified and the objects in collision will be displayed. For the purposes of visualization, the robot will still have its attempt to reach the input angles simulated in V-REP.
