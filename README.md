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

### __2. Determining and applying forward kinematics using V-REP__

    2a. Determining physical parameters
    2b.
    2c.
    2d.
    2e.
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

## Determining and applying forward kinematics using V-REP

### __2a. Determining physical parameters__