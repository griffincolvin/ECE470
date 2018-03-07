# Demonstrating Robot Motion in V-REP

#### ECE470 Project Team
@jakecza @gcolvin2 @namanj2

---

## Table of Contents

#### 1. Installation of necessary software
* V-REP EDU
* Anaconda (Python3.6)

#### 2. Connecting to V-REP through Python remote API

#### 3. Required scene setup
#### 4. Running the main script
#### 5. Final overview
#### References

---

### 1. Installation of necessary software

#### V-REP EDU

This instructable is to be followed using V-REP simulation software. Recommended steps to downloading the free educational version are:

1. Download your OS version of V-REP EDU at: http://www.coppeliarobotics.com/downloads.html <br>(This instructable was created using V-REP EDU v3.4.0 on Windows x64)
2. Rename main V-REP install folder to something easier to type like `vrep`

#### Anaconda (Python3.6)

If not already installed on your computer, or if some other method of accessing Python 2.7/3.6 is not, then a suitable choice is Anaconda and can be found at: https://www.anaconda.com/download/ <br>(This instructable was created using Anaconda 5.1/Python 3.6)

### 2. Connecting to V-REP through Python remote API

In order to control V-REP simulations through a terminal, three files from V-REP's installation folders are required to be pulled and put in a seperate folder.<br>**Files:**
`vrep/programming/remoteApiBindings/python/python/vrep.py`
`vrep/programming/remoteApiBindings/python/python/vrepConst.py`
`vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib`

Download `jacoPaint_JDN.py` and place inside the same folder as the previous three files

Continue to a terminal/cmd prompt, change directory to the V-REP install path and run `vrep.exe`
**Note:** the Console Hide setting under `Tools > User Settings` might have to be changed first before the terminal/cmd prompt will stay open on V-REP launch

Finally, change the directory to the newly created folder in preparation for Python script execution

### 3. Required scene setup

There are two options for this step, either download and open the provided `jacoPaint.ttt` file or recreate the scene as follows:

1. Drag and drop a Kinova Robotics Jaco robot arm onto the workspace
2. Continue to right click the robot and select `Edit > Remove > Associated Child Script`
3. Drag and drop a paint nozzle tool onto the workspace
4. Select the attachment point in the Jaco arm dropdown list and `Assemble` it to the paint nozzle tool
5. Drag and drop a projector screen at a reasonable distance away from the Jaco arm while also being placed with the nozzle facing away from the screen

### 4. Running the main script

#### Structure

The `jacoPaint_JGN.py` file provided is the only script necessary to complete this simulation. It is comprised of a few key sections.

* Script setup
    * This section begins with importing the needed Python packages `numpy`,`vrep`, and `time`. The connection is then made to V-REP through the `vrep.simxStart` function using system default port and connection settings.
* Joint initalization
    * All 6 joints are then initialized through the `vrep.simxGetObjectHandle` function and given a respective variable name of jointHands_x_.The simulation is then started through `vrep.simxStartSimulation`. Directly after, each joint position is gathered by calling `vrep.simxGetJointPosition` and assigning each to a variable theta_x_.
* Making moves
    * The desired joint rotations to generate the desired question mark symbol on the canvas are then implemented with specific `time.sleep()` functions inbetween in order to allow motions to be completed in full. The rotations are done by calling the `simxSetJointTargetPosition` function with the desired jointHands_x_ and theta_x_ variables. The rotation angle is specified by adding a specfic angle in radians to the initial theta.
* Closing out
    * To wrap up the script, the simulation is ended through `vrep.simxStopSimulation` and then the connection to V-REP is closed.

#### Running the script

From within the newly created folder from before, run `python jacoPaint_JDN.py` from within the terminal/cmd prompt. The simulation should then begin inside of the V-REP window.

### 5. Final overview

This simple simulation using the Jaco robot arm is a great demonstration of how quickly V-REP can be picked up and put to use. The supporting video for this instructable can be found at <https://www.youtube.com/watch?v=IwFgBY0sgR0> and was recorded using V-REP's simple in-house screen recorder. This can be easily found within the left sidebar of the GUI.<br>
Have fun creating your own simulations!

### References

__Python V-REP Remote API Cheatsheet__
<http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm>

__Anaconda Cheatsheet__
<https://conda.io/docs/_downloads/conda-cheatsheet.pdf>

__Kinova Robotics__
<http://www.kinovarobotics.com>