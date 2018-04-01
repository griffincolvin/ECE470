import vrep
import numpy as np
import mathHelpers as mh

def getJoiHands (clientID,robotname) :
    res,jh1 = vrep.simxGetObjectHandle(clientID,robotname + '_joint1',vrep.simx_opmode_blocking)
    res,jh2 = vrep.simxGetObjectHandle(clientID,robotname + '_joint2',vrep.simx_opmode_blocking)
    res,jh3 = vrep.simxGetObjectHandle(clientID,robotname + '_joint3',vrep.simx_opmode_blocking)
    res,jh4 = vrep.simxGetObjectHandle(clientID,robotname + '_joint4',vrep.simx_opmode_blocking)
    res,jh5 = vrep.simxGetObjectHandle(clientID,robotname + '_joint5',vrep.simx_opmode_blocking)
    res,jh6 = vrep.simxGetObjectHandle(clientID,robotname + '_joint6',vrep.simx_opmode_blocking)
    jointHands = np.array([jh1,jh2,jh3,jh4,jh5,jh6])

    return jointHands

def setObjPos (clientID,objHand,refHand,pos) :
    vrep.simxSetObjectPosition(clientID,objHand,refHand,pos,vrep.simx_opmode_blocking)

    return

def setObjOrient (clientID, objHand, refHand, euls) :
    vrep.simxSetObjectOrientation(clientID, objHand, refHand, euls, vrep.simx_opmode_blocking)

    return

def setJoiTargPos (clientID,joiHand,targPos) :
    vrep.simxSetJointTargetPosition(clientID,joiHand,targPos,vrep.simx_opmode_blocking)

    return

def getJoiPos (clientID, joiHand) :
    res,joiPos = vrep.simxGetJointPosition(clientID, joiHand, vrep.simx_opmode_blocking)

    return joiPos

def setObjPose (clientID, objHand, refHand, goalPose) :
    goalRot,goalPos = mh.fromPose(goalPose)
    goalEuls = mh.rot2eul(goalRot)
    setObjPos(clientID,objHand,refHand,goalPos)
    setObjOrient(clientID,objHand,refHand,goalEuls)

    return
