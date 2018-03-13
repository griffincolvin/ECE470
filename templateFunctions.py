import numpy as np
import math
__all__ = ['brack3by3' , 'brack4by4', 'rotationMatrixToEulerAngles']

def brack3by3( inputVector):
    a = inputVector[0]
    b = inputVector[1]
    c = inputVector[2]
    return np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])

def brack4by4(inputVector):
    R = brack3by3(inputVector[:-3])
    p = inputVector[-3:]
    temp = np.concatenate((R, p.reshape(3,1)), axis = 1)
    return np.concatenate( (temp, np.zeros((1,4)) ) , axis = 0)

def rotationMatrixToEulerAngles(R) : 
    
    sy = math.sqrt(R[2,1] * R[2,1] +  R[2,2] * R[2,2])
  
    x = math.atan2(R[2,1] , R[2,2])
    y = math.atan2(-R[2,0], sy)
    z = math.atan2(R[1,0], R[0,0])
 
    return np.array([x, y, z])
