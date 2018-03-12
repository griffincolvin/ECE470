import numpy as np

__all__ = ['brack3by3' , 'brack4by4']

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
