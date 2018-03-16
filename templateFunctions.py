import numpy as np
import math
__all__ = ['brack3by3' , 'brack4by4', 'rot2eul', 'eul2rot', 'revS', 'deg2rad', 'rad2deg']

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

def rot2eul(R) : 
    
    sy = math.sqrt(R[2,1] * R[2,1] +  R[2,2] * R[2,2])
  
    x = math.atan2(R[2,1] , R[2,2])
    y = math.atan2(-R[2,0], sy)
    z = math.atan2(R[1,0], R[0,0])
 
    return np.array([x, y, z])

def eul2rot(theta) :
        
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         np.cos(theta[0]), -np.sin(theta[0]) ],
                    [0,         np.sin(theta[0]), np.cos(theta[0])  ]
                    ])
            
            
                        
    R_y = np.array([[np.cos(theta[1]),    0,      np.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-np.sin(theta[1]),   0,      np.cos(theta[1])  ]
                    ])
                    
    R_z = np.array([[np.cos(theta[2]),    -np.sin(theta[2]),    0],
                    [np.sin(theta[2]),    np.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                        
                        
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def revS(a,q) :
    w = a
    v = np.matmul(brack3by3(a).dot(-1),q)
    
    return np.concatenate((w,v), axis=0)

def deg2rad(deg) :
    res = deg*(np.pi/180)

    return res

def rad2deg(rad) :
    res = rad*(180/np.pi)

    return res