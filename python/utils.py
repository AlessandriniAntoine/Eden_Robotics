import modern_robotics as mr
import numpy as np
from math import sqrt,cos,sin,pi

def changePointScrew(s_w,s_v,p):
    """ Function to define the screw axis in an other point
    Parameters :
        - s_w : rotation in point A
        - s_v : translation in point A
        - p : new point
    """
    s_v = s_v - np.dot(mr.VecToso3(s_w),p)
    s = np.concatenate((s_w,s_v))
    return s

def point2Homogenous(point):
    t = np.array([[1,0,0,point[0]],[0,1,0,point[1]],[0,0,1,point[2]],[0,0,0,1]])
    return t