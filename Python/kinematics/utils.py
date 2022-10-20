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
    """ 
    transform point into homogeneous matrix
    """
    
    t = np.array([[1,0,0,point[0]],[0,1,0,point[1]],[0,0,1,point[2]],[0,0,0,1]])
    return t


def cuboid_data(pos, size=(1,1,1)):
    # code taken from
    # https://stackoverflow.com/a/35978146/4124317
    # suppose axis direction: x: to left; y: to inside; z: to upper
    # get the (left, outside, bottom) point
    o = [a - b for a, b in zip(pos, size)]
    # get the length, width, and height
    l, w, h = size
    x = [[o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]]]  
    y = [[o[1], o[1], o[1] + w, o[1] + w, o[1]],  
         [o[1], o[1], o[1] + w, o[1] + w, o[1]],  
         [o[1], o[1], o[1], o[1], o[1]],          
         [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]]   
    z = [[o[2], o[2], o[2], o[2], o[2]],                       
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],   
         [o[2], o[2], o[2] + h, o[2] + h, o[2]],               
         [o[2], o[2], o[2] + h, o[2] + h, o[2]]]               
    return np.array(x), np.array(y), np.array(z)

def plotCubeAt(pos=(0,0,0),ax=None,size = (1,1,1)):
    # Plotting a cube element at position pos
    if ax !=None:
        X, Y, Z = cuboid_data( pos, size )
        ax.plot_surface(X, Y, Z, color='r', rstride=1, cstride=1, alpha=1)