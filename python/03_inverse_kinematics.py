"""
Inverse kinematics in space frame
"""

from parameters import * 

# point = np.array([-0.331,0.0597,0.3453])
# t = point2Homogenous(point)
thetalist_init = np.array([0.01,0.02,0,0])
t =np.array([
 [-1.  ,     0.   ,    0.   ,   -0.33104],
 [ 0.  ,     0.   ,    1.   ,    0.05971],
 [ 0.  ,    -1.   ,    0.   ,    0.34531],
 [ 0.  ,     0.   ,    0.   ,    1.     ]])
[thetalist,success] = mr.IKinSpace(screw_list,m,t,thetalist_init,eomg,ev)
print(success)
print(thetalist)