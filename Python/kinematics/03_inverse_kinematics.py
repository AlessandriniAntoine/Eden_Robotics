"""
Inverse kinematics in space frame
"""

from parameters import * 

point = np.array([-0.331,0.0597,0.3453])
t = point2Homogenous(point)
thetalist_init = np.array([0.01,0.02,0,0])

[thetalist,success] = mr.IKinSpace(screw_list,m,t,thetalist_init,eomg,ev)
print(success)
print(thetalist)