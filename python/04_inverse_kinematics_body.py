"""
Inverse kinematics in body frame
"""

from numpy import s_
from parameters_body import * 

point = np.array([-0.331,0.0597,0.3453])
thetalist_init = np.array([0.2,0.4,0.4,0.4])
t =point2Homogenous(point)
[thetalist,success] = mr.IKinBody(screw_list,m,t,thetalist_init,eomg,ev)
print(thetalist)
