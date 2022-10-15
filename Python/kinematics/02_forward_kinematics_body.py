"""
Forward Kinematics in body frame
"""

from parameters_body import * 

thetalist = np.array([ 0,  0 ,0 ,0])
# configuration
t = mr.FKinBody(m_c,screw_list,thetalist)
p = t.dot(np.array([0,0,0,1]))[:-1]
print(t)