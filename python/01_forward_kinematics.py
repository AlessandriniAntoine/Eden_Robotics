"""
Forward Kinematics in space frame
"""

from parameters import * 

thetalist = np.array([0,0,0,0])

# configuration
t = mr.FKinSpace(m,screw_list,thetalist)
p = t.dot(np.array([0,0,0,1]))[:-1]
print(t)