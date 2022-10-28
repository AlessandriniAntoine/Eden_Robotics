"""
Forward Kinematics in space frame
"""

from parameters import * 

thetalists = [[0,0,0,0],[pi/4,-pi/3,0,pi/3],[pi/4,pi/6,-pi/6,pi/3]]

for thetalist in thetalists:
    thetalist = np.array(thetalist)
    # configuration
    t = mr.FKinSpace(m_e,screw_list,thetalist)
    p = t.dot(np.array([0,0,0,1]))[:-1]
    print(p)