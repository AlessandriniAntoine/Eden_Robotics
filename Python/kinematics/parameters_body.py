# parameters in body frame

from utils import *
"""
unit : 
    - distance : m
    - mass : kg
    - time : s
"""
###################
# mechanical 
###################

# main length 
l0 = 0.069
l1 = 0.116
l2 = 0.16
l3 = 0.155
lc = 0.053
le = 0.2377

# deviation along y
d1 = 0.018
d2 = 0.042
d3 = 0.01413
dc = 0.0105
de = 0.0105

h0 = 0.06
hc = 0.0815
he = 0.0005

# position 
q1 = np.array([-lc-l2-l1,d1-d2+d3+dc,l3+hc])
q2 = np.array([-lc-l2,dc+d3-d2,l3+hc])
q3 = np.array([-lc,dc+d3,l3+hc])
q4 = np.array([-lc,dc,hc])
qc = np.array([0,0,0])
qe = np.array([-lc-he,dc+de,hc+le])


# position space frame for config 
qc_s = np.array([-h0-l3-hc,d1-d2+d3+dc,l0+l1+l2+lc])
qe_s = np.array([-h0-l3-le,d1-d2+d3+de,l0+l1+l2+he])

# configuration
m_c = np.array([[0,0,1,qc_s[0]],[0,-1,0,qc_s[1]],[1,0,0,qc_s[2]],[0,0,0,1]]) # config camera in state frame
m_e = np.array([[1,0,0,qe_s[0]],[0,-1,0,qe_s[1]],[0,0,-1,qe_s[2]],[0,0,0,1]]) # config end effector in state frame
t_ce = np.dot(np.linalg.inv(m_c),m_e) # config end effector in camera frame

# rotation
s_w1 = np.array([-1,0,0])
s_w2 = np.array([0,-1,0])
s_w3 = np.array([0,1,0])
s_w4 = np.array([0,1,0])

# screw axis
s1 = changePointScrew(s_w1,np.array([0,0,0]),q1)
s2 = changePointScrew(s_w2,np.array([0,0,0]),q2)
s3 = changePointScrew(s_w3,np.array([0,0,0]),q3)
s4 = changePointScrew(s_w4,np.array([0,0,0]),q4)

screw_list = np.array([s1,s2,s3,s4]).T
translation_ce = np.dot(t_ce,np.array([0,0,0,1])) # translation from camera to end effector in camera frame

###########
# inverse kinematics
###########

eomg = 0
ev = 0