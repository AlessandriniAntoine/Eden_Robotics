from utils import *

"""
Units :
    - distance : m
    - weight : g
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
q1 = np.array([-h0,0,l0])
q2 = np.array([-h0,d1,l0+l1])
q3 = np.array([-h0,d1-d2,l0+l1+l2])
q4 = np.array([-h0-l3,d1-d2+d3,l0+l1+l2])
qc = np.array([-h0-l3-hc,d1-d2+d3+dc,l0+l1+l2+lc])
qe = np.array([-h0-l3-le,d1-d2+d3+de,l0+l1+l2+he])

# initial configuration
m_c = np.array([[0,0,1,qc[0]],[0,-1,0,qc[1]],[1,0,0,qc[2]],[0,0,0,1]]) # config camera in state frame

m_e = np.array([[1,0,0,qe[0]],[0,-1,0,qe[1]],[0,0,-1,qe[2]],[0,0,0,1]]) # config end effector in state frame
t_ce = np.dot(np.linalg.inv(m_c),m_e) # config end effector in camera frame

# rotation
s_w1 = np.array([0,0,-1])
s_w2 = np.array([0,1,0])
s_w3 = np.array([0,-1,0])
s_w4 = np.array([0,-1,0])


# screw axis
s1 = changePointScrew(s_w1,np.array([0,0,0]),q1)
s2 = changePointScrew(s_w2,np.array([0,0,0]),q2)
s3 = changePointScrew(s_w3,np.array([0,0,0]),q3)
s4 = changePointScrew(s_w4,np.array([0,0,0]),q4)

screw_list = np.array([s1,s2,s3,s4]).T

translation_ce = np.dot(t_ce,np.array([0,0,0,1])) # translation from camera to end effector in camera frame
print(translation_ce)