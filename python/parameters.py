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
lc = 0.04429
le = 0.239

# deviation along y
d1 = 0.018
d2 = 0.047
d3 = 0.01413
dc = 0.0105
de = 0.01063

h0 = 0.06
hc = 0.083
he = 5e-4

# position 
q1 = np.array([-h0,0,l0])
q2 = np.array([-h0,d1,l0+l1])
q3 = np.array([-h0,d1-d2,l0+l1+l2])
q4 = np.array([-h0-l3,d1-d2+d3,l0+l1+l2])
qc = np.array([-h0-l3-hc,d1-d2+d3+dc,l0+l1+l2+lc])
qe = np.array([-h0-l3-le,d1-d2+d3+de,l0+l1+l2+he])

# initial configuration
m = np.array([[0,0,1,qc[0]],[0,-1,0,qc[1]],[1,0,0,qc[2]],[0,0,0,1]])

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
###################
# numerical 
###################

eomg = 100000
ev = 0.01