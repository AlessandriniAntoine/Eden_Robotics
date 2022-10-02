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

# length of each part
l1 = 0.06931
l2 = 0.116
l3 = 0.160
l4 = 0.155 
l5 = 0.2362

# deviation of each part
d1 = 0.05908
d2 = 0.018
d3 = 0.042
d4 = 0.01413
d5 = 0.0105

h1 = 0.06016

# initial configuration
m = np.array([[-1,0,0,h1-l4-l5],[0,0,1,d1+d2-d3+d4+d5],[0,-1,0,l1+l2+l3],[0,0,0,1]])

# rotation
s_w1 = np.array([0,0,-1])
s_w2 = np.array([0,1,0])
s_w3 = np.array([0,-1,0])
s_w4 = np.array([0,1,0])

# position 
p1 = np.array([h1,d1,l1])
p2 = np.array([h1,d1+d2,l1+l2])
p3 = np.array([h1,d1+d2-d3,l1+l2+l3])
p4 = np.array([h1-l4,d1+d2-d3+d4,l1+l2+l3])
p5 = np.array([h1-l4-l5,d1+d2-d3+d4+d5,l1+l2+l3])

# screw axis
s1 = changePointScrew(s_w1,np.array([0,0,0]),p1)
s2 = changePointScrew(s_w2,np.array([0,0,0]),p2)
s3 = changePointScrew(s_w3,np.array([0,0,0]),p3)
s4 = changePointScrew(s_w4,np.array([0,0,0]),p4)

screw_list = np.array([s1,s2,s3,s4]).T


###################
# numerical 
###################

eomg = 100000
ev = 0.01