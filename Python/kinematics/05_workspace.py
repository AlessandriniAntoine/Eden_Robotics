"""
Forward Kinematics in space frame
"""

from parameters import * 
import csv
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

# joint values
pelvis_limits = list(np.arange(-1.57,1.57,0.2))
shoulder_limits = list(np.arange(-1.57,0.69,0.25))
elbow_limits = list(np.arange(-1.57,1.05,0.25))
wrist_limits = list(np.arange(0,1.57,0.25))

pelvis_limits.append(1.57)
shoulder_limits.append(.069)
elbow_limits.append(1.05)
wrist_limits.append(1.57)

# box value
pos = [0.120, 0.06, 0]
size = (0.240,0.12,-0.074)

# arm position
p1 = q1
p2 = q3
pe =qe

# compute workspace
f = open('workspace.csv','w',newline='')
writer = csv.writer(f)
header = ['pelvis','shoulder','elbow','wrist','x','y','z']
writer.writerow(header)

x,y,z = [],[],[]
for pelvis in pelvis_limits:
    for shoulder in shoulder_limits:
        for elbow in elbow_limits:
            for wrist in wrist_limits: 

                thetalist = np.array([pelvis,shoulder,elbow,wrist])

                # configuration
                t = mr.FKinSpace(m_e,screw_list,thetalist)
                p = t.dot(np.array([0,0,0,1]))[:-1]
                x.append(p[0])
                y.append(p[1])
                z.append(p[2])
                writer.writerow([pelvis,shoulder,elbow,wrist]+list(p))

f.close()

x = np.array(x)
y = np.array(y)
z = np.array(z)
c = x*y*z

# open figure
fig = plt.figure()

# syntax for 3-D projection
ax = plt.axes(projection ='3d')

# plotting
ax.scatter(x, y, z,c=c,alpha=0.1,marker=",")
ax.plot3D([p1[0],p2[0],pe[0]], [p1[1],p2[1],pe[1]], [p1[2],p2[2],pe[2]], 'red')
plotCubeAt(pos=pos,ax=ax,size = size)

# label
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.set_title('3D workspace')

# save image
# ax.view_init(elev=90, azim=0)
# plt.savefig("xy_plan.png")

# ax.view_init(elev=0, azim=0)
# plt.savefig("yz_plan.png")

# ax.view_init(elev=0, azim=90)
# plt.savefig("xz_plan.png")

plt.show()