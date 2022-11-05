"""
Forward Kinematics in space frame
"""

from parameters import * 
import csv
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

# joint values in radian
pelvis_limits = list(np.arange(-1.57,1.57,0.2))
shoulder_limits = list(np.arange(-1.57,0.69,0.3))
elbow_limits = list(np.arange(-1.57,1.05,0.4))
wrist_limits = list(np.arange(0,1.57,0.4))

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
            if shoulder != -1.57:
                elbow = -1.57
            for wrist in wrist_limits:
                if shoulder != -1.57 and elbow != 1.05:
                    wrist = 0 

                thetalist = np.array([pelvis,shoulder,elbow,wrist])

                # configuration
                t = mr.FKinSpace(m_e,screw_list,thetalist)
                p = t.dot(np.array([0,0,0,1]))[:-1]
                if -0.12<p[0]<0.12 and -0.06<p[1]<0.06 and 0<p[2]<0.074:
                    continue
                x.append(p[0])
                y.append(p[1])
                z.append(p[2])
                writer.writerow([pelvis,shoulder,elbow,wrist]+list(p))

f.close()

x = np.array(x)
y = np.array(y)
z = np.array(z)
c = x*y*z

print(c)

# open figure
fig = plt.figure(1)

# syntax for 3-D projection
ax = plt.axes(projection ='3d')

# plotting
ax.scatter(x, y, z,c=c,alpha=0.8,marker=".")
ax.plot3D([p1[0],p2[0],pe[0]], [p1[1],0,pe[1]], [p1[2],p2[2],pe[2]], 'red',ms=0.2)
plotCubeAt(pos=pos,ax=ax,size = size)

# label
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.set_title('3D workspace')

ax.set_xlim(-0.6,0.2)
ax.set_ylim(-0.6,0.6)
ax.set_zlim(-0.3,0.8)
plt.show()

# plot xy plan
# plt.axes()
# plt.plot(x,y,".b",ms = 3,alpha=0.3)
# plt.plot([p1[0],p2[0],pe[0]], [p1[1],0,pe[1]],"-r")
# rectangle = plt.Rectangle((-0.12,-0.06), 0.24, 0.12, fc='red',ec="red")
# plt.gca().add_patch(rectangle)
# plt.title('xy plan')
# plt.xlabel('x (m)')
# plt.ylabel('y (m)')

# # plot xz plan
# plt.axes()
# plt.plot(x,z,".b",ms = 3,alpha=0.3)
# plt.plot([p1[0],p2[0],pe[0]], [p1[2],p2[2],pe[2]],"-r")
# rectangle = plt.Rectangle((-0.12,0), 0.24, 0.074, fc='red',ec="red")
# plt.gca().add_patch(rectangle)
# plt.title('xz plan')
# plt.xlabel('x (m)')
# plt.ylabel('z (m)')

# # # plot yz plan
# plt.figure()
# plt.plot(y,z,".b",ms = 3,alpha=0.3)
# plt.plot([p1[1],0,pe[1]], [p1[2],p2[2],pe[2]],"-r")
# rectangle = plt.Rectangle((-0.06,0), 0.12, 0.074, fc='red',ec="red")
# plt.gca().add_patch(rectangle)
# plt.title('yz plan')
# plt.xlabel('y (m)')
# plt.ylabel('z (m)')
# plt.show()