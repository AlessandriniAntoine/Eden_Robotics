"""
Inverse kinematics in body frame
"""

from numpy import s_
from parameters_body import * 

desired_positions = [[-0.45270, 0.00063, 0.3455],[-0.1286, 0.06948, 0.07537],[-0.2259, 0.1668, 0.4583]]
thetalist_inits = [[0.,0.,0,0],[0.7854, 0.6981, 0.7006, 1.377],[0.7855, 0.6981, -0.1312, 0.6766]]

eomg = 10
ev = 0.1

for i in range(len(desired_positions)):
    position = desired_positions[i]
    thetalist_init = np.array(thetalist_inits[i])
    t = point2Homogenous(position)

    [thetalist,success] = mr.IKinBody(screw_list,m_e,t,thetalist_init,eomg,ev)

    print(f'Success : {success}')
    print(f'Initial guess : {thetalist_init}')
    print(f'Joints position : {thetalist}')

    if success:
        t = mr.FKinBody(m_e,screw_list,thetalist)
        p = t.dot(np.array([0,0,0,1]))[:-1]
        dist = np.linalg.norm(position - p)
        print(f'Target position : {position}')
        print(f'Final Position : {p}')
        print(f'Euclidien distance : {dist}\n')
    else:
        print('No solution found.\n')
        
