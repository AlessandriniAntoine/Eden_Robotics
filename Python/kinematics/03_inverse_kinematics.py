"""
Inverse kinematics in space frame
"""

from parameters import * 

desired_positions = [[-0.45270, 0.00063, 0.3455],[-0.1286, 0.06948, 0.07537],[-0.2259, 0.1668, 0.4583]]
thetalist_inits = [[0,0,0,0],[pi/4,-pi/3,0,pi/3],[pi/4,pi/6,-pi/6,pi/3]]

desired_positions = [
    [   [ 1.000e+00,  0.000e+00,  0.000e+00, -4.527e-01]
        [ 0.000e+00, -1.000e+00,  0.000e+00,  6.300e-04]
        [ 0.000e+00,  0.000e+00, -1.000e+00,  3.455e-01]
        [ 0.000e+00,  0.000e+00,  0.000e+00,  1.000e+00]],
    
    [   [-0.35355339, -0.70710678,  0.61237244, -0.12860143]
        [ 0.35355339, -0.70710678, -0.61237244,  0.06949239]
        [ 0.8660254,   0.,          0.5,        -0.07533818]
        [ 0.,          0.,          0.,          1.        ]],

    [   [ 7.07106781e-01, -7.07106781e-01, -1.05176252e-17, -2.25866038e-01]
        [-7.07106781e-01, -7.07106781e-01,  1.05176252e-17,  1.66756992e-01]
        [-3.31998990e-17,  0.00000000e+00, -1.00000000e+00,  4.58298002e-01]
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
]

eomg = 10
ev = 0.1

for i in range(len(desired_positions)):
    desired = desired_positions[i]
    thetalist_init = np.array(thetalist_inits[i])
    # t = point2Homogenous(position)


    [thetalist,success] = mr.IKinSpace(screw_list,m_e,t,thetalist_init,eomg,ev)

    print(f'Success : {success}')
    print(f'Initial guess : {thetalist_init}')
    print(f'Joints position : {thetalist}')

    if success:
        t = mr.FKinSpace(m_e,screw_list,thetalist)
        p = t.dot(np.array([0,0,0,1]))[:-1]
        dist = np.linalg.norm(position - p)
        print(f'Target position : {position}')
        print(f'Final Position : {p}')
        print(f'Euclidien distance : {dist}\n')
    else:
        print('No solution found.\n')
        
