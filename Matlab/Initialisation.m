%% create robot matrix
open_system('robot.slx')
S=sim('robot.slx')
[robot,importInfo] = importrobot(gcs)
robot.DataFormat = 'column';

%% load robot matrix

% load('/home/alessandrini/Documents/Robot/Matlab/robot')
