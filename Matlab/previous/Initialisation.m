%% Import from Onshape
% smexportonshape('https://cad.onshape.com/documents/1e4662605519e61dc058d665/w/07df068be4edf05f00108356/e/49578bce04e658622f4f35b3');
% smimport('AssemblageParfait.xml');

%% Initialisation (A toujours lancer)

% from simulink
%parameters
%load('robot.mat')

% open_system('Arm.slx')
S=sim('robot.slx')
[robot,importInfo] = importrobot(gcs)
robot.DataFormat = 'column';

% from workspace
% load('/home/alessandrini/Documents/Robot/Matlab/robot')
