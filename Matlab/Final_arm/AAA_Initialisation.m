%% Importation depuis Onshape
% smexportonshape('https://cad.onshape.com/documents/1e4662605519e61dc058d665/w/07df068be4edf05f00108356/e/49578bce04e658622f4f35b3');
% smimport('AssemblageParfait.xml');
%% Initialisation (A toujours lancer)

% Via simulink
% AAA_Arm_Data_File
% open_system('AA_Arm.slx')
% S=sim('AA_Arm.slx')
% [robot,importInfo] = importrobot(gcs)
% robot.DataFormat = 'column';

% Via workspace
% load('/home/alessandrini/Documents/Eden_Robotics/Commande/Matlab_V4/A_robot_workspace')
% open_system('/home/alessandrini/Documents/Eden_Robotics/Commande/Matlab_V3/AA_Assemblage.slx')