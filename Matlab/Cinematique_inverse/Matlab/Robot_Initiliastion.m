% %% Importation depuis Onshape
%smexportonshape('https://cad.onshape.com/documents/1e4662605519e61dc058d665/w/07df068be4edf05f00108356/e/0aa380034bbe73ef5b6c5e07');
%smimport('AssemblageFinal.xml');
%% Initialisation (A toujours lancer)
Robot_DataFile_PLA
open_system('Robot_PLA.slx')
S=sim('Robot_PLA.slx')
[robot,importInfo] = importrobot(gcs)
robot.DataFormat = 'column';
%% trajectoire
C=S.get('courbe');

X=C.Data(:,1,:);
Y=C.Data(:,2,:);
Z=C.Data(:,3,:);

Courbe=[X Y Z];

%% affichage robot

ax = show(robot,deg2rad([0;90;-90;90]))
%% affichage robot + trajectoire

hold on
show(robot,deg2rad([0;90;-90;90]))
plot3(X,Y,Z,'r','LineWidth',2)
xlabel('x')
ylabel('y')
zlabel('z')
hold off