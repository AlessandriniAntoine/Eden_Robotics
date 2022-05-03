%% Get trajectory

% % rotation_bassin_max_bas = sim('AA_Assemblage');
% % rotation_bassin_min_bas = sim('AA_Assemblage');
% % rotation_bassin_max_haut = sim('AA_Assemblage');
% rotation_bassin_min_haut = sim('AA_Assemblage');
% % rotation_epaule_max_droite = sim('AA_Assemblage');
% % rotation_epaule_max_gauche = sim('AA_Assemblage');
% % rotation_epaule_min_droite = sim('AA_Assemblage');
% % rotation_epaule_min_gauche = sim('AA_Assemblage');
% % rotation1_min = sim('AA_Assemblage');
% % rotation2_min = sim('AA_Assemblage');
% % rotation3_min = sim('AA_Assemblage');
% % rotation4_min = sim('AA_Assemblage');
% % rotation5_min = sim('AA_Assemblage');
% % rotation6_min = sim('AA_Assemblage');
% % rotation1e_min = sim('AA_Assemblage');
% % rotation2e_min = sim('AA_Assemblage');
% % rotation3e_min = sim('AA_Assemblage');
% % rotation4e_min = sim('AA_Assemblage');
% % rotation5e_min = sim('AA_Assemblage');
% % rotation6e_min = sim('AA_Assemblage');
% % rotation1_max = sim('AA_Assemblage');
% % rotation2_max = sim('AA_Assemblage');
% % rotation3_max = sim('AA_Assemblage');
% % rotation4_max = sim('AA_Assemblage');
% % rotation5_max = sim('AA_Assemblage');
% % rotation6_max = sim('AA_Assemblage');
% % rotation1e_max = sim('AA_Assemblage');
% % rotation2e_max = sim('AA_Assemblage');
% % rotation3e_max = sim('AA_Assemblage');
% % rotation4e_max = sim('AA_Assemblage');
% % rotation5e_max = sim('AA_Assemblage');
% % rotation6e_max = sim('AA_Assemblage');
% % rotation1b_max = sim('AA_Assemblage');

%% x,y,z

% bassin max bas
x_max_bas = rotation_bassin_max_bas.Position_robot.Data(:,1);
y_max_bas = rotation_bassin_max_bas.Position_robot.Data(:,2);
z_max_bas = rotation_bassin_max_bas.Position_robot.Data(:,3);

% % bassin min bas 
x_min_bas = rotation_bassin_min_bas.Position_robot.Data(:,1);
y_min_bas = rotation_bassin_min_bas.Position_robot.Data(:,2);
z_min_bas = rotation_bassin_min_bas.Position_robot.Data(:,3);

% bassin max haut
x_max_haut = rotation_bassin_max_haut.Position_robot.Data(:,1);
y_max_haut = rotation_bassin_max_haut.Position_robot.Data(:,2);
z_max_haut = rotation_bassin_max_haut.Position_robot.Data(:,3);

% % bassin min haut
% x_min_haut = rotation_bassin_min_haut.Position_robot.Data(:,1);
% y_min_haut = rotation_bassin_min_haut.Position_robot.Data(:,2);
% z_min_haut = rotation_bassin_min_haut.Position_robot.Data(:,3);
 
% epaule max droite
xe_max_droite = rotation_epaule_max_droite.Position_robot.Data(:,1);
ye_max_droite = rotation_epaule_max_droite.Position_robot.Data(:,2);
ze_max_droite = rotation_epaule_max_droite.Position_robot.Data(:,3);
 
% % epaule max gauche
xe_max_gauche = rotation_epaule_max_gauche.Position_robot.Data(:,1);
ye_max_gauche = rotation_epaule_max_gauche.Position_robot.Data(:,2);
ze_max_gauche = rotation_epaule_max_gauche.Position_robot.Data(:,3);

% epaule min droite
xe_min_droite = rotation_epaule_min_droite.Position_robot.Data(:,1);
ye_min_droite = rotation_epaule_min_droite.Position_robot.Data(:,2);
ze_min_droite = rotation_epaule_min_droite.Position_robot.Data(:,3);

% epaule min gauche
xe_min_gauche = rotation_epaule_min_gauche.Position_robot.Data(:,1);
ye_min_gauche = rotation_epaule_min_gauche.Position_robot.Data(:,2);
ze_min_gauche = rotation_epaule_min_gauche.Position_robot.Data(:,3);

% arc min 
% 1
% x1_min = rotation1_min.Position_robot.Data(:,1);
% y1_min = rotation1_min.Position_robot.Data(:,2);
% z1_min = rotation1_min.Position_robot.Data(:,3);

% 2
x2_min = rotation2_min.Position_robot.Data(:,1);
y2_min = rotation2_min.Position_robot.Data(:,2);
z2_min = rotation2_min.Position_robot.Data(:,3);

% 3
x3_min = rotation3_min.Position_robot.Data(:,1);
y3_min = rotation3_min.Position_robot.Data(:,2);
z3_min = rotation3_min.Position_robot.Data(:,3);

% 4
x4_min = rotation4_min.Position_robot.Data(:,1);
y4_min = rotation4_min.Position_robot.Data(:,2);
z4_min = rotation4_min.Position_robot.Data(:,3);

% 5
x5_min = rotation5_min.Position_robot.Data(:,1);
y5_min = rotation5_min.Position_robot.Data(:,2);
z5_min = rotation5_min.Position_robot.Data(:,3);

% 6
x6_min = rotation6_min.Position_robot.Data(:,1);
y6_min = rotation6_min.Position_robot.Data(:,2);
z6_min = rotation6_min.Position_robot.Data(:,3);

% arc min epaule
% 1
xe1_min = rotation1e_min.Position_robot.Data(:,1);
ye1_min = rotation1e_min.Position_robot.Data(:,2);
ze1_min = rotation1e_min.Position_robot.Data(:,3);

% 2
xe2_min = rotation2e_min.Position_robot.Data(:,1);
ye2_min = rotation2e_min.Position_robot.Data(:,2);
ze2_min = rotation2e_min.Position_robot.Data(:,3);

% 3
xe3_min = rotation3e_min.Position_robot.Data(:,1);
ye3_min = rotation3e_min.Position_robot.Data(:,2);
ze3_min = rotation3e_min.Position_robot.Data(:,3);

% 4
xe4_min = rotation4e_min.Position_robot.Data(:,1);
ye4_min = rotation4e_min.Position_robot.Data(:,2);
ze4_min = rotation4e_min.Position_robot.Data(:,3);

% 5
xe5_min = rotation5e_min.Position_robot.Data(:,1);
ye5_min = rotation5e_min.Position_robot.Data(:,2);
ze5_min = rotation5e_min.Position_robot.Data(:,3);

% 6
xe6_min = rotation6e_min.Position_robot.Data(:,1);
ye6_min = rotation6e_min.Position_robot.Data(:,2);
ze6_min = rotation6e_min.Position_robot.Data(:,3);

% arc max 
% 1
x1b_max = rotation1b_max.Position_robot.Data(:,1);
y1b_max = rotation1b_max.Position_robot.Data(:,2);
z1b_max = rotation1b_max.Position_robot.Data(:,3);

% % 2
x2_max = rotation2_max.Position_robot.Data(:,1);
y2_max = rotation2_max.Position_robot.Data(:,2);
z2_max = rotation2_max.Position_robot.Data(:,3);

% arc max 
% 1
x1_max = rotation1_max.Position_robot.Data(:,1);
y1_max = rotation1_max.Position_robot.Data(:,2);
z1_max = rotation1_max.Position_robot.Data(:,3);

% 2
x2_max = rotation2_max.Position_robot.Data(:,1);
y2_max = rotation2_max.Position_robot.Data(:,2);
z2_max = rotation2_max.Position_robot.Data(:,3);

% 3
x3_max = rotation3_max.Position_robot.Data(:,1);
y3_max = rotation3_max.Position_robot.Data(:,2);
z3_max = rotation3_max.Position_robot.Data(:,3);

% 4
x4_max = rotation4_max.Position_robot.Data(:,1);
y4_max = rotation4_max.Position_robot.Data(:,2);
z4_max = rotation4_max.Position_robot.Data(:,3);

% 5
x5_max = rotation5_max.Position_robot.Data(:,1);
y5_max = rotation5_max.Position_robot.Data(:,2);
z5_max = rotation5_max.Position_robot.Data(:,3);

% 6
x6_max = rotation6_max.Position_robot.Data(:,1);
y6_max = rotation6_max.Position_robot.Data(:,2);
z6_max = rotation6_max.Position_robot.Data(:,3);

% arc min epaule
% 1
xe1_max = rotation1e_max.Position_robot.Data(:,1);
ye1_max = rotation1e_max.Position_robot.Data(:,2);
ze1_max = rotation1e_max.Position_robot.Data(:,3);

% 2
xe2_max = rotation2e_max.Position_robot.Data(:,1);
ye2_max = rotation2e_max.Position_robot.Data(:,2);
ze2_max = rotation2e_max.Position_robot.Data(:,3);

% 3
xe3_max = rotation3e_max.Position_robot.Data(:,1);
ye3_max = rotation3e_max.Position_robot.Data(:,2);
ze3_max = rotation3e_max.Position_robot.Data(:,3);

% 4
xe4_max = rotation4e_max.Position_robot.Data(:,1);
ye4_max = rotation4e_max.Position_robot.Data(:,2);
ze4_max = rotation4e_max.Position_robot.Data(:,3);

% 5
xe5_max = rotation5e_max.Position_robot.Data(:,1);
ye5_max = rotation5e_max.Position_robot.Data(:,2);
ze5_max = rotation5e_max.Position_robot.Data(:,3);

% 6
xe6_max= rotation6e_max.Position_robot.Data(:,1);
ye6_max = rotation6e_max.Position_robot.Data(:,2);
ze6_max = rotation6e_max.Position_robot.Data(:,3);
%% Plotting space

ax = show(robot,[0 ;0 ; 0;0])

% plot
hold on
%bassin
plot3(x_max_bas,y_max_bas,z_max_bas,'color','blue','linewidth',0.5)
plot3(x1b_max,y1b_max,z1b_max,'color','blue','linewidth',0.5)
plot3(x_min_bas,y_min_bas,z_min_bas,'color','red','linewidth',3)
plot3(x_max_haut,y_max_haut,z_max_haut,'color','blue','linewidth',0.5)
%epaule extrem
plot3(xe_max_droite,ye_max_droite,ze_max_droite,'color','blue','linewidth',0.5)
plot3(xe_max_gauche,ye_max_gauche,ze_max_gauche,'color','blue','linewidth',0.5)
plot3(xe_min_droite,ye_min_droite,ze_min_droite,'color','red','linewidth',3)
plot3(xe_min_gauche,ye_min_gauche,ze_min_gauche,'color','red','linewidth',3)
%droite
plot3([0.1388 0.1013],[0.2263 0.1457],[-0.1555 -0.0341],'color','blue','linewidth',0.5)
plot3([-0.1002 -0.0232],[-0.0328 0.0117],[-0.1555 -0.0341],'color','blue','linewidth',0.5)
%tour min
plot3(x1_min,y1_min,z1_min,'color','red','linewidth',3)
plot3(x2_min,y2_min,z2_min,'color','red','linewidth',3)
plot3(x3_min,y3_min,z3_min,'color','red','linewidth',3)
plot3(x4_min,y4_min,z4_min,'color','red','linewidth',3)
plot3(x5_min,y5_min,z5_min,'color','red','linewidth',3)
plot3(x6_min,y6_min,z6_min,'color','red','linewidth',3)
plot3(xe1_min,ye1_min,ze1_min,'color','red','linewidth',3)
plot3(xe2_min,ye2_min,ze2_min,'color','red','linewidth',3)
plot3(xe3_min,ye3_min,ze3_min,'color','red','linewidth',3)
plot3(xe4_min,ye4_min,ze4_min,'color','red','linewidth',3)
plot3(xe5_min,ye5_min,ze5_min,'color','red','linewidth',3)
plot3(xe6_min,ye6_min,ze6_min,'color','red','linewidth',3)
%tour max
plot3(x1_max,y1_max,z1_max,'color','blue','linewidth',0.5)
plot3(x2_max,y2_max,z2_max,'color','blue','linewidth',0.5)
plot3(x3_max,y3_max,z3_max,'color','blue','linewidth',0.5)
plot3(x4_max,y4_max,z4_max,'color','blue','linewidth',0.5)
plot3(x5_max,y5_max,z5_max,'color','blue','linewidth',0.5)
plot3(x6_max,y6_max,z6_max,'color','blue','linewidth',0.5)
plot3(xe1_max,ye1_max,ze1_max,'color','blue','linewidth',0.5)
plot3(xe2_max,ye2_max,ze2_max,'color','blue','linewidth',0.5)
plot3(xe3_max,ye3_max,ze3_max,'color','blue','linewidth',0.5)
plot3(xe4_max,ye4_max,ze4_max,'color','blue','linewidth',0.5)
plot3(xe5_max,ye5_max,ze5_max,'color','blue','linewidth',0.5)
plot3(xe6_max,ye6_max,ze6_max,'color','blue','linewidth',0.5)
hold off

