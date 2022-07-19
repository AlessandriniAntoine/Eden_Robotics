%Penser à lancer Initialisation avant Etude
%% Simulation et récupération des données
open_system('Moteurs_PLA.slx')
S = sim('Moteurs_PLA');smiData.Solid(25).mass = 0.010505847400000001;

%Moteur bassin
Cb=S.get('Cb');      % N.m
Vb=S.get('Vb');      % tour/min
Pb=S.get('Pb');      % W

%Moteur épaule
Cm=S.get('Cm');      % N.m
Vm=S.get('Vm');      % tour/min
Pm=S.get('Pm');      % W

%Moteur coude
Ch=S.get('Ch');      % N.m
Vh=S.get('Vh');      % tour/min
Ph=S.get('Ph');      % W

%Moteur Poignet
Cp=S.get('Cp');      % N.m
Vp=S.get('Vp');      % tour/min
Pp=S.get('Pp');      % W

%% Affichage graphique

%Moteur bassin
figure('NumberTitle', 'off', 'Name', 'Bassin');
% Top two plots
tiledlayout(2,2)
nexttile
plot(Vb)
xlabel('temps (s)')
ylabel('Vitesse (rpm)')
title('Vitesse moteur bassin')
nexttile
plot(Pb)
xlabel('temps (s)')
ylabel('Puissance (W)')
title('Puissance moteur bassin')
% Plot that spans
nexttile([1 2])
plot(Cb)
xlabel('temps (s)')
ylabel('Couple (N.m)')
title('Couple moteur bassin')

%Moteur epaule
figure('NumberTitle', 'off', 'Name', 'Epaule');
% Top two plots
tiledlayout(2,2)
nexttile
plot(Vm)
xlabel('temps (s)')
ylabel('Vitesse (rpm)')
title('Vitesse moteur epaule')
nexttile
plot(Pm)
xlabel('temps (s)')
ylabel('Puissance (W)')
title('Puissance moteur epaule')
% Plot that spans
nexttile([1 2])
plot(Cm)
xlabel('temps (s)')
ylabel('Couple (N.m)')
title('Couple moteur epaule')

%Moteur coude
figure('NumberTitle', 'off', 'Name', 'Coude');
% Top two plots
tiledlayout(2,2)
nexttile
plot(Vh)
xlabel('temps (s)')
ylabel('Vitesse (rpm)')
title('Vitesse moteur coude')
nexttile
plot(Ph)
xlabel('temps (s)')
ylabel('Puissance (W)')
title('Puissance moteur coude')
% Plot that spans
nexttile([1 2])
plot(Ch)
xlabel('temps (s)')
ylabel('Couple (N.m)')
title('Couple moteur coude')

%Moteur poignet
figure('NumberTitle', 'off', 'Name', 'Poignet');
% Top two plots
tiledlayout(2,2)
nexttile
plot(Vp)
xlabel('temps (s)')
ylabel('Vitesse (rpm)')
title('Vitesse moteur poignet')
nexttile
plot(Pp)
xlabel('temps (s)')
ylabel('Puissance (W)')
title('Puissance moteur poignet')
% Plot that spans
nexttile([1 2])
plot(Cp)
xlabel('temps (s)')
ylabel('Couple (N.m)')
title('Couple moteur poignet')
%% Affichage des extrèmes

%moteur bas
fprintf('Moteur bassin \n')
fprintf(1,'\n Vitesse maximale  :  %G  rpm\n ',max(Vb.Data));
fprintf(1,'Vitesse minimale :  %G  rpm\n\n ',min(Vb.Data));

fprintf(1,'Couple maximale :  %G  N.m\n ',max(Cb.Data));
fprintf(1,'Couple minimale :  %G  N.m\n\n ',min(Cb.Data));

fprintf(1,'Puissance maximale :  %G  W\n ',max(Pb.Data));
fprintf(1,'Puissance minimale :  %G  W\n\n ',min(Pb.Data));

%moteur milieu
fprintf('Moteur épaule \n')
fprintf(1,'\n Vitesse maximale  :  %G  rpm\n ',max(Vm.Data));
fprintf(1,'Vitesse minimale :  %G  rpm\n\n ',min(Vm.Data));

fprintf(1,'Couple maximale :  %G  N.m\n ',max(Cm.Data));
fprintf(1,'Couple minimale :  %G  N.m\n\n ',min(Cm.Data));

fprintf(1,'Puissance maximale :  %G  W\n ',max(Pm.Data));
fprintf(1,'Puissance minimale :  %G  W\n\n ',min(Pm.Data));

%moteur haut
fprintf('Moteur coude \n')
fprintf(1,'\n Vitesse maximale  :  %G  rpm\n ',max(Vh.Data));
fprintf(1,'Vitesse minimale :  %G  rpm\n\n ',min(Vh.Data));

fprintf(1,'Couple maximale :  %G  N.m\n ',max(Ch.Data));
fprintf(1,'Couple minimale :  %G  N.m\n\n ',min(Ch.Data));

fprintf(1,'Puissance maximale :  %G  W\n ',max(Ph.Data));
fprintf(1,'Puissance minimale :  %G  W\n\n ',min(Ph.Data));

%moteur poignet
fprintf('Moteur poignet \n')
fprintf(1,'\n Vitesse maximale  :  %G  rpm\n ',max(Vp.Data));
fprintf(1,'Vitesse minimale :  %G  rpm\n\n ',min(Vp.Data));

fprintf(1,'Couple maximale :  %G  N.m\n ',max(Cp.Data));
fprintf(1,'Couple minimale :  %G  N.m\n\n ',min(Cp.Data));

fprintf(1,'Puissance maximale :  %G  W\n ',max(Pp.Data));
fprintf(1,'Puissance minimale :  %G  W\n\n ',min(Pp.Data));