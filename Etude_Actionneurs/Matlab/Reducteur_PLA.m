%% Bassin
n = 0.98;      % rendement du réducteur
Cs = 5;         % N.m couple sortie du réducteur
% Cm = r*Cs/n ;  % N.m couple moteur
Cm = 0.5;           % N.m couple moteur
r= n*Cm/Cs;           % rapport de réduction
we = 20;         % rpm  vitesse moteur
ws = we*r ;         % rpm  vitesse sortie reducteur

Ze = 10;         % roues menantes
Zs = Ze/r;     % roues menées

fprintf('Moteur bassin \n')
fprintf(1,'  Rapport de réduction  :  %G  \n ',1/r);
fprintf(1,' Couple moteur :  %G N.m \n ',Cm);
fprintf(1, ' Couple sortie :   %G  N.m\n ',Cs);
fprintf(1,' Vitesse rotation moteur  :  %G  rpm\n ',we);
fprintf(1, ' vitesse rotation sortie  :  %G  rpm\n ',ws);
fprintf(1,' Roues menantes   :  %G  \n ',Ze);
fprintf(1, ' Roues menées :  %G  \n\n ',Zs);
%% Epaule
n = 0.98;      % rendement du réducteur
Cs = 9.88;         % N.m couple sortie du réducteur
% Cm = r*Cs/n ;  % N.m couple moteur
Cm = 1.75;           % N.m couple moteur
r= n*Cm/Cs;           % rapport de réduction
we = 24;         % rpm  vitesse moteur
ws = we*r ;         % rpm  vitesse sortie reducteur

Ze = 10;         % roues menantes
Zs = Ze/r;     % roues menées

fprintf('Moteur épaule \n')
fprintf(1,'  Rapport de réduction  :  %G  \n ',1/r);
fprintf(1,' Couple moteur :  %G N.m \n ',Cm);
fprintf(1, ' Couple sortie :   %G  N.m\n ',Cs);
fprintf(1,' Vitesse rotation moteur  :  %G  rpm\n ',we);
fprintf(1, ' vitesse rotation sortie  :  %G  rpm\n ',ws);
fprintf(1,' Roues menantes   :  %G  \n ',Ze);
fprintf(1, ' Roues menées :  %G  \n\n ',Zs);
%% Coude
n = 0.98;      % rendement du réducteur
Cs = 2.34;         % N.m couple sortie du réducteur
% Cm = r*Cs/n ;  % N.m couple moteur
Cm = 0.5;           % N.m couple moteur
r= n*Cm/Cs;           % rapport de réduction
we = 52;         % rpm  vitesse moteur
ws = we*r ;         % rpm  vitesse sortie reducteur

Ze = 10;         % roues menantes
Zs = Ze/r;     % roues menées

fprintf('Moteur coude \n')
fprintf(1,'  Rapport de réduction  :  %G  \n ',1/r);
fprintf(1,' Couple moteur :  %G N.m \n ',Cm);
fprintf(1, ' Couple sortie :   %G  N.m\n ',Cs);
fprintf(1,' Vitesse rotation moteur  :  %G  rpm\n ',we);
fprintf(1, ' vitesse rotation sortie  :  %G  rpm\n ',ws);
fprintf(1,' Roues menantes   :  %G  \n ',Ze);
fprintf(1, ' Roues menées :  %G  \n\n ',Zs);
%% Poignet
n = 0.98;      % rendement du réducteur
Cs = 1.3;         % N.m couple sortie du réducteur
% Cm = r*Cs/n ;  % N.m couple moteur
Cm = 0.39;           % N.m couple moteur
r= n*Cm/Cs;           % rapport de réduction
we = 81;         % rpm  vitesse moteur
ws = we*r ;         % rpm  vitesse sortie reducteur

Ze = 10;         % roues menantes
Zs = Ze/r;     % roues menées

fprintf('Moteur poignet \n')
fprintf(1,'  Rapport de réduction  :  %G  \n ',1/r);
fprintf(1,' Couple moteur :  %G N.m \n ',Cm);
fprintf(1, ' Couple sortie :   %G  N.m\n ',Cs);
fprintf(1,' Vitesse rotation moteur  :  %G  rpm\n ',we);
fprintf(1, ' vitesse rotation sortie  :  %G  rpm\n ',ws);
fprintf(1,' Roues menantes   :  %G  \n ',Ze);
fprintf(1, ' Roues menées :  %G  \n\n ',Zs);