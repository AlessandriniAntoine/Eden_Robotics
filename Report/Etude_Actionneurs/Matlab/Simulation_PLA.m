%% Dynamique
%% Simulation_1
%Temps de simulation :
tsim= 6;   % s

%Point de départ
x0= -0.369;  
y0= -0.006639;
z0= 0.244;

%Point d'arrivée
x1= -0.193;
y1= 0.02451;
z1= 0.5511;

%%% Définition des pentes
a=(x1-x0)/tsim;
b=(y1-y0)/tsim;
c=(z1-z0)/tsim;

if a>0
    xratep=a;
    xraten=0;
else 
    xratep=0;
    xraten=a;
end

if b>0
    yratep=b;
    yraten=0;
else 
    yratep=0;
    yraten=b;
end

if c>0
    zratep=c;
    zraten=0;
else 
    zratep=0;
    zraten=c;
end
%% Simulation_2
%Temps de simulation :
tsim= 6;   % s

%Point de départ
x0= -0.2303;
y0= -0.1796;
z0=0.2431;

%Point d'arrivée
x1= -0.2479;
y1= -0.08928;
z1= 0.533;

%%% Définition des pentes
a=(x1-x0)/tsim;
b=(y1-y0)/tsim;
c=(z1-z0)/tsim;

if a>0
    xratep=a;
    xraten=0;
else 
    xratep=0;
    xraten=a;
end

if b>0
    yratep=b;
    yraten=0;
else 
    yratep=0;
    yraten=b;
end

if c>0
    zratep=c;
    zraten=0;
else 
    zratep=0;
    zraten=c;
end

%% Simulation_3
%Temps de simulation :
tsim= 8;   % s

%Point de départ
x0= -0.3228;
y0= -0.006639;
z0=0.2991;

%Point d'arrivée
x1= -0.000678;
y1= 0.3105;
z1= 0.3329;

%%% Définition des pentes
a=(x1-x0)/tsim;
b=(y1-y0)/tsim;
c=(z1-z0)/tsim;

if a>0
    xratep=a;
    xraten=0;
else 
    xratep=0;
    xraten=a;
end

if b>0
    yratep=b;
    yraten=0;
else 
    yratep=0;
    yraten=b;
end

if c>0
    zratep=c;
    zraten=0;
else 
    zratep=0;
    zraten=c;
end

%% Simulation_4
%Temps de simulation :
tsim= 5;   % s

%Point de départ
x0= -0.2709;
y0= -0.006639;
z0=0.1831;

%Point d'arrivée
x1= -0.3071;
y1= -0.00639;
z1=0.4357;

%%% Définition des pentes
a=(x1-x0)/tsim;
b=(y1-y0)/tsim;
c=(z1-z0)/tsim;

if a>0
    xratep=a;
    xraten=0;
else 
    xratep=0;
    xraten=a;
end

if b>0
    yratep=b;
    yraten=0;
else 
    yratep=0;
    yraten=b;
end

if c>0
    zratep=c;
    zraten=0;
else 
    zratep=0;
    zraten=c;
end
%% Simulation_5
%Temps de simulation :
tsim= 10;   % s

%Point de départ
x0= 0.2601;
y0= 0.05904;
z0= 0.4002;

%Point d'arrivée
x1= -0.03772;
y1= 0.1877;
z1= 0.4474;

%%% Définition des pentes
a=(x1-x0)/tsim;
b=(y1-y0)/tsim;
c=(z1-z0)/tsim;

if a>0
    xratep=a;
    xraten=0;
else 
    xratep=0;
    xraten=a;
end

if b>0
    yratep=b;
    yraten=0;
else 
    yratep=0;
    yraten=b;
end

if c>0
    zratep=c;
    zraten=0;
else 
    zratep=0;
    zraten=c;
end
%% Statique
%% Simulation_1
%Temps de simulation :
tsim= 0.1;   % s

%Point de départ
x0= -0.3929;
y0= -0.006639;
z0= 0.25;

%Point d'arrivée
x1= -0.3929;
y1= -0.006639;
z1= 0.25;

%%% Définition des pentes
a=(x1-x0)/tsim;
b=(y1-y0)/tsim;
c=(z1-z0)/tsim;

if a>0
    xratep=a;
    xraten=0;
else 
    xratep=0;
    xraten=a;
end

if b>0
    yratep=b;
    yraten=0;
else 
    yratep=0;
    yraten=b;
end

if c>0
    zratep=c;
    zraten=0;
else 
    zratep=0;
    zraten=c;
end