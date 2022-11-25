%% Init variables

% time
t_sim = 2; % s

% starting angles
Pelvis.p0 = 0; % rad
Shoulder.p0 = -1.5708; % rad
Elbow.p0 = 0; % rad
Wrist.p0 = 0; % rad

% ending angles
Pelvis.p1 = 0; % rad
Shoulder.p1 = -1.5708; % rad
Elbow.p1 = -1.5708; % rad
Wrist.p1 = 0; % rad

% slope 
Pelvis.slope = (Pelvis.p1-Pelvis.p0)/t_sim; % rad/s
Shoulder.slope = (Shoulder.p1-Shoulder.p0)/t_sim; % rad/s
Elbow.slope = (Elbow.p1-Elbow.p0)/t_sim; % rad/s
Wrist.slope = (Wrist.p1-Wrist.p0)/t_sim; % rad/s

%% Simulate
S = sim('motors_study.slx');

%% Pelvis value

% measures
Pelvis.C = S.Cp;
Pelvis.W = S.Wp;
Pelvis.P = S.Pp;

% maximum
[Pelvis.Pm,i] = max(Pelvis.P.signals.values(:));
Pelvis.Cm = Pelvis.C.signals.values(i);
Pelvis.Wm = Pelvis.W.signals.values(i);

% safety
Pelvis.Ps = Pelvis.Pm*1.3;
Pelvis.Cs = Pelvis.Cm*1.3;

% trajectory reference
Pelvis.trajectory.ref.x = S.ref.signals.values(:,1);
Pelvis.trajectory.ref.y = S.ref.signals.values(:,2);
Pelvis.trajectory.ref.z = S.ref.signals.values(:,3);

% trajectory measure
Pelvis.trajectory.measure.x = S.measure.signals.values(:,1);
Pelvis.trajectory.measure.y = S.measure.signals.values(:,2);
Pelvis.trajectory.measure.z = S.measure.signals.values(:,3);

%% Pelvis plot
figure()
subplot(2,2,[1,2]);
plot(Pelvis.P.time,Pelvis.P.signals.values(:),'r');
xlabel('time (s)')
ylabel('Power (W)')

subplot(2,2,3);
plot(Pelvis.C.time,Pelvis.C.signals.values(:),'r');
xlabel('time (s)')
ylabel('Torque (N)')

subplot(2,2,4);
plot(Pelvis.W.time,Pelvis.W.signals.values(:),'r');
xlabel('time (s)')
ylabel('Angular speed (rad/s)')
sgtitle('Pelvis measures')

% trajectory
figure
plot3(Pelvis.trajectory.ref.x,Pelvis.trajectory.ref.y,Pelvis.trajectory.ref.z,Pelvis.trajectory.measure.x,Pelvis.trajectory.measure.y,Pelvis.trajectory.measure.z)
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Trajectory')

figure
hold on; 
plot(Pelvis.trajectory.ref.x,Pelvis.trajectory.ref.y,'Color','r')
plot(Pelvis.trajectory.measure.x,Pelvis.trajectory.measure.y,'Color','b')
plot([-0.060,Pelvis.trajectory.ref.x(25)],[0.0,Pelvis.trajectory.ref.y(25)],'Color',[0 .5 .5],'LineWidth',2)
rectangle('Position',[-0.12,-0.06,0.24,0.12],'FaceColor',[0 .5 .5],'LineWidth',2)
xlabel('x (m)')
ylabel('y (m)')
title('trajectory xy plan')
legend({'reference','measure','arm'},'Location','northeast')
hold off; 

%% Shoulder value

% measures
Shoulder.C = S.Cs;
Shoulder.W = S.Ws;
Shoulder.P = S.Ps;

% maximum
[Shoulder.Pm,i] = max(Shoulder.P.signals.values(:));
Shoulder.Cm = Shoulder.C.signals.values(i);
Shoulder.Wm = Shoulder.W.signals.values(i);

% safety
Shoulder.Ps = Shoulder.Pm*1.3;
Shoulder.Cs = Shoulder.Cm*1.3;

% trajectory reference
Shoulder.trajectory.ref.x = S.ref.signals.values(:,1);
Shoulder.trajectory.ref.y = S.ref.signals.values(:,2);
Shoulder.trajectory.ref.z = S.ref.signals.values(:,3);

% trajectory measure
Shoulder.trajectory.measure.x = S.measure.signals.values(:,1);
Shoulder.trajectory.measure.y = S.measure.signals.values(:,2);
Shoulder.trajectory.measure.z = S.measure.signals.values(:,3);

%% Shoulder plot
figure
subplot(2,2,[1,2]);
plot(Shoulder.P.time,Shoulder.P.signals.values(:),'r');
xlabel('time (s)')
ylabel('Power (W)')

subplot(2,2,3);
plot(Shoulder.C.time,Shoulder.C.signals.values(:),'r');
xlabel('time (s)')
ylabel('Torque (N)')

subplot(2,2,4);
plot(Shoulder.W.time,Shoulder.W.signals.values(:),'r');
xlabel('time (s)')
ylabel('Angular speed (rad/s)')
sgtitle('Shoulder measures')

% trajectory 3d
figure
plot3(Shoulder.trajectory.ref.x,Shoulder.trajectory.ref.y,Shoulder.trajectory.ref.z,Shoulder.trajectory.measure.x,Shoulder.trajectory.measure.y,Shoulder.trajectory.measure.z)
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Trajectory')

% trajectory 2d
figure
hold on; 
plot(Shoulder.trajectory.ref.x,Shoulder.trajectory.ref.z,'Color','r')
plot(Shoulder.trajectory.measure.x,Shoulder.trajectory.measure.z,'Color','b')
plot([-0.060,-0.060,Shoulder.trajectory.ref.x(25)],[0.069,0.188,Shoulder.trajectory.ref.z(25)],'Color',[0 .5 .5],'LineWidth',2)
rectangle('Position',[-0.134,0,0.268,0.074],'FaceColor',[0 .5 .5],'LineWidth',2)
xlabel('x (m)')
ylabel('z (m)')
title('trajectory xz plan')
legend({'reference','measure','arm'},'Location','northwest')
hold off; 

%% Elbow measures

% measures
Elbow.C = S.Ce;
Elbow.W = S.We;
Elbow.P = S.Pe;

% maximum
[Elbow.Pm,i] = max(Elbow.P.signals.values(:));
Elbow.Cm = Elbow.C.signals.values(i);
Elbow.Wm = Elbow.W.signals.values(i);

% safety
Elbow.Ps = Elbow.Pm*1.3;
Elbow.Cs = Elbow.Cm*1.3;

% trajectory reference
Elbow.trajectory.ref.x = S.ref.signals.values(:,1);
Elbow.trajectory.ref.y = S.ref.signals.values(:,2);
Elbow.trajectory.ref.z = S.ref.signals.values(:,3);

% trajectory measure
Elbow.trajectory.measure.x = S.measure.signals.values(:,1);
Elbow.trajectory.measure.y = S.measure.signals.values(:,2);
Elbow.trajectory.measure.z = S.measure.signals.values(:,3);

%% Elbow plot 
figure
subplot(2,2,[1,2]);
plot(Elbow.P.time,Elbow.P.signals.values(:),'r');
xlabel('time (s)')
ylabel('Power (W)')

subplot(2,2,3);
plot(Elbow.C.time,Elbow.C.signals.values(:),'r');
xlabel('time (s)')
ylabel('Torque (N)')

subplot(2,2,4);
plot(Elbow.W.time,Elbow.W.signals.values(:),'r');
xlabel('time (s)')
ylabel('Angular speed (rad/s)')
sgtitle('Elbow measures')

% trajectory 3d
figure
plot3(Elbow.trajectory.ref.x,Elbow.trajectory.ref.y,Elbow.trajectory.ref.z,Elbow.trajectory.measure.x,Elbow.trajectory.measure.y,Elbow.trajectory.measure.z)
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Trajectory')

% trajectory 2d
figure
hold on; 
plot(Elbow.trajectory.ref.x,Elbow.trajectory.ref.z,'Color','r')
plot(Elbow.trajectory.measure.x,Elbow.trajectory.measure.z,'Color','b')
plot([-0.060,-0.060,-0.220,Elbow.trajectory.ref.x(25)],[0.069,0.188,0.188,Elbow.trajectory.ref.z(25)],'Color',[0 .5 .5],'LineWidth',2)
rectangle('Position',[-0.134,0,0.268,0.074],'FaceColor',[0 .5 .5],'LineWidth',2)
xlabel('x (m)')
ylabel('z (m)')
title('trajectory xz plan')
legend({'reference','measure','arm'},'Location','northwest')
hold off; 

%% Wrist measures

% measures
Wrist.C = S.Cw;
Wrist.W = S.Ww;
Wrist.P = S.Pw;

% maximum
[Wrist.Pm,i] = max(Wrist.P.signals.values(:));
Wrist.Cm = Wrist.C.signals.values(i);
Wrist.Wm = Wrist.W.signals.values(i);

% safety
Wrist.Ps = Wrist.Pm*1.3;
Wrist.Cs = Wrist.Cm*1.3;

% trajectory reference
Wrist.trajectory.ref.x = S.ref.signals.values(:,1);
Wrist.trajectory.ref.y = S.ref.signals.values(:,2);
Wrist.trajectory.ref.z = S.ref.signals.values(:,3);

% trajectory measure
Wrist.trajectory.measure.x = S.measure.signals.values(:,1);
Wrist.trajectory.measure.y = S.measure.signals.values(:,2);
Wrist.trajectory.measure.z = S.measure.signals.values(:,3);

%% Wrist plot
figure
subplot(2,2,[1,2]);
plot(Wrist.P.time,Wrist.P.signals.values(:),'r');
xlabel('time (s)')
ylabel('Power (W)')

subplot(2,2,3);
plot(Wrist.C.time,Wrist.C.signals.values(:),'r');
xlabel('time (s)')
ylabel('Torque (N)')

subplot(2,2,4);
plot(Wrist.W.time,Wrist.W.signals.values(:),'r');
xlabel('time (s)')
ylabel('Angular speed (rad/s)')
sgtitle('Wrist measures')

% trajectory 3d
figure
plot3(Wrist.trajectory.ref.x,Wrist.trajectory.ref.y,Wrist.trajectory.ref.z,Wrist.trajectory.measure.x,Wrist.trajectory.measure.y,Wrist.trajectory.measure.z)
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Trajectory')

% trajectory 2d
figure
hold on; 
plot(Wrist.trajectory.ref.x,Wrist.trajectory.ref.z,'Color','r')
plot(Wrist.trajectory.measure.x,Wrist.trajectory.measure.z,'Color','b')
plot([-0.060,-0.060,Wrist.trajectory.ref.x(25)],[0.069,0.188,Wrist.trajectory.ref.z(25)],'Color',[0 .5 .5],'LineWidth',2)
rectangle('Position',[-0.134,0,0.268,0.074],'FaceColor',[0 .5 .5],'LineWidth',2)
xlabel('x (m)')
ylabel('z (m)')
title('trajectory xz plan')
legend({'reference','measure','arm'},'Location','northwest')
hold off; 