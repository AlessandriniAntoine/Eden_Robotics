%% record data
S = out;

%% save data
OpenLoop.time = S.Measure.time;

OpenLoop.Measure.x = S.Measure.signals.values(:,1);
OpenLoop.Measure.y = S.Measure.signals.values(:,2);
OpenLoop.Measure.z = S.Measure.signals.values(:,3);

OpenLoop.Reference.x = S.Reference.signals.values(:,1);
OpenLoop.Reference.y = S.Reference.signals.values(:,2);
OpenLoop.Reference.z = S.Reference.signals.values(:,3);

%%
figure()
subplot(2,2,1);
hold on
plot(OpenLoop.time,OpenLoop.Reference.x,'r');
plot(OpenLoop.time,OpenLoop.Measure.x,'b');
xlabel('time (s)')
ylabel('x (m)')
legend({'reference','measure'},'Location','northeast')
hold off

subplot(2,2,2);
hold on
plot(OpenLoop.time,OpenLoop.Reference.y,'r');
plot(OpenLoop.time,OpenLoop.Measure.y,'b');
xlabel('time (s)')
ylabel('y (m)')
hold off

subplot(2,2,[3,4]);
hold on
plot(OpenLoop.time,OpenLoop.Reference.z,'r');
plot(OpenLoop.time,OpenLoop.Measure.z,'b');
xlabel('time (s)')
ylabel('z (m)')
hold off

sgtitle('Open Loop on Base frame')

%% save data
ClosedLoop.time = S.Measure.time;

ClosedLoop.Measure.x = [S.Measure.signals.values(1,1)*ones(1,8) S.Measure.signals.values(1:end-8,1)']';
ClosedLoop.Measure.y = [S.Measure.signals.values(1,2)*ones(1,8) S.Measure.signals.values(1:end-8,2)']';
ClosedLoop.Measure.z = [S.Measure.signals.values(1,3)*ones(1,8) S.Measure.signals.values(1:end-8,3)']';

ClosedLoop.Reference.x = S.Reference.signals.values(:,1);
ClosedLoop.Reference.y = S.Reference.signals.values(:,2);
ClosedLoop.Reference.z = S.Reference.signals.values(:,3);

%%
figure()
subplot(3,1,1);
hold on
plot(ClosedLoop.time,ClosedLoop.Reference.x,'r');
plot(ClosedLoop.time,ClosedLoop.Measure.x,'b');
xlabel('time (s)')
ylabel('x (m)')
legend({'reference','measure'},'Location','northeast')
hold off

subplot(3,1,2);
hold on
plot(ClosedLoop.time,ClosedLoop.Reference.y,'r');
plot(ClosedLoop.time,ClosedLoop.Measure.y,'b');
xlabel('time (s)')
ylabel('y (m)')
hold off

subplot(3,1,3);
hold on
plot(ClosedLoop.time,ClosedLoop.Reference.z,'r');
plot(ClosedLoop.time,ClosedLoop.Measure.z,'b');
xlabel('time (s)')
ylabel('z (m)')
hold off

sgtitle('Closed Loop on Base frame')

%% save data
Xbox.time = S.Measure.time;

Xbox.Measure.x = S.Measure.signals.values(:,1);
Xbox.Measure.y = S.Measure.signals.values(:,2);
Xbox.Measure.z = S.Measure.signals.values(:,3);

Xbox.Reference.x = S.Reference.signals.values(:,1);
Xbox.Reference.y = S.Reference.signals.values(:,2);
Xbox.Reference.z = S.Reference.signals.values(:,3);

%%
figure()
subplot(3,1,1);
hold on
plot(Xbox.time,Xbox.Reference.x,'r');
plot(Xbox.time,Xbox.Measure.x,'b');
xlabel('time (s)')
ylabel('x (m)')
legend({'reference','measure'},'Location','northeast')
hold off

subplot(3,1,2);
hold on
plot(Xbox.time,Xbox.Reference.y,'r');
plot(Xbox.time,Xbox.Measure.y,'b');
xlabel('time (s)')
ylabel('y (m)')
hold off

subplot(3,1,3);
hold on
plot(Xbox.time,Xbox.Reference.z,'r');
plot(Xbox.time,Xbox.Measure.z,'b');
xlabel('time (s)')
ylabel('z (m)')
hold off

sgtitle('Xbox Controller trajectory on Base frame')
