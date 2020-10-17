% ***************** Task(4) -- Joint trajectory (Trapzoidal) LIN ***************** %
%  p1 = [1 0 1] to p2 = [sqrt(2)/2 sqrt(2)/2 1.2] Cartesian points
% Controller command interpretation frequency 
% f = 100 Hz
% Maximum linear velocity = 1 m/s
% Maximum linear acceleration = 10 m/s2
% ******************************************************************************** %


clear
P1 = [1 0 1];
P2 = [sqrt(2)/2 sqrt(2)/2 1.2];

tf = 20;

% No of points
N = 300;

% time
t = linspace(0,tf,N);

% Get the waypoints
x = ((P2(1)-P1(1))/tf).*t + P1(1);
y = ((P2(2)-P1(2))/tf).*t + P1(2);
z = ((P2(3)-P1(3))/tf).*t + P1(3);

% Get the velocities
vx = ((P2(1)-P1(1))/tf);
vy = ((P2(2)-P1(2))/tf);
vz = ((P2(3)-P1(3))/tf);

vel = [vx vy vz];


Joint_config = zeros(3,N);
Joint_vel = zeros(3,N);
for joint = 1:N
    point_vec = [x(joint) y(joint) z(joint)];
    Joint_config(:,joint) = RRR_IK(point_vec);
    if joint == 1 || joint == N
        Joint_vel(:,joint) = 0;
    else
        J = Jac_fn(Joint_config(:,joint));
        %inv(J)*vel
        Joint_vel(:,joint) = J(1:3,1:3)\vel';
    end
end

q_pos = zeros(3,25);
q_vel = zeros(3,25);
q_acc = zeros(3,25);
joints_pos = [];
joints_vel = [];
joints_acc = [];

for point = 1:N-1
    t0 = t(point);
    tf = t(point+1);
    A = [1 t0 t0^2 t0^3;
         0 1 2*t0 3*t0^2;
         1 tf tf^2 tf^3;
         0 1 2*tf 3*tf^2;];
     for joint = 1:3
         b = [Joint_config(joint,point) Joint_vel(joint,point) Joint_config(joint,point+1) Joint_vel(joint,point+1)]';
         %  inv(A) * b
         x =A\b;
         ti = linspace(t0,tf,25);
         q_pos(joint,:) = x(1)+x(2).*ti + x(3).*ti.^2 + x(4).* ti.^3;
         q_vel(joint,:) = x(2) + 2*x(3).*ti + 3*x(4).* ti.^2;
         q_acc(joint,:) = 2*x(3)+ x(4)*6.*ti;
     end	
     joints_pos = [joints_pos q_pos];
     joints_vel = [joints_vel q_vel];
     joints_acc = [joints_acc q_acc];
end
t = linspace(0,tf,length(joints_pos));

 %% now plotting time
figure;
subplot(3,3,1)
plot(t,joints_pos(1,:))
title('joint1 position')

subplot(3,3,2)
plot(t,joints_pos(2,:),'r')
title('joint2 position')

subplot(3,3,3)
plot(t,joints_pos(3,:),'m')
title('joint3 position')

subplot(3,3,4)
plot(t,joints_vel(1,:))
title('joint1 velocity')

subplot(3,3,5)
plot(t,joints_vel(2,:),'r')
title('joint2 velocity')

subplot(3,3,6)
plot(t,joints_vel(3,:),'m')
title('joint3 velocity')

subplot(3,3,7)
plot(t,joints_acc(1,:))
title('joint1 accelration')

subplot(3,3,8)
plot(t,joints_acc(2,:),'r')
title('joint2 accelration')
subplot(3,3,9)
plot(t,joints_acc(3,:),'m')
title('joint3 accelration')
hold off
figure;
plot(t,joints_vel(1,:),'b',t,joints_vel(2,:),'r',t,joints_vel(3,:),'m')
grid on
figure;
plot(t,joints_acc(1,:),'b',t,joints_acc(2,:),'r',t,joints_acc(3,:),'m')
grid on



