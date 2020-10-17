% ***************** Task(3) -- Joint trajectory (Trapzoidal) PTP ***************** %
% q(t) from q(0) = (0, 0, 0) to q(2) = (2, 3, 4)
% Controller command interpretation frequency 
% f = 100 Hz
% Maximum joint velocity = 1 rad/s
% Maximum joint acceleration = 10rad/s2
% ******************************************************************************** %

clear
% Initial joints angles
q0 = [0 0 0];

% Final joint angles
qf = [2 3 4];

% Max q_dot
q_dot_max = [1 1 1];

% Max acc
a_max = [10 10 10];

% t1
t1 = q_dot_max(1)/a_max(1) ;

%delta_q
delta_q = (qf(:)-q0(:));

% tf 
tf = ((abs(delta_q(:))./q_dot_max(1))+t1);

%control frquency = 100 hz
delta_t = 1/100;

% we will pick max tf and synchronize all for it

tf_sync = max(tf) ;
v_sync = (delta_q(:))/(tf_sync - t1);
a_sync = v_sync/t1;

% initate a vector to but paramters in it
B = zeros(3,8);
for joint = 1:3
    % t0 --> t1:
    a12 = 0.5*a_sync(joint);
    a11 = 0;
    a10 = q0(joint);
    
    
    
    
    % t1 --> taw:
    a20 = q0(joint) + 0.5*a_sync(joint).*t1.^2 - v_sync(joint).*t1;
    a21 = v_sync(joint);
    
    % taw --> tf:
    a32 = -0.5*a_sync(joint);
    a31 = a_sync(joint).*tf_sync;
    a30 = qf(joint) - 0.5*a_sync(joint).*tf_sync.^2;
    
    
    
    % but all our Coffiecnts in the cofficient vector B
    B(joint,:) = [a10; a11; a12; a20; a21; a30; a31; a32];
end


%last

t = (0:delta_t:tf_sync);
joints_pos = zeros(3,length(t));
joints_vel = zeros(3,length(t));
joints_acc = zeros(3,length(t));

for joint = 1:3
    joints_pos(joint,:) = (B(joint,1) +B(joint,2).*t + B(joint,3).*t.^2).*(t<=t1)...
            +(B(joint,4) +B(joint,5).*t ).*(t1 < t).*(t <= (tf_sync-t1))...
            +(B(joint,6) +B(joint,7).*t + B(joint,8).*t.^2).*...
            ((tf_sync-t1)<t).*(t <= tf_sync);
    joints_vel(joint,:) = (B(joint,2)+ 2*B(joint,3).*t).*(t<=t1)...
        + B(joint,5).*(t1 < t).*(t <= (tf_sync-t1))...
        +(B(joint,7)+ 2*B(joint,8).*t).*((tf_sync-t1)<t).*(t <= tf_sync);
    joints_acc(joint,:) = 2*B(joint,3).*(t<=t1)...
        + 0.*(t1 < t).*(t <= (tf_sync-t1)) ...
        + 2*B(joint,8).*((tf_sync-t1)<t).*(t <= tf_sync);
end

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
