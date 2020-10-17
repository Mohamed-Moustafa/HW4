% ***************** Task(2) -- Joint trajectory (polynomial) ***************** %
% q(t) from q(0) = (0, 0, 0) to q(2) = (2, 3, 4) 
% null initial and final velocities and accelerations. (polynomial)
% **************************************************************************** %
clear


% 6 constrains so we have 5th order (quantitic)

%intial and final time
t0 = 0;
tf = 2;

% Initial joints angles
q0 = [0 0 0];
qf =[2 3 4];

% Initial velocities and accelerations
q0_dot = [0 0 0];
a0 = [0 0 0];
% Final velocities and accelerations
qf_dot = [0 0 0];
af = [0 0 0];

% Estanlishing our A matrix
A = [1 t0 t0^2 t0^3 t0^4 t0^5;
     0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
     0 0 2 6*t0 12*t0^2 20*t0^3;
     1 tf tf^2 tf^3 tf^4 tf^5;
     0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
     0 0 2 6*tf 12*tf^2 20*tf^3]
 
 
 %
 
 
 N=300;
 t = linspace(t0,tf,N);
 % of size 3 x 300
 joints_pos = zeros(3,N);
 joints_vel = zeros(3,N);
 joints_acc = zeros(3,N);

 for joint = 1:3
     
     % our B vector
     B = [q0(joint) q0_dot(joint) a0(joint) qf(joint) qf_dot(joint) af(joint)]';
 
     % Our X vector that holds poloynomial coffiecients
     X =A\B;
     
     
     % our position , velocity & accelration through time for the 3 joints
     joints_pos(joint,:) = X(1)+X(2).*t + X(3).*t.^2 + X(4).* t.^3+ X(5).*t.^4 + X(6).*t.^5;
     joints_vel(joint,:) = X(2) + 2*X(3).*t + 3*X(4).* t.^2+ 4*X(5).*t.^3 + 5*X(6).*t.^4;
     joints_acc(joint,:) = 2*X(3)+ X(4)*6.*t + X(5)*12.*t.^2 + X(6)* 20.*t.^3;
 end

 %% now plotting time
 
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


