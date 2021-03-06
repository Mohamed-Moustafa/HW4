% ***************** Task(1) -- Jacobian ***************** %

%% 1-forward kinematics
syms q1 q2 q3 real
L1 = 1;
L2 = 1;
L3 = 1;
H = Rz(q1) * Tz(L1) * Ry(q2) * Tx(L2) * Ry(q3) * Tx(L3);
H = simplify(H);

%% 2-jacobian by numerical method
R = H(1:3,1:3);

% First joint
J1r = Rzd(q1) * Tz(L1) * Ry(q2) * Tx(L2) * Ry(q3) * Tx(L3) * [R' zeros(3,1);0 0 0 1];
J1r = simplify(J1r);
J1 =  [ J1r(1,4) J1r(2,4) J1r(3,4) J1r(3,2) J1r(1,3) J1r(2,1)].';

% Second joint
J2r = Rz(q1) * Tz(L1) * Ryd(q2) * Tx(L2) * Ry(q3) * Tx(L3) * [R' zeros(3,1);0 0 0 1];
J2r = simplify(J2r);
J2 =  [ J2r(1,4) J2r(2,4) J2r(3,4) J2r(3,2) J2r(1,3) J2r(2,1)].';

% Third joint
J3r = Rz(q1) * Tz(L1) * Ry(q2) * Tx(L2) * Ryd(q3) * Tx(L3) * [R' zeros(3,1);0 0 0 1];
J3r = simplify(J3r);
J3 =  [ J3r(1,4) J3r(2,4) J3r(3,4) J3r(3,2) J3r(1,3) J3r(2,1)].';

% Full jacobian matrix
Jacobian = [J1 J2 J3]

