function q = RRR_IK(P)

L1 = 1;
L2 = 1;
L3 = 1;
Px=P(1);Py=P(2);Pz=P(3);
%H = Rz(q1) * Tz(L1) * Ry(q2) * Tx(L2) * Ry(q3) * Tx(L3);
q1=atan2(Py,Px);
if q1>= pi
    q1=q1-pi;
end
s=sqrt(Px^2+Py^2);
d=L1-Pz;
% D= sqrt(s^2+d^2);
% epsi=atan2(Pz-L1,s);
% phi= acos ((L1^2+D^2 - L2^2) / 2*L1*D);
% 
% q2= epsi+phi;
% 
% phi2= acos ((L1^2+L2^2 - D^2) / 2*L1*L2);
% q3= pi-phi2;

q3 = acos( ( s^2 + d^2 - L2^2 - L3^2)/ (2 * L2 * L3) );

q2 = - atan2( L3 * sin(q3), (L2 + L3 * cos(q3))) + atan2(d,s);



q = [q1 q2 q3];

end