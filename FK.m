function P = FK(q)

L1 = 1;
L2 = 1;
L3 = 1;

if length(q)==1
q1=q(1);
H = Rz(q1) * Tz(L1);
elseif length(q)==2
q1=q(1);q2=q(2);
H = Rz(q1) * Tz(L1) * Ry(q2) * Tx(L2);
else
q1=q(1);q2=q(2);q3=q(3);

H = Rz(q1) * Tz(L1) * Ry(q2) * Tx(L2) * Ry(q3) * Tx(L3);
    
end

Px= H(1,4);
Py= H(2,4);
Pz= H(3,4);

P = [Px Py Pz];

end