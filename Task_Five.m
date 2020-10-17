P1=[1 0 1];
P2=[sqrt(2)/2 sqrt(2)/2 1.2];

% will return q1 q2 q3
Config1=RRR_IK(P1);
Config2=RRR_IK(P2);

% will return x y z
P1=FK(Config1(1));
Px1=P1(1);
Py1=P1(2);
Pz1=P1(3);

% will return x y z
P2=FK(Config1(1:2));
Px2=P2(1);
Py2=P2(2);
Pz2=P2(3);

% will return x y z
P3=FK(Config1(1:3));
Px3=P3(1);
Py3=P3(2);
Pz3=P3(3);

%link 1
plot3([-2 2],[0 0],[0 0],'color','black','LineStyle','--')
hold on
plot3([0 0],[-2 2],[0 0],'color','black','LineStyle','--')
plot3([0 0],[0 0],[-2 2],'color','black','LineStyle','--')

plot3([0 Px1],[0 Py1],[0 Pz1],'linewidth',3,'color','black')
plot3(Px1,Py1,Pz1,'r*','linewidth',3,'MarkerSize',7)
plot3([Px1 Px2],[Py1 Py2],[Pz1 Pz2],'linewidth',3,'color','black')
plot3(Px2,Py2,Pz2,'r*','linewidth',3,'MarkerSize',7)
plot3([Px2 Px3],[Py2 Py3],[Pz2 Pz3],'linewidth',3,'color','black')
plot3(Px3,Py3,Pz3,'r*','linewidth',3,'MarkerSize',7)

% will return x y z
P1=FK(Config2(1));
Px1=P1(1);
Py1=P1(2);
Pz1=P1(3);

% will return x y z
P2=FK(Config2(1:2));
Px2=P2(1);
Py2=P2(2);
Pz2=P2(3);

% will return x y z
P3=FK(Config2(1:3));
Px3=P3(1);
Py3=P3(2);
Pz3=P3(3);

plot3([0 Px1],[0 Py1],[0 Pz1],'linewidth',3,'color','b')
plot3(Px1,Py1,Pz1,'ro','linewidth',3,'MarkerSize',7)
plot3([Px1 Px2],[Py1 Py2],[Pz1 Pz2],'linewidth',3,'color','b')
plot3(Px2,Py2,Pz2,'ro','linewidth',3,'MarkerSize',7)
plot3([Px2 Px3],[Py2 Py3],[Pz2 Pz3],'linewidth',3,'color','b')
plot3(Px3,Py3,Pz3,'ro','linewidth',3,'MarkerSize',7)

f=zeros(3,length(t));
for p = 1:length(t)
fk= RRR_FK(joints_pos(:,p));
f(:,p) = fk(1:3)';
end
plot3(f(1,:),f(2,:),f(3,:),'g')
    