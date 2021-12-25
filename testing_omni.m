
l1=0.07;l2=0.07;l3=0.07;
theta=0;

A=[cos(theta),sin(theta),0;-sin(theta),cos(theta),0;0,0,1];
B=[0,-1,-l1;cos(pi/4),sin(pi/4),-l2;-sin(pi/4),cos(pi/4),-l3];

B*A*[0;1;0.549]