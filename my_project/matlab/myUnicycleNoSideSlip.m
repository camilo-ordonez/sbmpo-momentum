function dq = myUnicycleNoSideSlip(t,q,u,params);

%q = [X Y th dX dY dth vx]; 
%dq = [dX dY dth d2X d2Y d2th dvx];

Fx = u(1);
tau = u(2);

m = params.m;
I = params.I;
b = 0;
bz = 0;

X = q(1); Y = q(2); th = q(3);
dX = q(4); dY = q(5); dth = q(6);
vx = q(7);

M = [m 0 0;0 0 1; 0 I 0];


B = [Fx - b*vx;dth*vx;tau-bz*dth];
tmp = inv(M)*B;

dvx = tmp(1);
d2th = tmp(2);
Fy = tmp(3); % constraint force
vy = 0; % no side slip
dvy = 0;

ax = dvx - dth*vy;
ay = dvy + dth*vx;

d2X = ax*cos(th) - ay*sin(th);
d2Y = ax*sin(th) + ay*cos(th);

%Fload = 2.0*Fx;
%dvx = (Fx-Fload)/m;
%d2th = Tau/I;

%Acc_x = dvx*cos(th) - dth*vx*sin(th); % global X acceleration
%Acc_y = dvx*sin(th) + dth*vx*cos(th); % global Y acceleration

%th = atan2(sin(th),cos(th))

%d2X = (dvx*cos(th) - dth*vx*sin(th));
%d2Y = (dvx*sin(th) + dth*vx*cos(th));



dq1 = dX; dq2 = dY; dq3 = dth;
dq4 = d2X; dq5 = d2Y; dq6 = d2th;
dq7 = dvx;


dq = [dq1;dq2;dq3;dq4;dq5;dq6;dq7];

