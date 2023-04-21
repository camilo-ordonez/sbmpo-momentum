function dq = myUnicycleKinematics(t,q,u,params);


v = u(1);
omega = u(2);


X = q(1); Y = q(2); th = q(3);
dX = v*cos(th); dY = v*sin(th); dth = omega;


dq = [dX;dY;dth];

