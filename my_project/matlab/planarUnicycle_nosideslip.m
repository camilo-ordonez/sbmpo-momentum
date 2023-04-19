% planar vehicle dynamic model. 
% Unicycle with zero side slip velocity. 

clear all
close all
clc

%params = getparams();

params.m = 10;
params.I = 10;

m = params.m;
I = params.I;

%q = [X Y th dX dY dth vx]; 

%q = [X Y th dX dY dth vx vy]; 


q0 = [0 0 0 1 0 0.2 1];
tspan = [0:0.01:30];

[t,q] = ode45(@(t,q)myUnicycleNoSideSlip(t,q,params),tspan,q0);

figure(1)

    plot(q(:,1),q(:,2))
    xlabel('time (s)')
    ylabel('fwd vel (m/s')

    axis equal
    grid on

figure(2)

    plot(t,q(:,7))
    xlabel('time (s)')
    ylabel('fwd vel (m/s')

    axis equal
    grid on





