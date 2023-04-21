%% Momentum Configuration Benchmark
% 2023-April-19
% unicycle driven by tractive force and steering torque

%state = [X Y th dX dY dth V]
% ATV weight = 500kg
% ATV max accel = 2m/s^2 -> max force = 1000N
% Izz = (500)*k^2 = 500*0.2^2 = 20 kgm^2

% Expected angular acceleration
% vmax = 10m/s
% rho_min = 2.74m % minimum turn radius
% over an 90 deg turn s = 2.74*pi/2 = 4.30 m
% delta_t = 4.30/5, close to 1
% ang_acc = pi/2 = 1.5 rad/sec^2
% max_torque = 20*1.5 = 30Nm

clear
close all

%% Vehicle parameters
mass = 500; % mass in kg
acc_max = 2; % max accel in m/s^2
Fmax = mass*acc_max;
k = 0.1; % radius of gyration guess
Izz = mass*k^2;
vmax = 10; % m/s
rho_min = 2.0; % minimum turn radius in m
time_to_90deg = (rho_min*pi/2)/vmax; % at vmax
ang_acc = 0.5*pi/time_to_90deg;
max_torque = Izz*ang_acc;


%% Generate halton samples
n = 9; % number of samples
dim = 2; 

xmin = -Fmax; xmax = Fmax; % force
ymin = -max_torque; ymax = max_torque; % torque

p = haltonset(dim);
p = net(p,n)


p(:,1) = xmin +(xmax-xmin)*p(:,1);
p(:,2) = ymin + (ymax-ymin)*p(:,2);

%figure()
%scatter(p(:,1),p(:,2),'filled','o');

%% Simulate vehicle motion with the provided samples
% this could be useful for debugging.

params.m = mass;
params.I = Izz;

%q = [X Y th dX dY dth vx]; 


q0 = [0 0 0 0 0 0.0 0];
tspan = [0:0.01:0.2];

figure()

for iter = 1:n
        
    u = [p(iter,1);p(iter,2)];
    [t,q] = ode45(@(t,q)myUnicycleNoSideSlip(t,q,u,params),tspan,q0);

    hold on
    plot(q(:,1),q(:,2))
    xlabel('X(m)')
    ylabel('Y(m)')
    %ylabel('fwd vel (m/s')

    axis equal
    grid on
end

q0 = [0 0 0 0 0 0.0 0];
tspan = [0:0.01:1.0];

figure()

        
    u = [p(4,1);p(4,2)];
    [t,q] = ode45(@(t,q)myUnicycleNoSideSlip(t,q,u,params),tspan,q0);

    hold on
    plot(q(:,1),q(:,2))
    xlabel('X(m)')
    ylabel('Y(m)')
    %ylabel('fwd vel (m/s')

    axis equal
    grid on





%scatter(p(:,1),p(:,2))


%% Parameters

runs = 1;

params = struct;
params.max_iterations = 100000;
params.max_generations = 100;
params.horizon_time = 0.3;
params.num_states = 7;
params.num_controls = 2;
params.grid_resolution = [0.01; 0.01; 5*pi/180;0;0;0;0.1];

params.start_state = [0; 0; 0; 0; 0; 0; 0.0];
params.goal_state =  [4; 4; 0; 0; 0; 0; 0.0]; 
params.branchout_factor = n;
params.branchouts = p';
% params.branchouts = [
%     [p(1,1); p(1,2)], ...
%     [p(2,1); p(2,2)], ...
%     [p(3,1); p(3,2)], ...
%     [p(4,1); p(4,2)], ...
%     [p(5,1); p(5,2)], ...
%     [p(6,1); p(6,2)], ...
%     [p(7,1); p(7,2)], ...
%     [p(8,1); p(8,2)], ...
%     [p(9,1); p(9,2)]
%     ];
%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
