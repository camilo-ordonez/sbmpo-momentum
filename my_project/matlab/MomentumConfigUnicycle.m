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
k = 0.2; % radius of gyration guess
Izz = mass*k^2;
vmax = 10; % m/s
rho_min = 2.74; % minimum turn radius in m
time_to_90deg = (rho_min*pi/2)/(vmax/3); % at 1/3 of vmax
ang_acc = 0.5*pi/time_to_90deg;
max_torque = Izz*ang_acc;



keyboard()



%% Generate halton samples
n = 9; % number of samples
dim = 2; 

xmin = -1000; xmax = 1000; % force
ymin = -30; ymax = 30; % torque

p = haltonset(dim);
p = net(p,n)


p(:,1) = xmin +(xmax-xmin)*p(:,1);
p(:,2) = ymin + (ymax-ymin)*p(:,2);


%% Simulate vehicle motion with the provided samples
% this could be useful for debugging.




%scatter(p(:,1),p(:,2))

keyboard()

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 100000;
params.max_generations = 100;
params.horizon_time = 0.1;
params.num_states = 7;
params.num_controls = 2;
params.grid_resolution = [0.1; 0.1; 5*pi/180;0;0;0;0.1];

params.start_state = [1; 1; 0; 0; 0; 0; 0.0];
params.goal_state =  [4; 4; 0; 0; 0; 0; 0.0]; 
params.branchout_factor = 9;
params.branchouts = [
    [p(1,1); p(1,2)], ...
    [p(2,1); p(2,2)], ...
    [p(3,1); p(3,2)], ...
    [p(4,1); p(4,2)], ...
    [p(5,1); p(5,2)], ...
    [p(6,1); p(6,2)], ...
    [p(7,1); p(7,2)], ...
    [p(8,1); p(8,2)], ...
    [p(9,1); p(9,2)], ...
    [p(10,1); p(10,2)], ...
    ];
%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
