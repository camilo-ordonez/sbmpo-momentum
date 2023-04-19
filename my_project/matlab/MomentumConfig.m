%% Momentum Configuration Benchmark
% 2023-04-08

clear
close all

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 100000;
params.max_generations = 100;
params.horizon_time = 0.1;
params.num_states = 2;
params.num_controls = 1;
params.grid_resolution = [0.005; 0.0005];
%params.grid_resolution = [0.02; 0.4];
%params.grid_resolution = [0.01; 0.01];

params.start_state = [-1.1; 0.0];
params.goal_state = [0.5; 0]; 
params.branchout_factor = 7;
params.branchouts = linspace(-1,1,params.branchout_factor);


%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
