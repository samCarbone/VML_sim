

close all;
clear;
clc;

% ------------------------------- Setup -----------------------------------

att_ctrl_gains = [6; 6; 5; 3]; % Gains for the attitude/thrust controller in the model
initial_state = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % Initial state
% [x, y, z, vx, vy, vz, phi, theta, psi, T]
g = 9.81;

% Position controller
xy_pos_gain = [1; 1];
xy_vel_gain = [1; 1];

% Waypoints
s.pos = zeros(3, 1);
s.psi = 0;
wps = repmat(s,4,1);

wps(1).pos = [2; 0; -1];
wps(2).pos = [2; 2; -1];
wps(3).pos = [0; 2; -1];
wps(4).pos = [0; 0; -1];

wps(1).psi = pi/2;
wps(2).psi = pi;
wps(3).psi = 3*pi/2;
wps(4).psi = 0;

% Setup RANSAC queue

% Queue of uncorrected states and camera measurements
queue_t = []; % Time
queue_x_meas = []; % Cam measurement
queue_x_ucor = []; % Estimate (uncorrected)
queue_y_meas = []; % Cam measurement
queue_y_ucor = []; % Estimate (uncorrected)

% ---------------------------- Simulation ---------------------------------

% Run the simulink model
sim_time = 20; % s, sim duration
out = sim('VML_Sim');

% Extract results, get Euler angles too, and put things in the state vector
% t     = out.tout';
% quat  = out.state(:,1:4)';
% omega = out.state(:,5:7)';
% ctrl  = out.control';
% x = [quat; omega];

% Save the workspace
save('run_data.mat');

%% ---------------------------- Result ---------------------------------

x = out.state.signals.values(:, 1);
y = out.state.signals.values(:, 2);
z = out.state.signals.values(:, 3);

x_est = out.x_est.signals.values(:, 1);
y_est = out.y_est.signals.values(:, 1);
z_est = ones(size(x_est));


figure(1);
clf;
plot3(x, y, -z);
axis equal;

hold on;
plot3(x_est, y_est, z_est);



for i=1:length(wps)
    plot3(wps(i).pos(1), wps(i).pos(2), -wps(i).pos(3), 'o')
end