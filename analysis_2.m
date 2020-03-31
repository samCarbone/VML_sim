% Run after the simulink model has been run
% Loads the workspace saved by analysis.m

clear;
clc;

% Load the workspace saved in the analysis code
load('run_data.mat');

% Hard-coded waypoints
wps(1).pos = [2; 0; -1];
wps(2).pos = [2; 2; -1];
wps(3).pos = [0; 2; -1];
wps(4).pos = [0; 0; -1];

% Based upon the x, y, z truth data, generate camera measurements
% Based upon the angle data, generate position estimates
% Then use the RANSAC method on them

% Generate camera-based samples
cam_meas.values = [];
cam_meas.time = [];
last_t = -1;
max_dist = 1;
frame_time = 0.02; % 50Hz camera sample rate

for i=1:length(out.state.time)
    delta_t = out.state.time(i) - last_t;
    if out.dist_to_gate.signals.values(i) < max_dist && delta_t >= frame_time
        cam_meas.values(end+1, :) = out.state.signals.values(i, 1:3);
        cam_meas.time(end+1, 1) = out.state.time(i);
        last_t = out.state.time(i);
    end

end

% Generate imu-based samples
% [roll, pitch, yaw]
imu_meas.values = []; imu_meas.time = [];
last_t = -1;
frame_time = 0.002; % 500Hz imu sample rate

for i=1:length(out.state.time)
    delta_t = out.state.time(i) - last_t;
    if delta_t >= frame_time
        imu_meas.values(end+1, :) = out.state.signals.values(i, 7:9);
        imu_meas.time(end+1, 1) = out.state.time(i);
        last_t = out.state.time(i);
    end

end

%% Plot position, measurements and waypoints
figure(1);
clf;

% Plot actual position
x_act = out.state.signals.values(:, 1);
y_act = out.state.signals.values(:, 2);
z_act = out.state.signals.values(:, 3);
plot3(x_act, y_act, -z_act);
axis equal;

% Plot waypoints
hold on;
for i=1:length(wps)
    plot3(wps(i).pos(1), wps(i).pos(2), -wps(i).pos(3), 'o')
end

% Plot camera measurements
plot3(cam_meas.values(:, 1), cam_meas.values(:, 2), -cam_meas.values(:, 3), '*');


%% We now have imu data, and camera data, each with different timestamps

% For now, just do position/velocity estimation based upon imu angles
% Without RANSAC correction

% Array of states
x = [0;0]; % Initial state [x; v_x]
y = [0;0]; % Initial state [y; v_y]

A = [0, 1; 0, -0.5];
B = [0; 1]; % using 1 since the x-component of thrust is calculated directly


for k = 2:length(imu_meas.time)
    
    % Calc components of thrust in x and y (Earth) directions
    yaw = imu_meas.values(k-1, 3);
    pitch = imu_meas.values(k-1, 2);
    roll = imu_meas.values(k-1, 1);
    dcm_be = angle2dcm( yaw, pitch, roll ); % Earth to Body
    dcm_eb = dcm_be'; % Body to Earth

    F_t_unit = dcm_eb*[0; 0; -1]; % Thrust unit vector
    F_t = -9.81/F_t_unit(3) * F_t_unit; % Magnitude of thrust such that vertical component of accel equal to g
    
    % Estimate the state
    T_s = imu_meas.time(k) - imu_meas.time(k-1);
    x(:, k) = (eye(2) + A*T_s)*x(:, k-1) + eye(2)*T_s*B*F_t(1);
    y(:, k) = (eye(2) + A*T_s)*y(:, k-1) + eye(2)*T_s*B*F_t(2);
end

%% Plot the position estimate
plot3(x(1, :), y(1, :), ones(size(x(1, :))), 'r');





%% Now perform the algorithm, but with corrections from the images


