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
pos_actual_at_imu_time = []; % The truth measurements, but aligned with the imu samples

for i=1:length(out.state.time)
    delta_t = out.state.time(i) - last_t;
    if delta_t >= frame_time
        imu_meas.values(end+1, :) = out.state.signals.values(i, 7:9);
        imu_meas.time(end+1, 1) = out.state.time(i);
        last_t = out.state.time(i);
        pos_actual_at_imu_time(end+1, :) = out.state.signals.values(i, 1:3);
    end

end


%% We now have imu data, and camera data, each with different timestamps

% For now, just do position/velocity estimation based upon imu angles
% Without RANSAC correction

% Array of states without correction
x = [0;0]; % Initial state [x; v_x]
y = [0;0]; % Initial state [y; v_y]

% Corrections
x_off = [0;0];
y_off = [0;0];

x_off_array = [0;0];
y_off_array = [0;0];

% State estimate with correction
x_est = [0;0];
y_est = [0;0];

% Queue of uncorrected states and camera measurements
queue.t = []; % Time
queue.x_meas = []; % Cam measurement
queue.x_ucor = []; % Estimate (uncorrected)
queue.y_meas = []; % Cam measurement
queue.y_ucor = []; % Estimate (uncorrected)

% Dynamic model
A = [0, 1; 0, -0.5];
B = [0; 1]; % using 1 since the x-component of thrust is calculated directly

% Next cam measurement index
cam_index = 1;

% The time window the linear regression will be calculated over
T_window = 2;
N_fit = 10; % Minimum number of samples for the regression
t_reg_0 = 0; % The time 0 for the last regression

for k = 2:length(imu_meas.time)
    
    % Calc components of thrust in x and y (Earth) directions
    yaw = imu_meas.values(k-1, 3);
    pitch = imu_meas.values(k-1, 2);
    roll = imu_meas.values(k-1, 1);
    dcm_be = angle2dcm( yaw, pitch, roll ); % Earth to Body
    dcm_eb = dcm_be'; % Body to Earth

    F_t_unit = dcm_eb*[0; 0; -1]; % Thrust unit vector
    F_t = -9.81/F_t_unit(3) * F_t_unit; % Magnitude of thrust such that vertical component of accel equal to g
    
    % Estimate the state (without correction)
    T_s = imu_meas.time(k) - imu_meas.time(k-1);
    x(:, k) = (eye(2) + A*T_s)*x(:, k-1) + eye(2)*T_s*B*F_t(1);
    y(:, k) = (eye(2) + A*T_s)*y(:, k-1) + eye(2)*T_s*B*F_t(2);
    
    % If a new camera measurement is avaiable
    % --> in this case, we wait for the imu measurement at or immediately
    % after the cam measurement
    % In this case, we are not taking into account a delay in the
    % measurement
    
    % First check if the camera array is long enough
    if length(cam_meas.time) >= cam_index
        if cam_meas.time(cam_index) <= imu_meas.time(k)    
            
            % Clear old elements in the queue
            for i = 1:length(queue.t)
                if queue.t(i) < imu_meas.time(k) - T_window
                    % Remove the element
                    queue.t(i) = [];
                    queue.x_meas(i) = [];
                    queue.x_ucor(i) = [];
                    queue.y_meas(i) = [];
                    queue.y_ucor(i) = [];
                else
                    break;
                end
            end
            
            % Add the new camera measurement to the queue
            queue.t(end+1) = imu_meas.time(k);
            queue.x_meas(end+1) = cam_meas.values(cam_index, 1);
            queue.x_ucor(end+1) = x(1, k);
            queue.y_meas(end+1) = cam_meas.values(cam_index, 2);
            queue.y_ucor(end+1) = y(1, k);

            % Set the element for the next camera measurement
            cam_index = cam_index + 1;
            
            % Check if the queue has sufficient elements
            if length(queue.t) >= N_fit
                
                [x_off, y_off] = filter_correct(queue);
                t_reg_0 = queue.t(end);
                
            end
            
        end
        
    end
    
    % Append the corrections to the array
    x_off_array(:, k) = x_off;
    y_off_array(:, k) = y_off;
    
    % State estimate with correction
    del_t = imu_meas.time(k) - t_reg_0;
    x_est(:, k) = x(:, k) + [1, del_t; 0, 1]*x_off; 
    y_est(:, k) = y(:, k) + [1, del_t; 0, 1]*y_off; 
    
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


%% Plot the position estimatea
plot3(x(1, :), y(1, :), ones(size(x(1, :))), 'r'); % uncorrected
plot3(x_est(1, :), y_est(1, :), ones(size(x(1, :))), 'k'); % corrected

figure(2);
clf;
hold on;
plot(out.state.time, x_act);
plot(imu_meas.time, x(1, :));
plot(imu_meas.time, x_off_array(1, :));
plot(imu_meas.time, x_off_array(2, :));
plot(imu_meas.time, x_est(1, :));

legend('x actual', 'x uncorr', 'x offset', 'v offset', 'x est');

figure(3);
clf;
hold on;
plot(imu_meas.time, x(1, :) - pos_actual_at_imu_time(:, 1)');
plot(imu_meas.time, x_est(1, :) - pos_actual_at_imu_time(:, 1)');

legend('x err uncorrected', 'x err corrected');

%% Now perform the algorithm, but with corrections from the images


%%
% Calculate the correction to the states
function [x_off, y_off] = filter_correct(queue)

    Dt = zeros(length(queue.t), 1);
    Dx = zeros(length(queue.t), 1);
    Dy = zeros(length(queue.t), 1);

    for i = 1:length(queue.t)
        
        Dt(i) = queue.t(i) - queue.t(end);
        Dx(i) = queue.x_meas(i) - queue.x_ucor(i);
        Dy(i) = queue.y_meas(i) - queue.y_ucor(i);
        
    end

    iter = 10;
    sigma_th = 0.5;
    n = round(0.8*length(queue.t));
    P = [0, 0; 0, 0.3];
    [rx_off, vx_off] = prior_ransac(Dx, Dt, iter, sigma_th, n, P);
    [ry_off, vy_off] = prior_ransac(Dy, Dt, iter, sigma_th, n, P);

    x_off = [rx_off; vx_off];
    y_off = [ry_off; vy_off];
    
end
