
% Run after the simulink model has been run
% Do not clear the workspace
% Print the position of the vehicle
% Only run directly after a simulink run, not after another script

wps(1).pos = [2; 0; -1];
wps(2).pos = [2; 2; -1];
wps(3).pos = [0; 2; -1];
wps(4).pos = [0; 0; -1];

save('run_data.mat');

x = out.state.signals.values(:, 1);
y = out.state.signals.values(:, 2);
z = out.state.signals.values(:, 3);

figure(1);
clf;
plot3(x, y, -z);
axis equal;

hold on;
for i=1:length(wps)
    plot3(wps(i).pos(1), wps(i).pos(2), -wps(i).pos(3), 'o')
end