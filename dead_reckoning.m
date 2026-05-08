
waypoints = [
    52.0720, -0.6280;
    52.0732, -0.6245;
    52.0718, -0.6208;
    52.0695, -0.6185;
    52.0672, -0.6210;
    52.0660, -0.6250
];


time = 0;

% Speed in m/s
speed = 3.0;


[drlat, drlon, drtime] = dreckon(waypoints, time, speed);

% Add initial waypoint to the DR trajectory for plotting
plot_drlat = [waypoints(1,1); drlat];
plot_drlon = [waypoints(1,2); drlon];
plot_drtime = [time; drtime];

figure;
geoplot(waypoints(:,1), waypoints(:,2), '--o', 'LineWidth', 1.2);
hold on;
geoplot(plot_drlat, plot_drlon, '-x', 'LineWidth', 1.5);
geobasemap streets;
legend('Waypoint route', 'Dead-reckoned trajectory');
title('Waypoint Route and Dead-Reckoned Trajectory');