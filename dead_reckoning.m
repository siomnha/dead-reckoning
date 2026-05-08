%% 3-D time-based dead reckoning for a UAV trajectory
% This example replaces 2-D waypoint-to-waypoint dead reckoning with a
% timestamped 3-D trajectory.  The trajectory is sampled at a fixed real-time
% update rate, velocity measurements are integrated in the local ENU frame, and
% the integrated positions are converted back to latitude/longitude/altitude for
% reporting and plotting.

clear; clc;

%% Timestamped UAV trajectory: [time_s, latitude_deg, longitude_deg, altitude_m]
trajectory = [
     0, 52.0720, -0.6280,  80;
    45, 52.0732, -0.6245, 110;
    95, 52.0718, -0.6208, 135;
   140, 52.0695, -0.6185, 120;
   190, 52.0672, -0.6210,  95;
   245, 52.0660, -0.6250,  85
];

% Dead-reckoning update interval.  In an onboard implementation this would be
% the control-loop or sensor-fusion cycle time.
sampleTime = 1.0;     % seconds

% Simple velocity-sensor model in the local ENU frame.  Keep both at zero for
% ideal dead reckoning, or increase them to see drift accumulate over time.
velocityBiasENU = [0.02, -0.01, 0.005];  % [east north up] m/s
velocityNoiseStd = 0.03;                 % m/s, set to 0 for deterministic output
rng(7);                                  % repeatable noise for the demo

%% Build the time-based reference trajectory and run real-time DR integration
[reference, origin] = makeReferenceTrajectory(trajectory, sampleTime);
measurements = makeVelocityMeasurements(reference.velocityENU, velocityBiasENU, velocityNoiseStd);
deadReckoned = integrateDeadReckoning(reference.time, reference.positionENU(1,:), measurements, origin);

%% Print the time-stamped DR output
results = table( ...
    deadReckoned.time(:), ...
    deadReckoned.latitude(:), ...
    deadReckoned.longitude(:), ...
    deadReckoned.altitude(:), ...
    deadReckoned.positionENU(:,1), ...
    deadReckoned.positionENU(:,2), ...
    deadReckoned.positionENU(:,3), ...
    'VariableNames', {'time_s','latitude_deg','longitude_deg','altitude_m','east_m','north_m','up_m'});
disp(results(1:min(10,height(results)),:));

%% Plot 3-D trajectory in local ENU coordinates
figure('Name', '3-D UAV Dead Reckoning');
plot3(reference.positionENU(:,1), reference.positionENU(:,2), reference.positionENU(:,3), ...
    '--', 'LineWidth', 1.2, 'DisplayName', 'Time trajectory');
hold on;
plot3(deadReckoned.positionENU(:,1), deadReckoned.positionENU(:,2), deadReckoned.positionENU(:,3), ...
    '-', 'LineWidth', 1.5, 'DisplayName', 'Dead-reckoned trajectory');
scatter3(reference.waypointENU(:,1), reference.waypointENU(:,2), reference.waypointENU(:,3), ...
    45, 'filled', 'DisplayName', 'Timed waypoints');
grid on; axis equal;
xlabel('East (m)'); ylabel('North (m)'); zlabel('Up / altitude change (m)');
title('3-D Time-Based UAV Dead Reckoning');
legend('Location', 'best');
view(38, 24);

%% Optional 2-D map view with altitude encoded by marker colour
figure('Name', 'Map View with Altitude');
geoplot(trajectory(:,2), trajectory(:,3), '--o', 'LineWidth', 1.1, 'DisplayName', 'Timed waypoints');
hold on;
geoscatter(deadReckoned.latitude, deadReckoned.longitude, 12, deadReckoned.altitude, 'filled', ...
    'DisplayName', 'Dead-reckoned samples');
geobasemap streets;
cb = colorbar;
cb.Label.String = 'Altitude (m)';
title('Dead-Reckoned UAV Trajectory Samples');
legend('Location', 'best');

function [reference, origin] = makeReferenceTrajectory(trajectory, sampleTime)
%MAKEREFERENCETRAJECTORY Interpolate a timed geodetic trajectory in ENU.
    validateTrajectory(trajectory, sampleTime);

    origin.latitude = trajectory(1,2);
    origin.longitude = trajectory(1,3);
    origin.altitude = trajectory(1,4);

    waypointENU = geodeticToENU(trajectory(:,2), trajectory(:,3), trajectory(:,4), origin);
    time = (trajectory(1,1):sampleTime:trajectory(end,1)).';
    if time(end) < trajectory(end,1)
        time(end+1,1) = trajectory(end,1); %#ok<AGROW>
    end

    positionENU = interp1(trajectory(:,1), waypointENU, time, 'linear');
    velocityENU = zeros(size(positionENU));

    for k = 1:(numel(time)-1)
        dt = time(k+1) - time(k);
        velocityENU(k,:) = (positionENU(k+1,:) - positionENU(k,:)) ./ dt;
    end
    velocityENU(end,:) = velocityENU(end-1,:);

    reference.time = time;
    reference.positionENU = positionENU;
    reference.velocityENU = velocityENU;
    reference.waypointENU = waypointENU;
end

function measurements = makeVelocityMeasurements(trueVelocityENU, biasENU, noiseStd)
%MAKEVELOCITYMEASUREMENTS Apply a simple bias/noise model to ENU velocities.
    noise = noiseStd .* randn(size(trueVelocityENU));
    measurements = trueVelocityENU + biasENU + noise;
end

function deadReckoned = integrateDeadReckoning(time, initialPositionENU, velocityENU, origin)
%INTEGRATEDEADRECKONING Integrate ENU velocity samples over time.
    positionENU = zeros(numel(time), 3);
    positionENU(1,:) = initialPositionENU;

    for k = 2:numel(time)
        dt = time(k) - time(k-1);
        positionENU(k,:) = positionENU(k-1,:) + velocityENU(k-1,:) .* dt;
    end

    [latitude, longitude, altitude] = enuToGeodetic(positionENU, origin);

    deadReckoned.time = time;
    deadReckoned.positionENU = positionENU;
    deadReckoned.latitude = latitude;
    deadReckoned.longitude = longitude;
    deadReckoned.altitude = altitude;
end

function enu = geodeticToENU(latitude, longitude, altitude, origin)
%GEODETICTOENU Convert geodetic coordinates to a local tangent ENU frame.
% This spherical-earth conversion is appropriate for short UAV routes.  Use
% MATLAB's geodetic2enu with a reference ellipsoid if survey-grade accuracy is
% needed over larger operating areas.
    earthRadiusM = 6378137;
    lat0 = deg2rad(origin.latitude);
    lon0 = deg2rad(origin.longitude);

    north = earthRadiusM .* (deg2rad(latitude) - lat0);
    east = earthRadiusM .* cos(lat0) .* (deg2rad(longitude) - lon0);
    up = altitude - origin.altitude;

    enu = [east, north, up];
end

function [latitude, longitude, altitude] = enuToGeodetic(enu, origin)
%ENUTOGEODETIC Convert local ENU positions back to latitude/longitude/altitude.
    earthRadiusM = 6378137;
    lat0 = deg2rad(origin.latitude);
    lon0 = deg2rad(origin.longitude);

    latitude = rad2deg(lat0 + enu(:,2) ./ earthRadiusM);
    longitude = rad2deg(lon0 + enu(:,1) ./ (earthRadiusM .* cos(lat0)));
    altitude = origin.altitude + enu(:,3);
end

function validateTrajectory(trajectory, sampleTime)
%VALIDATETRAJECTORY Validate time, position, and altitude inputs.
    if size(trajectory,2) ~= 4
        error('Trajectory must be [time_s, latitude_deg, longitude_deg, altitude_m].');
    end
    if any(diff(trajectory(:,1)) <= 0)
        error('Trajectory timestamps must be strictly increasing.');
    end
    if sampleTime <= 0
        error('sampleTime must be positive.');
    end
end
