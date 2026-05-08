%% 3-D curved, time-stepped dead reckoning for a UAV trajectory
% This example replaces 2-D straight waypoint segments with a timestamped 3-D
% UAV trajectory.  The route between timed waypoints is a smooth cubic Hermite
% curve, and the dead-reckoning estimate advances one sampling interval at a
% time to mimic an onboard real-time update loop.

clear; clc;

%% Timestamped UAV trajectory control points: [time_s, latitude_deg, longitude_deg, altitude_m]
trajectory = [
     0, 52.0720, -0.6280,  80;
    45, 52.0732, -0.6245, 110;
    95, 52.0718, -0.6208, 135;
   140, 52.0695, -0.6185, 120;
   190, 52.0672, -0.6210,  95;
   245, 52.0660, -0.6250,  85
];

% Dead-reckoning update interval.  In an onboard implementation this is the
% control-loop or sensor-fusion cycle time.
sampleTime = 1.0;       % seconds
playbackInRealTime = false;  % set true to pause sampleTime seconds inside the simulation loop
animationSpeedup = 12;       % 1 = real time, 12 = play the 245 s route in about 20 s
showStaticPlots = true;      % keep summary plots after the animated flight

% Simple velocity-sensor model in the local ENU frame.  Keep both at zero for
% ideal dead reckoning, or increase them to see drift accumulate over time.
velocityBiasENU = [0.02, -0.01, 0.005];  % [east north up] m/s
velocityNoiseStd = 0.03;                 % m/s, set to 0 for deterministic output
rng(7);                                  % repeatable noise for the demo

%% Create the curved trajectory model and fly it one sample at a time
[trajectoryModel, origin] = createCurvedTrajectoryModel(trajectory, sampleTime);
flightLog = flyDeadReckoningLoop(trajectoryModel, sampleTime, playbackInRealTime, ...
    velocityBiasENU, velocityNoiseStd, origin);

%% Print the first time-stamped DR samples
results = table( ...
    flightLog.time(:), ...
    flightLog.latitude(:), ...
    flightLog.longitude(:), ...
    flightLog.altitude(:), ...
    flightLog.deadReckonedENU(:,1), ...
    flightLog.deadReckonedENU(:,2), ...
    flightLog.deadReckonedENU(:,3), ...
    'VariableNames', {'time_s','latitude_deg','longitude_deg','altitude_m','east_m','north_m','up_m'});
disp(results(1:min(10,height(results)),:));

%% Animate the UAV flying the trajectory over time
animateFlightTrajectory(flightLog, trajectoryModel, animationSpeedup);

%% Optional static summary plots after the animation
if showStaticPlots
    figure('Name', '3-D Curved UAV Dead Reckoning');
    plot3(flightLog.referenceENU(:,1), flightLog.referenceENU(:,2), flightLog.referenceENU(:,3), ...
        '--', 'LineWidth', 1.4, 'DisplayName', 'Curved reference trajectory');
    hold on;
    plot3(flightLog.deadReckonedENU(:,1), flightLog.deadReckonedENU(:,2), flightLog.deadReckonedENU(:,3), ...
        '-', 'LineWidth', 1.5, 'DisplayName', 'Dead-reckoned trajectory');
    scatter3(trajectoryModel.waypointENU(:,1), trajectoryModel.waypointENU(:,2), trajectoryModel.waypointENU(:,3), ...
        45, 'filled', 'DisplayName', 'Timed control points');
    grid on; axis equal;
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up / altitude change (m)');
    title('3-D Curved Time-Stepped UAV Dead Reckoning');
    legend('Location', 'best');
    view(38, 24);

    figure('Name', 'Map View with Altitude');
    geoplot(trajectory(:,2), trajectory(:,3), 'o', 'LineWidth', 1.1, 'DisplayName', 'Timed control points');
    hold on;
    geoscatter(flightLog.referenceLatitude, flightLog.referenceLongitude, 10, flightLog.referenceAltitude, ...
        'filled', 'DisplayName', 'Sampled curved reference');
    geoscatter(flightLog.latitude, flightLog.longitude, 12, flightLog.altitude, 'filled', ...
        'DisplayName', 'Dead-reckoned samples');
    geobasemap streets;
    cb = colorbar;
    cb.Label.String = 'Altitude (m)';
    title('Curved UAV Trajectory Samples');
    legend('Location', 'best');
end


function animateFlightTrajectory(flightLog, model, animationSpeedup)
%ANIMATEFLIGHTTRAJECTORY Replay the UAV trajectory sample by sample.
    if animationSpeedup <= 0
        error('animationSpeedup must be positive.');
    end

    allPositions = [flightLog.referenceENU; flightLog.deadReckonedENU; model.waypointENU];
    padding = max(10, 0.08 .* max(max(allPositions, [], 1) - min(allPositions, [], 1)));
    minLimits = min(allPositions, [], 1) - padding;
    maxLimits = max(allPositions, [], 1) + padding;

    figure('Name', 'Animated UAV Dead Reckoning');
    axesHandle = axes;
    hold(axesHandle, 'on');
    grid(axesHandle, 'on');
    axis(axesHandle, 'equal');
    xlim(axesHandle, [minLimits(1), maxLimits(1)]);
    ylim(axesHandle, [minLimits(2), maxLimits(2)]);
    zlim(axesHandle, [minLimits(3), maxLimits(3)]);
    xlabel(axesHandle, 'East (m)');
    ylabel(axesHandle, 'North (m)');
    zlabel(axesHandle, 'Up / altitude change (m)');
    view(axesHandle, 38, 24);

    plot3(axesHandle, model.waypointENU(:,1), model.waypointENU(:,2), model.waypointENU(:,3), ...
        'ko', 'MarkerFaceColor', [0.2 0.2 0.2], 'DisplayName', 'Timed control points');

    referenceTrail = animatedline(axesHandle, 'LineStyle', '--', 'LineWidth', 1.4, ...
        'Color', [0.1 0.45 0.9], 'DisplayName', 'Reference flown so far');
    deadReckonedTrail = animatedline(axesHandle, 'LineStyle', '-', 'LineWidth', 1.7, ...
        'Color', [0.9 0.25 0.1], 'DisplayName', 'Dead-reckoned flown so far');

    referenceAircraft = plot3(axesHandle, NaN, NaN, NaN, '^', 'MarkerSize', 9, ...
        'MarkerFaceColor', [0.1 0.45 0.9], 'MarkerEdgeColor', 'k', 'DisplayName', 'Reference UAV');
    deadReckonedAircraft = plot3(axesHandle, NaN, NaN, NaN, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.9 0.25 0.1], 'MarkerEdgeColor', 'k', 'DisplayName', 'DR estimate');
    velocityVector = quiver3(axesHandle, NaN, NaN, NaN, NaN, NaN, NaN, 0, ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 1.1, 'DisplayName', 'Measured velocity');

    legend(axesHandle, 'Location', 'best');

    for k = 1:numel(flightLog.time)
        referencePosition = flightLog.referenceENU(k,:);
        deadReckonedPosition = flightLog.deadReckonedENU(k,:);
        measuredVelocity = flightLog.measuredVelocityENU(k,:);

        addpoints(referenceTrail, referencePosition(1), referencePosition(2), referencePosition(3));
        addpoints(deadReckonedTrail, deadReckonedPosition(1), deadReckonedPosition(2), deadReckonedPosition(3));

        set(referenceAircraft, 'XData', referencePosition(1), 'YData', referencePosition(2), 'ZData', referencePosition(3));
        set(deadReckonedAircraft, 'XData', deadReckonedPosition(1), 'YData', deadReckonedPosition(2), 'ZData', deadReckonedPosition(3));
        set(velocityVector, ...
            'XData', deadReckonedPosition(1), 'YData', deadReckonedPosition(2), 'ZData', deadReckonedPosition(3), ...
            'UData', measuredVelocity(1), 'VData', measuredVelocity(2), 'WData', measuredVelocity(3));

        title(axesHandle, sprintf('UAV Dead Reckoning Animation: t = %.1f s', flightLog.time(k)));
        drawnow;

        if k < numel(flightLog.time)
            dt = flightLog.time(k+1) - flightLog.time(k);
            pause(dt / animationSpeedup);
        end
    end
end

function [model, origin] = createCurvedTrajectoryModel(trajectory, sampleTime)
%CREATECURVEDTRAJECTORYMODEL Build a cubic Hermite trajectory from timed points.
    validateTrajectory(trajectory, sampleTime);

    origin.latitude = trajectory(1,2);
    origin.longitude = trajectory(1,3);
    origin.altitude = trajectory(1,4);

    waypointENU = geodeticToENU(trajectory(:,2), trajectory(:,3), trajectory(:,4), origin);
    waypointVelocityENU = estimateWaypointVelocities(trajectory(:,1), waypointENU);

    model.time = trajectory(:,1);
    model.waypointENU = waypointENU;
    model.waypointVelocityENU = waypointVelocityENU;
end

function flightLog = flyDeadReckoningLoop(model, sampleTime, playbackInRealTime, biasENU, noiseStd, origin)
%FLYDEADRECKONINGLOOP Advance the simulated UAV and DR estimate sample by sample.
    startTime = model.time(1);
    endTime = model.time(end);
    maxSamples = ceil((endTime - startTime) / sampleTime) + 2;

    time = zeros(maxSamples, 1);
    referenceENU = zeros(maxSamples, 3);
    measuredVelocityENU = zeros(maxSamples, 3);
    deadReckonedENU = zeros(maxSamples, 3);

    sampleIndex = 1;
    currentTime = startTime;
    [referencePosition, referenceVelocity] = sampleCurvedTrajectory(model, currentTime);
    measuredVelocity = makeVelocityMeasurement(referenceVelocity, biasENU, noiseStd);
    deadReckonedPosition = referencePosition;

    time(sampleIndex) = currentTime;
    referenceENU(sampleIndex,:) = referencePosition;
    measuredVelocityENU(sampleIndex,:) = measuredVelocity;
    deadReckonedENU(sampleIndex,:) = deadReckonedPosition;

    while currentTime < endTime
        nextTime = min(currentTime + sampleTime, endTime);
        dt = nextTime - currentTime;

        if playbackInRealTime
            pause(dt);
        end

        % Dead reckoning is causal: propagate with the velocity measurement that
        % was available during the previous sample interval.
        deadReckonedPosition = deadReckonedPosition + measuredVelocity .* dt;

        currentTime = nextTime;
        [referencePosition, referenceVelocity] = sampleCurvedTrajectory(model, currentTime);
        measuredVelocity = makeVelocityMeasurement(referenceVelocity, biasENU, noiseStd);

        sampleIndex = sampleIndex + 1;
        time(sampleIndex) = currentTime;
        referenceENU(sampleIndex,:) = referencePosition;
        measuredVelocityENU(sampleIndex,:) = measuredVelocity;
        deadReckonedENU(sampleIndex,:) = deadReckonedPosition;
    end

    time = time(1:sampleIndex);
    referenceENU = referenceENU(1:sampleIndex,:);
    measuredVelocityENU = measuredVelocityENU(1:sampleIndex,:);
    deadReckonedENU = deadReckonedENU(1:sampleIndex,:);

    [latitude, longitude, altitude] = enuToGeodetic(deadReckonedENU, origin);
    [referenceLatitude, referenceLongitude, referenceAltitude] = enuToGeodetic(referenceENU, origin);

    flightLog.time = time;
    flightLog.referenceENU = referenceENU;
    flightLog.measuredVelocityENU = measuredVelocityENU;
    flightLog.deadReckonedENU = deadReckonedENU;
    flightLog.latitude = latitude;
    flightLog.longitude = longitude;
    flightLog.altitude = altitude;
    flightLog.referenceLatitude = referenceLatitude;
    flightLog.referenceLongitude = referenceLongitude;
    flightLog.referenceAltitude = referenceAltitude;
end

function [positionENU, velocityENU] = sampleCurvedTrajectory(model, queryTime)
%SAMPLECURVEDTRAJECTORY Evaluate position and velocity on a cubic Hermite curve.
    times = model.time;
    waypoints = model.waypointENU;
    waypointVelocities = model.waypointVelocityENU;

    if queryTime <= times(1)
        segmentIndex = 1;
    elseif queryTime >= times(end)
        segmentIndex = numel(times) - 1;
    else
        segmentIndex = find(times <= queryTime, 1, 'last');
        if segmentIndex == numel(times)
            segmentIndex = segmentIndex - 1;
        end
    end

    t0 = times(segmentIndex);
    t1 = times(segmentIndex + 1);
    p0 = waypoints(segmentIndex,:);
    p1 = waypoints(segmentIndex + 1,:);
    v0 = waypointVelocities(segmentIndex,:);
    v1 = waypointVelocities(segmentIndex + 1,:);

    duration = t1 - t0;
    s = (queryTime - t0) / duration;
    s = max(0, min(1, s));

    h00 = 2*s^3 - 3*s^2 + 1;
    h10 = s^3 - 2*s^2 + s;
    h01 = -2*s^3 + 3*s^2;
    h11 = s^3 - s^2;

    positionENU = h00*p0 + h10*duration*v0 + h01*p1 + h11*duration*v1;

    dh00 = 6*s^2 - 6*s;
    dh10 = 3*s^2 - 4*s + 1;
    dh01 = -6*s^2 + 6*s;
    dh11 = 3*s^2 - 2*s;

    velocityENU = (dh00*p0 + dh10*duration*v0 + dh01*p1 + dh11*duration*v1) ./ duration;
end

function waypointVelocityENU = estimateWaypointVelocities(time, waypointENU)
%ESTIMATEWAYPOINTVELOCITIES Estimate smooth curve tangents at timed waypoints.
    waypointVelocityENU = zeros(size(waypointENU));
    waypointVelocityENU(1,:) = (waypointENU(2,:) - waypointENU(1,:)) ./ (time(2) - time(1));
    waypointVelocityENU(end,:) = (waypointENU(end,:) - waypointENU(end-1,:)) ./ (time(end) - time(end-1));

    for k = 2:(numel(time)-1)
        waypointVelocityENU(k,:) = (waypointENU(k+1,:) - waypointENU(k-1,:)) ./ (time(k+1) - time(k-1));
    end
end

function measurementENU = makeVelocityMeasurement(trueVelocityENU, biasENU, noiseStd)
%MAKEVELOCITYMEASUREMENT Apply a simple bias/noise model to one ENU velocity.
    measurementENU = trueVelocityENU + biasENU + noiseStd .* randn(1, 3);
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
    if size(trajectory,1) < 2
        error('Trajectory must contain at least two timed control points.');
    end
    if any(diff(trajectory(:,1)) <= 0)
        error('Trajectory timestamps must be strictly increasing.');
    end
    if sampleTime <= 0
        error('sampleTime must be positive.');
    end
end
