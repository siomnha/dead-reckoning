%% 3-D curved, time-stepped IMU dead reckoning for a UAV trajectory
% This example replaces 2-D straight waypoint segments with a timestamped 3-D
% UAV trajectory.  The route between timed waypoints is a smooth cubic Hermite
% curve.  At each sampling interval the UAV now propagates with IMU data:
% gyroscope angular velocity updates attitude, and accelerometer specific force
% updates velocity and position.

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

% IMU/dead-reckoning update interval.  In an onboard implementation this is
% the sensor-fusion or inertial propagation cycle time.
sampleTime = 1.0;       % seconds
playbackInRealTime = false;  % set true to pause sampleTime seconds inside the simulation loop
animationSpeedup = 12;       % 1 = real time, 12 = play the 245 s route in about 20 s
showStaticPlots = true;      % keep summary plots after the animated flight

% Simple IMU sensor model.  Gyroscope measurements are body-frame angular
% velocity [rad/s].  Accelerometer measurements are body-frame specific force
% [m/s^2], so gravity is added back during navigation-frame propagation.
imuParams.gyroBiasBody = deg2rad([0.01, -0.015, 0.02]);       % [p q r] rad/s
imuParams.gyroNoiseStd = deg2rad(0.02);                       % rad/s
imuParams.accelBiasBody = [0.015, -0.010, 0.020];             % [fx fy fz] m/s^2
imuParams.accelNoiseStd = 0.025;                              % m/s^2
imuParams.gravityENU = [0, 0, -9.80665];                      % ENU gravity, up-positive
rng(7);                                                       % repeatable noise for the demo

%% Create the curved trajectory model and fly it one IMU sample at a time
[trajectoryModel, origin] = createCurvedTrajectoryModel(trajectory, sampleTime, imuParams.gravityENU);
flightLog = flyImuDeadReckoningLoop(trajectoryModel, sampleTime, playbackInRealTime, imuParams, origin);

%% Print the first time-stamped IMU DR samples
results = table( ...
    flightLog.time(:), ...
    flightLog.latitude(:), ...
    flightLog.longitude(:), ...
    flightLog.altitude(:), ...
    flightLog.deadReckonedENU(:,1), ...
    flightLog.deadReckonedENU(:,2), ...
    flightLog.deadReckonedENU(:,3), ...
    flightLog.deadReckonedVelocityENU(:,1), ...
    flightLog.deadReckonedVelocityENU(:,2), ...
    flightLog.deadReckonedVelocityENU(:,3), ...
    flightLog.measuredAngularVelocityBody(:,1), ...
    flightLog.measuredAngularVelocityBody(:,2), ...
    flightLog.measuredAngularVelocityBody(:,3), ...
    flightLog.measuredSpecificForceBody(:,1), ...
    flightLog.measuredSpecificForceBody(:,2), ...
    flightLog.measuredSpecificForceBody(:,3), ...
    'VariableNames', {'time_s','latitude_deg','longitude_deg','altitude_m', ...
    'east_m','north_m','up_m','ve_mps','vn_mps','vu_mps', ...
    'gyro_p_radps','gyro_q_radps','gyro_r_radps','accel_fx_mps2','accel_fy_mps2','accel_fz_mps2'});
disp(results(1:min(10,height(results)),:));

%% Animate the UAV flying the trajectory over time
animateFlightTrajectory(flightLog, trajectoryModel, animationSpeedup);

%% Optional static summary plots after the animation
if showStaticPlots
    figure('Name', '3-D Curved IMU Dead Reckoning');
    plot3(flightLog.referenceENU(:,1), flightLog.referenceENU(:,2), flightLog.referenceENU(:,3), ...
        '--', 'LineWidth', 1.4, 'DisplayName', 'Curved reference trajectory');
    hold on;
    plot3(flightLog.deadReckonedENU(:,1), flightLog.deadReckonedENU(:,2), flightLog.deadReckonedENU(:,3), ...
        '-', 'LineWidth', 1.5, 'DisplayName', 'IMU dead-reckoned trajectory');
    scatter3(trajectoryModel.waypointENU(:,1), trajectoryModel.waypointENU(:,2), trajectoryModel.waypointENU(:,3), ...
        45, 'filled', 'DisplayName', 'Timed control points');
    grid on; axis equal;
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up / altitude change (m)');
    title('3-D Curved Time-Stepped UAV IMU Dead Reckoning');
    legend('Location', 'best');
    view(38, 24);

    figure('Name', 'Map View with Altitude');
    geoplot(trajectory(:,2), trajectory(:,3), 'o', 'LineWidth', 1.1, 'DisplayName', 'Timed control points');
    hold on;
    geoscatter(flightLog.referenceLatitude, flightLog.referenceLongitude, 10, flightLog.referenceAltitude, ...
        'filled', 'DisplayName', 'Sampled curved reference');
    geoscatter(flightLog.latitude, flightLog.longitude, 12, flightLog.altitude, 'filled', ...
        'DisplayName', 'IMU dead-reckoned samples');
    geobasemap streets;
    cb = colorbar;
    cb.Label.String = 'Altitude (m)';
    title('Curved UAV IMU Dead-Reckoning Samples');
    legend('Location', 'best');

    figure('Name', 'IMU Measurements');
    tiledlayout(2,1);
    nexttile;
    plot(flightLog.time, flightLog.measuredAngularVelocityBody, 'LineWidth', 1.2);
    grid on;
    ylabel('Angular velocity (rad/s)');
    legend('p','q','r', 'Location', 'best');
    title('Measured Gyroscope Output');
    nexttile;
    plot(flightLog.time, flightLog.measuredSpecificForceBody, 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Specific force (m/s^2)');
    legend('f_x','f_y','f_z', 'Location', 'best');
    title('Measured Accelerometer Output');
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

    figure('Name', 'Animated UAV IMU Dead Reckoning');
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
        'Color', [0.9 0.25 0.1], 'DisplayName', 'IMU DR flown so far');

    referenceAircraft = plot3(axesHandle, NaN, NaN, NaN, '^', 'MarkerSize', 9, ...
        'MarkerFaceColor', [0.1 0.45 0.9], 'MarkerEdgeColor', 'k', 'DisplayName', 'Reference UAV');
    deadReckonedAircraft = plot3(axesHandle, NaN, NaN, NaN, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.9 0.25 0.1], 'MarkerEdgeColor', 'k', 'DisplayName', 'IMU DR estimate');
    velocityVector = quiver3(axesHandle, NaN, NaN, NaN, NaN, NaN, NaN, 0, ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 1.1, 'DisplayName', 'Estimated velocity');

    legend(axesHandle, 'Location', 'best');

    for k = 1:numel(flightLog.time)
        referencePosition = flightLog.referenceENU(k,:);
        deadReckonedPosition = flightLog.deadReckonedENU(k,:);
        estimatedVelocity = flightLog.deadReckonedVelocityENU(k,:);

        addpoints(referenceTrail, referencePosition(1), referencePosition(2), referencePosition(3));
        addpoints(deadReckonedTrail, deadReckonedPosition(1), deadReckonedPosition(2), deadReckonedPosition(3));

        set(referenceAircraft, 'XData', referencePosition(1), 'YData', referencePosition(2), 'ZData', referencePosition(3));
        set(deadReckonedAircraft, 'XData', deadReckonedPosition(1), 'YData', deadReckonedPosition(2), 'ZData', deadReckonedPosition(3));
        set(velocityVector, ...
            'XData', deadReckonedPosition(1), 'YData', deadReckonedPosition(2), 'ZData', deadReckonedPosition(3), ...
            'UData', estimatedVelocity(1), 'VData', estimatedVelocity(2), 'WData', estimatedVelocity(3));

        title(axesHandle, sprintf('UAV IMU Dead Reckoning Animation: t = %.1f s', flightLog.time(k)));
        drawnow;

        if k < numel(flightLog.time)
            dt = flightLog.time(k+1) - flightLog.time(k);
            pause(dt / animationSpeedup);
        end
    end
end

function [model, origin] = createCurvedTrajectoryModel(trajectory, sampleTime, gravityENU)
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
    model.gravityENU = gravityENU;
end

function flightLog = flyImuDeadReckoningLoop(model, sampleTime, playbackInRealTime, imuParams, origin)
%FLYIMUDEADRECKONINGLOOP Propagate attitude, velocity, and position from IMU.
    startTime = model.time(1);
    endTime = model.time(end);
    maxSamples = ceil((endTime - startTime) / sampleTime) + 2;

    time = zeros(maxSamples, 1);
    referenceENU = zeros(maxSamples, 3);
    referenceVelocityENU = zeros(maxSamples, 3);
    referenceAccelerationENU = zeros(maxSamples, 3);
    measuredAngularVelocityBody = zeros(maxSamples, 3);
    measuredSpecificForceBody = zeros(maxSamples, 3);
    deadReckonedENU = zeros(maxSamples, 3);
    deadReckonedVelocityENU = zeros(maxSamples, 3);
    deadReckonedEuler = zeros(maxSamples, 3);

    sampleIndex = 1;
    currentTime = startTime;
    [referencePosition, referenceVelocity, referenceAcceleration] = sampleCurvedTrajectory(model, currentTime);
    referenceAttitude = attitudeFromVelocity(referenceVelocity);
    imuMeasurement = makeImuMeasurement(referenceAttitude, [], referenceAcceleration, sampleTime, imuParams);

    deadReckonedPosition = referencePosition;
    deadReckonedVelocity = referenceVelocity;
    deadReckonedAttitude = referenceAttitude;

    [time, referenceENU, referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, ...
        measuredSpecificForceBody, deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler] = ...
        logFlightSample(sampleIndex, currentTime, referencePosition, referenceVelocity, referenceAcceleration, ...
        imuMeasurement, deadReckonedPosition, deadReckonedVelocity, deadReckonedAttitude, time, referenceENU, ...
        referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, measuredSpecificForceBody, ...
        deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler);

    while currentTime < endTime
        nextTime = min(currentTime + sampleTime, endTime);
        dt = nextTime - currentTime;

        [nextReferencePosition, nextReferenceVelocity, nextReferenceAcceleration] = sampleCurvedTrajectory(model, nextTime);
        nextReferenceAttitude = attitudeFromVelocity(nextReferenceVelocity);
        intervalImuMeasurement = makeImuMeasurement(nextReferenceAttitude, referenceAttitude, ...
            referenceAcceleration, dt, imuParams);

        if playbackInRealTime
            pause(dt);
        end

        % Strapdown IMU propagation.  The interval gyro measurement integrates
        % attitude; the interval accelerometer specific force is rotated into ENU
        % and gravity is restored to obtain translational acceleration.
        deadReckonedAttitude = deadReckonedAttitude * rotationExp(intervalImuMeasurement.angularVelocityBody .* dt);
        estimatedAcceleration = deadReckonedAttitude * intervalImuMeasurement.specificForceBody.' + imuParams.gravityENU.';
        estimatedAcceleration = estimatedAcceleration.';
        deadReckonedPosition = deadReckonedPosition + deadReckonedVelocity .* dt + 0.5 .* estimatedAcceleration .* dt.^2;
        deadReckonedVelocity = deadReckonedVelocity + estimatedAcceleration .* dt;

        currentTime = nextTime;
        referencePosition = nextReferencePosition;
        referenceVelocity = nextReferenceVelocity;
        referenceAcceleration = nextReferenceAcceleration;
        referenceAttitude = nextReferenceAttitude;
        imuMeasurement = intervalImuMeasurement;

        sampleIndex = sampleIndex + 1;
        [time, referenceENU, referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, ...
            measuredSpecificForceBody, deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler] = ...
            logFlightSample(sampleIndex, currentTime, referencePosition, referenceVelocity, referenceAcceleration, ...
            imuMeasurement, deadReckonedPosition, deadReckonedVelocity, deadReckonedAttitude, time, referenceENU, ...
            referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, measuredSpecificForceBody, ...
            deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler);
    end

    time = time(1:sampleIndex);
    referenceENU = referenceENU(1:sampleIndex,:);
    referenceVelocityENU = referenceVelocityENU(1:sampleIndex,:);
    referenceAccelerationENU = referenceAccelerationENU(1:sampleIndex,:);
    measuredAngularVelocityBody = measuredAngularVelocityBody(1:sampleIndex,:);
    measuredSpecificForceBody = measuredSpecificForceBody(1:sampleIndex,:);
    deadReckonedENU = deadReckonedENU(1:sampleIndex,:);
    deadReckonedVelocityENU = deadReckonedVelocityENU(1:sampleIndex,:);
    deadReckonedEuler = deadReckonedEuler(1:sampleIndex,:);

    [latitude, longitude, altitude] = enuToGeodetic(deadReckonedENU, origin);
    [referenceLatitude, referenceLongitude, referenceAltitude] = enuToGeodetic(referenceENU, origin);

    flightLog.time = time;
    flightLog.referenceENU = referenceENU;
    flightLog.referenceVelocityENU = referenceVelocityENU;
    flightLog.referenceAccelerationENU = referenceAccelerationENU;
    flightLog.measuredAngularVelocityBody = measuredAngularVelocityBody;
    flightLog.measuredSpecificForceBody = measuredSpecificForceBody;
    flightLog.deadReckonedENU = deadReckonedENU;
    flightLog.deadReckonedVelocityENU = deadReckonedVelocityENU;
    flightLog.deadReckonedEuler = deadReckonedEuler;
    flightLog.latitude = latitude;
    flightLog.longitude = longitude;
    flightLog.altitude = altitude;
    flightLog.referenceLatitude = referenceLatitude;
    flightLog.referenceLongitude = referenceLongitude;
    flightLog.referenceAltitude = referenceAltitude;
end

function [time, referenceENU, referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, ...
    measuredSpecificForceBody, deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler] = ...
    logFlightSample(sampleIndex, currentTime, referencePosition, referenceVelocity, referenceAcceleration, ...
    imuMeasurement, deadReckonedPosition, deadReckonedVelocity, deadReckonedAttitude, time, referenceENU, ...
    referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, measuredSpecificForceBody, ...
    deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler)
%LOGFLIGHTSAMPLE Store one reference, IMU, and propagated dead-reckoning state.
    time(sampleIndex) = currentTime;
    referenceENU(sampleIndex,:) = referencePosition;
    referenceVelocityENU(sampleIndex,:) = referenceVelocity;
    referenceAccelerationENU(sampleIndex,:) = referenceAcceleration;
    measuredAngularVelocityBody(sampleIndex,:) = imuMeasurement.angularVelocityBody;
    measuredSpecificForceBody(sampleIndex,:) = imuMeasurement.specificForceBody;
    deadReckonedENU(sampleIndex,:) = deadReckonedPosition;
    deadReckonedVelocityENU(sampleIndex,:) = deadReckonedVelocity;
    deadReckonedEuler(sampleIndex,:) = attitudeToEulerZYX(deadReckonedAttitude);
end

function [positionENU, velocityENU, accelerationENU] = sampleCurvedTrajectory(model, queryTime)
%SAMPLECURVEDTRAJECTORY Evaluate position, velocity, and acceleration on a Hermite curve.
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

    ddh00 = 12*s - 6;
    ddh10 = 6*s - 4;
    ddh01 = -12*s + 6;
    ddh11 = 6*s - 2;

    accelerationENU = (ddh00*p0 + ddh10*duration*v0 + ddh01*p1 + ddh11*duration*v1) ./ duration.^2;
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

function imuMeasurement = makeImuMeasurement(currentAttitude, previousAttitude, accelerationENU, dt, imuParams)
%MAKEIMUMEASUREMENT Simulate one gyro and accelerometer measurement.
    if isempty(previousAttitude) || dt <= 0
        trueAngularVelocityBody = [0, 0, 0];
    else
        trueAngularVelocityBody = rotationLog(previousAttitude.' * currentAttitude) ./ dt;
    end

    trueSpecificForceBody = (currentAttitude.' * (accelerationENU - imuParams.gravityENU).').';

    imuMeasurement.angularVelocityBody = trueAngularVelocityBody + imuParams.gyroBiasBody + ...
        imuParams.gyroNoiseStd .* randn(1, 3);
    imuMeasurement.specificForceBody = trueSpecificForceBody + imuParams.accelBiasBody + ...
        imuParams.accelNoiseStd .* randn(1, 3);
end

function attitude = attitudeFromVelocity(velocityENU)
%ATTITUDEFROMVELOCITY Build a simple body-to-ENU attitude from flight direction.
    speed = norm(velocityENU);
    if speed < 1e-6
        forward = [1, 0, 0];
    else
        forward = velocityENU ./ speed;
    end

    worldUp = [0, 0, 1];
    left = cross(worldUp, forward);
    if norm(left) < 1e-6
        left = [0, 1, 0];
    else
        left = left ./ norm(left);
    end
    up = cross(forward, left);
    up = up ./ norm(up);

    attitude = [forward(:), left(:), up(:)];
end

function rotationMatrix = rotationExp(rotationVector)
%ROTATIONEXP Convert a rotation vector to a rotation matrix with Rodrigues' formula.
    angle = norm(rotationVector);
    if angle < 1e-12
        rotationMatrix = eye(3) + skewSymmetric(rotationVector);
        return;
    end

    axis = rotationVector ./ angle;
    axisSkew = skewSymmetric(axis);
    rotationMatrix = eye(3) + sin(angle) .* axisSkew + (1 - cos(angle)) .* (axisSkew * axisSkew);
end

function rotationVector = rotationLog(rotationMatrix)
%ROTATIONLOG Convert a rotation matrix to a rotation vector.
    cosAngle = (trace(rotationMatrix) - 1) / 2;
    cosAngle = max(-1, min(1, cosAngle));
    angle = acos(cosAngle);

    if angle < 1e-12
        rotationVector = [0, 0, 0];
        return;
    end

    rotationVector = angle ./ (2 .* sin(angle)) .* [
        rotationMatrix(3,2) - rotationMatrix(2,3), ...
        rotationMatrix(1,3) - rotationMatrix(3,1), ...
        rotationMatrix(2,1) - rotationMatrix(1,2)];
end

function skew = skewSymmetric(vector)
%SKEWSYMMETRIC Return the skew-symmetric matrix for a 3-D vector.
    skew = [
        0, -vector(3), vector(2);
        vector(3), 0, -vector(1);
        -vector(2), vector(1), 0];
end

function eulerZYX = attitudeToEulerZYX(attitude)
%ATTITUDETOEULERZYX Convert body-to-ENU attitude to [yaw pitch roll] radians.
    yaw = atan2(attitude(2,1), attitude(1,1));
    pitch = atan2(-attitude(3,1), hypot(attitude(1,1), attitude(2,1)));
    roll = atan2(attitude(3,2), attitude(3,3));
    eulerZYX = [yaw, pitch, roll];
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
