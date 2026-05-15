%% 3-D curved, time-stepped IMU dead reckoning for a UAV trajectory
% This example replaces 2-D straight waypoint segments with a timestamped 3-D
% UAV trajectory.  The route between timed waypoints is a smooth cubic Hermite
% curve.  At each sampling interval the UAV now propagates with IMU data:
% gyroscope angular velocity updates attitude, and body-frame accelerometer measurements
% update velocity and position.

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
% velocity [rad/s].  By default the accelerometer channel is treated as a
% gravity-compensated body-frame linear acceleration, so it does not carry a
% constant ~9.81 m/s^2 offset.  Set accelerometerIncludesGravity = true to
% model a raw specific-force accelerometer instead.
imuParams.gyroBiasBody = deg2rad([0.001, -0.0015, 0.002]);    % [p q r] rad/s
imuParams.gyroNoiseStd = deg2rad(0.002);                      % rad/s
imuParams.accelBiasBody = [0.002, -0.0015, 0.003];            % [ax ay az] m/s^2
imuParams.accelNoiseStd = 0.005;                              % m/s^2
imuParams.gravityENU = [0, 0, -9.80665];                      % ENU gravity, up-positive
imuParams.accelerometerIncludesGravity = false;               % false avoids raw 1-g accelerometer offset

% Lidar-inertial odometry (LIO) scan-matching settings.  Landmarks are known
% 3-D map points around the route; each scan observes nearby landmarks in the
% body frame and the EKF fuses the scan-matched position with IMU propagation.
lioParams.lidarRange = 180;                 % m
lioParams.lidarNoiseStd = 0.35;             % m, per landmark body-frame point noise
lioParams.minLandmarksForUpdate = 3;        % minimum correspondences for scan matching
lioParams.initialPositionStd = 1.0;         % m
lioParams.initialVelocityStd = 0.5;         % m/s
lioParams.accelProcessNoiseStd = 0.08;      % m/s^2
rng(7);                                    % repeatable noise for the demo

%% Create the curved trajectory model, landmarks, and fly one IMU sample at a time
[trajectoryModel, origin] = createCurvedTrajectoryModel(trajectory, sampleTime, imuParams.gravityENU);
consistencyReport = checkTrajectoryWaypointConsistency(trajectoryModel);
disp(consistencyReport);
landmarksENU = generatePredefinedLandmarks(trajectoryModel);
flightLog = flyImuDeadReckoningLoop(trajectoryModel, sampleTime, playbackInRealTime, ...
    imuParams, lioParams, landmarksENU, origin);

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
    flightLog.measuredAccelerationBody(:,1), ...
    flightLog.measuredAccelerationBody(:,2), ...
    flightLog.measuredAccelerationBody(:,3), ...
    flightLog.lioENU(:,1), ...
    flightLog.lioENU(:,2), ...
    flightLog.lioENU(:,3), ...
    flightLog.lidarMatchedLandmarkCount(:), ...
    'VariableNames', {'time_s','latitude_deg','longitude_deg','altitude_m', ...
    'east_m','north_m','up_m','ve_mps','vn_mps','vu_mps', ...
    'gyro_p_radps','gyro_q_radps','gyro_r_radps','accel_x_mps2','accel_y_mps2','accel_z_mps2', ...
    'lio_east_m','lio_north_m','lio_up_m','matched_landmarks'});
disp(results(1:min(10,height(results)),:));

pureImuError = vecnorm(flightLog.deadReckonedENU - flightLog.referenceENU, 2, 2);
lioError = vecnorm(flightLog.lioENU - flightLog.referenceENU, 2, 2);
comparison = table(flightLog.time(end), pureImuError(end), lioError(end), ...
    mean(flightLog.lidarMatchedLandmarkCount), ...
    'VariableNames', {'final_time_s','pure_imu_final_error_m','lio_final_error_m','mean_matched_landmarks'});
disp(comparison);

%% Animate the UAV flying the trajectory over time
animateFlightTrajectory(flightLog, trajectoryModel, landmarksENU, animationSpeedup);

%% Optional static summary plots after the animation
if showStaticPlots
    figure('Name', '3-D Curved IMU Dead Reckoning');
    plot3(flightLog.referenceENU(:,1), flightLog.referenceENU(:,2), flightLog.referenceENU(:,3), ...
        '--', 'LineWidth', 1.4, 'DisplayName', 'Curved reference trajectory');
    hold on;
    plot3(flightLog.deadReckonedENU(:,1), flightLog.deadReckonedENU(:,2), flightLog.deadReckonedENU(:,3), ...
        '-', 'LineWidth', 1.5, 'DisplayName', 'IMU dead-reckoned trajectory');
    plot3(flightLog.lioENU(:,1), flightLog.lioENU(:,2), flightLog.lioENU(:,3), ...
        '-', 'LineWidth', 1.5, 'DisplayName', 'LIO EKF trajectory');
    plot3(landmarksENU(:,1), landmarksENU(:,2), landmarksENU(:,3), ...
        'd', 'MarkerSize', 5, 'Color', [0.2 0.6 0.2], 'DisplayName', 'Predefined lidar landmarks');
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
    geoscatter(flightLog.lioLatitude, flightLog.lioLongitude, 12, flightLog.lioAltitude, 'filled', ...
        'DisplayName', 'LIO EKF samples');
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
    plot(flightLog.time, flightLog.measuredAccelerationBody, 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Body acceleration (m/s^2)');
    legend('a_x','a_y','a_z', 'Location', 'best');
    title('Measured Accelerometer Output');
end


function animateFlightTrajectory(flightLog, model, landmarksENU, animationSpeedup)
%ANIMATEFLIGHTTRAJECTORY Replay the UAV trajectory sample by sample.
    if animationSpeedup <= 0
        error('animationSpeedup must be positive.');
    end

    allPositions = [flightLog.referenceENU; flightLog.deadReckonedENU; flightLog.lioENU; model.waypointENU; landmarksENU];
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
    plot3(axesHandle, landmarksENU(:,1), landmarksENU(:,2), landmarksENU(:,3), ...
        'd', 'MarkerSize', 5, 'Color', [0.2 0.6 0.2], 'DisplayName', 'Lidar landmarks');

    referenceTrail = animatedline(axesHandle, 'LineStyle', '--', 'LineWidth', 1.4, ...
        'Color', [0.1 0.45 0.9], 'DisplayName', 'Reference flown so far');
    deadReckonedTrail = animatedline(axesHandle, 'LineStyle', '-', 'LineWidth', 1.7, ...
        'Color', [0.9 0.25 0.1], 'DisplayName', 'IMU DR flown so far');
    lioTrail = animatedline(axesHandle, 'LineStyle', '-', 'LineWidth', 1.7, ...
        'Color', [0.2 0.65 0.2], 'DisplayName', 'LIO EKF flown so far');

    referenceAircraft = plot3(axesHandle, NaN, NaN, NaN, '^', 'MarkerSize', 9, ...
        'MarkerFaceColor', [0.1 0.45 0.9], 'MarkerEdgeColor', 'k', 'DisplayName', 'Reference UAV');
    deadReckonedAircraft = plot3(axesHandle, NaN, NaN, NaN, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.9 0.25 0.1], 'MarkerEdgeColor', 'k', 'DisplayName', 'IMU DR estimate');
    lioAircraft = plot3(axesHandle, NaN, NaN, NaN, 's', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.2 0.65 0.2], 'MarkerEdgeColor', 'k', 'DisplayName', 'LIO EKF estimate');
    velocityVector = quiver3(axesHandle, NaN, NaN, NaN, NaN, NaN, NaN, 0, ...
        'Color', [0.1 0.1 0.1], 'LineWidth', 1.1, 'DisplayName', 'Estimated velocity');

    legend(axesHandle, 'Location', 'best');

    for k = 1:numel(flightLog.time)
        referencePosition = flightLog.referenceENU(k,:);
        deadReckonedPosition = flightLog.deadReckonedENU(k,:);
        lioPosition = flightLog.lioENU(k,:);
        estimatedVelocity = flightLog.deadReckonedVelocityENU(k,:);

        addpoints(referenceTrail, referencePosition(1), referencePosition(2), referencePosition(3));
        addpoints(deadReckonedTrail, deadReckonedPosition(1), deadReckonedPosition(2), deadReckonedPosition(3));
        addpoints(lioTrail, lioPosition(1), lioPosition(2), lioPosition(3));

        set(referenceAircraft, 'XData', referencePosition(1), 'YData', referencePosition(2), 'ZData', referencePosition(3));
        set(deadReckonedAircraft, 'XData', deadReckonedPosition(1), 'YData', deadReckonedPosition(2), 'ZData', deadReckonedPosition(3));
        set(lioAircraft, 'XData', lioPosition(1), 'YData', lioPosition(2), 'ZData', lioPosition(3));
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

function consistencyReport = checkTrajectoryWaypointConsistency(model)
%CHECKTRAJECTORYWAYPOINTCONSISTENCY Confirm the curved trajectory hits every control point.
    sampledENU = zeros(size(model.waypointENU));
    for k = 1:numel(model.time)
        sampledENU(k,:) = sampleCurvedTrajectory(model, model.time(k));
    end

    positionError = sampledENU - model.waypointENU;
    errorNorm = vecnorm(positionError, 2, 2);
    consistencyReport = table(model.time(:), errorNorm(:), ...
        'VariableNames', {'time_s','curved_waypoint_error_m'});
    maxError = max(errorNorm);
    fprintf('Max curved trajectory waypoint consistency error: %.6g m\n', maxError);
    if maxError > 1e-6
        warning('Curved trajectory does not pass exactly through every predefined waypoint.');
    end
end

function landmarksENU = generatePredefinedLandmarks(model)
%GENERATEPREDEFINEDLANDMARKS Place deterministic 3-D landmarks around the route.
    sampleTimes = linspace(model.time(1), model.time(end), 14);
    lateralOffsets = [-70; 70];
    verticalOffsets = [18; -12];
    landmarksENU = zeros(numel(sampleTimes) * numel(lateralOffsets), 3);
    landmarkIndex = 1;

    for k = 1:numel(sampleTimes)
        [positionENU, velocityENU] = sampleCurvedTrajectory(model, sampleTimes(k));
        horizontalVelocity = velocityENU(1:2);
        if norm(horizontalVelocity) < 1e-6
            side = [0, 1, 0];
        else
            side2D = [-horizontalVelocity(2), horizontalVelocity(1)] ./ norm(horizontalVelocity);
            side = [side2D, 0];
        end

        for j = 1:numel(lateralOffsets)
            alongOffset = 18 .* sin(0.7 .* k + j);
            forward = [velocityENU(1:2), 0];
            if norm(forward) > 1e-6
                forward = forward ./ norm(forward);
            end
            landmarksENU(landmarkIndex,:) = positionENU + lateralOffsets(j) .* side + ...
                alongOffset .* forward + [0, 0, verticalOffsets(j)];
            landmarkIndex = landmarkIndex + 1;
        end
    end
end

function accelerationENU = accelerationMeasurementToENU(accelerationBody, attitude, imuParams)
%ACCELERATIONMEASUREMENTTOENU Rotate body acceleration to ENU and restore gravity if needed.
    accelerationENU = attitude * accelerationBody.';
    if imuParams.accelerometerIncludesGravity
        accelerationENU = accelerationENU + imuParams.gravityENU.';
    end
    accelerationENU = accelerationENU.';
end

function [lioState, lioCovariance] = propagateLioState(lioState, lioCovariance, accelerationENU, dt, lioParams)
%PROPAGATELIOSTATE EKF prediction using the same IMU acceleration as pure DR.
    stateTransition = [eye(3), dt .* eye(3); zeros(3), eye(3)];
    controlMatrix = [0.5 .* dt.^2 .* eye(3); dt .* eye(3)];

    lioState = stateTransition * lioState + controlMatrix * accelerationENU.';
    processNoise = (lioParams.accelProcessNoiseStd.^2) .* (controlMatrix * controlMatrix.');
    lioCovariance = stateTransition * lioCovariance * stateTransition.' + processNoise;
end

function [matchedPositionENU, matchedCount] = scanMatchLidarPosition(referencePosition, referenceAttitude, estimatedAttitude, landmarksENU, lioParams)
%SCANMATCHLIDARPOSITION Estimate position by matching visible landmarks to the known map.
    landmarkDelta = landmarksENU - referencePosition;
    ranges = vecnorm(landmarkDelta, 2, 2);
    visibleLandmarks = find(ranges <= lioParams.lidarRange);
    matchedCount = numel(visibleLandmarks);

    if matchedCount == 0
        matchedPositionENU = [NaN, NaN, NaN];
        return;
    end

    positionCandidates = zeros(matchedCount, 3);
    for k = 1:matchedCount
        landmarkIndex = visibleLandmarks(k);
        trueBodyPoint = referenceAttitude.' * (landmarksENU(landmarkIndex,:) - referencePosition).';
        measuredBodyPoint = trueBodyPoint.' + lioParams.lidarNoiseStd .* randn(1, 3);
        positionCandidates(k,:) = landmarksENU(landmarkIndex,:) - (estimatedAttitude * measuredBodyPoint.').';
    end

    matchedPositionENU = mean(positionCandidates, 1);
end

function [lioState, lioCovariance] = updateLioWithScanMatch(lioState, lioCovariance, lidarPositionENU, matchedCount, lioParams)
%UPDATELIOWITHSCANMATCH EKF position update from landmark scan matching.
    observationMatrix = [eye(3), zeros(3)];
    measurementNoise = (lioParams.lidarNoiseStd.^2 ./ matchedCount) .* eye(3);
    innovation = lidarPositionENU.' - observationMatrix * lioState;
    innovationCovariance = observationMatrix * lioCovariance * observationMatrix.' + measurementNoise;
    kalmanGain = lioCovariance * observationMatrix.' / innovationCovariance;

    lioState = lioState + kalmanGain * innovation;
    lioCovariance = (eye(6) - kalmanGain * observationMatrix) * lioCovariance;
end

function flightLog = flyImuDeadReckoningLoop(model, sampleTime, playbackInRealTime, imuParams, lioParams, landmarksENU, origin)
%FLYIMUDEADRECKONINGLOOP Propagate pure IMU and LIO EKF states from IMU/lidar.
    startTime = model.time(1);
    endTime = model.time(end);
    maxSamples = ceil((endTime - startTime) / sampleTime) + 2;

    time = zeros(maxSamples, 1);
    referenceENU = zeros(maxSamples, 3);
    referenceVelocityENU = zeros(maxSamples, 3);
    referenceAccelerationENU = zeros(maxSamples, 3);
    measuredAngularVelocityBody = zeros(maxSamples, 3);
    measuredAccelerationBody = zeros(maxSamples, 3);
    deadReckonedENU = zeros(maxSamples, 3);
    deadReckonedVelocityENU = zeros(maxSamples, 3);
    deadReckonedEuler = zeros(maxSamples, 3);
    lioENU = zeros(maxSamples, 3);
    lioVelocityENU = zeros(maxSamples, 3);
    lioPositionStdENU = zeros(maxSamples, 3);
    lidarMatchedLandmarkCount = zeros(maxSamples, 1);

    sampleIndex = 1;
    currentTime = startTime;
    [referencePosition, referenceVelocity, referenceAcceleration] = sampleCurvedTrajectory(model, currentTime);
    referenceAttitude = attitudeFromVelocity(referenceVelocity);
    imuMeasurement = makeImuMeasurement(referenceAttitude, [], referenceAcceleration, sampleTime, imuParams);

    deadReckonedPosition = referencePosition;
    deadReckonedVelocity = referenceVelocity;
    deadReckonedAttitude = referenceAttitude;

    lioState = [referencePosition, referenceVelocity].';
    lioCovariance = diag([repmat(lioParams.initialPositionStd.^2, 1, 3), ...
        repmat(lioParams.initialVelocityStd.^2, 1, 3)]);

    [time, referenceENU, referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, ...
        measuredAccelerationBody, deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler, ...
        lioENU, lioVelocityENU, lioPositionStdENU, lidarMatchedLandmarkCount] = ...
        logFlightSample(sampleIndex, currentTime, referencePosition, referenceVelocity, referenceAcceleration, ...
        imuMeasurement, deadReckonedPosition, deadReckonedVelocity, deadReckonedAttitude, lioState, ...
        lioCovariance, 0, time, referenceENU, referenceVelocityENU, referenceAccelerationENU, ...
        measuredAngularVelocityBody, measuredAccelerationBody, deadReckonedENU, deadReckonedVelocityENU, ...
        deadReckonedEuler, lioENU, lioVelocityENU, lioPositionStdENU, lidarMatchedLandmarkCount);

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

        deadReckonedAttitude = deadReckonedAttitude * rotationExp(intervalImuMeasurement.angularVelocityBody .* dt);
        estimatedAcceleration = accelerationMeasurementToENU(intervalImuMeasurement.accelerationBody, ...
            deadReckonedAttitude, imuParams);
        deadReckonedPosition = deadReckonedPosition + deadReckonedVelocity .* dt + 0.5 .* estimatedAcceleration .* dt.^2;
        deadReckonedVelocity = deadReckonedVelocity + estimatedAcceleration .* dt;

        [lioState, lioCovariance] = propagateLioState(lioState, lioCovariance, estimatedAcceleration, dt, lioParams);
        [lidarPositionENU, matchedCount] = scanMatchLidarPosition(nextReferencePosition, nextReferenceAttitude, ...
            deadReckonedAttitude, landmarksENU, lioParams);
        if matchedCount >= lioParams.minLandmarksForUpdate
            [lioState, lioCovariance] = updateLioWithScanMatch(lioState, lioCovariance, lidarPositionENU, ...
                matchedCount, lioParams);
        end

        currentTime = nextTime;
        referencePosition = nextReferencePosition;
        referenceVelocity = nextReferenceVelocity;
        referenceAcceleration = nextReferenceAcceleration;
        referenceAttitude = nextReferenceAttitude;
        imuMeasurement = intervalImuMeasurement;

        sampleIndex = sampleIndex + 1;
        [time, referenceENU, referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, ...
            measuredAccelerationBody, deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler, ...
            lioENU, lioVelocityENU, lioPositionStdENU, lidarMatchedLandmarkCount] = ...
            logFlightSample(sampleIndex, currentTime, referencePosition, referenceVelocity, referenceAcceleration, ...
            imuMeasurement, deadReckonedPosition, deadReckonedVelocity, deadReckonedAttitude, lioState, ...
            lioCovariance, matchedCount, time, referenceENU, referenceVelocityENU, referenceAccelerationENU, ...
            measuredAngularVelocityBody, measuredAccelerationBody, deadReckonedENU, deadReckonedVelocityENU, ...
            deadReckonedEuler, lioENU, lioVelocityENU, lioPositionStdENU, lidarMatchedLandmarkCount);
    end

    time = time(1:sampleIndex);
    referenceENU = referenceENU(1:sampleIndex,:);
    referenceVelocityENU = referenceVelocityENU(1:sampleIndex,:);
    referenceAccelerationENU = referenceAccelerationENU(1:sampleIndex,:);
    measuredAngularVelocityBody = measuredAngularVelocityBody(1:sampleIndex,:);
    measuredAccelerationBody = measuredAccelerationBody(1:sampleIndex,:);
    deadReckonedENU = deadReckonedENU(1:sampleIndex,:);
    deadReckonedVelocityENU = deadReckonedVelocityENU(1:sampleIndex,:);
    deadReckonedEuler = deadReckonedEuler(1:sampleIndex,:);
    lioENU = lioENU(1:sampleIndex,:);
    lioVelocityENU = lioVelocityENU(1:sampleIndex,:);
    lioPositionStdENU = lioPositionStdENU(1:sampleIndex,:);
    lidarMatchedLandmarkCount = lidarMatchedLandmarkCount(1:sampleIndex);

    [latitude, longitude, altitude] = enuToGeodetic(deadReckonedENU, origin);
    [lioLatitude, lioLongitude, lioAltitude] = enuToGeodetic(lioENU, origin);
    [referenceLatitude, referenceLongitude, referenceAltitude] = enuToGeodetic(referenceENU, origin);

    flightLog.time = time;
    flightLog.referenceENU = referenceENU;
    flightLog.referenceVelocityENU = referenceVelocityENU;
    flightLog.referenceAccelerationENU = referenceAccelerationENU;
    flightLog.measuredAngularVelocityBody = measuredAngularVelocityBody;
    flightLog.measuredAccelerationBody = measuredAccelerationBody;
    flightLog.deadReckonedENU = deadReckonedENU;
    flightLog.deadReckonedVelocityENU = deadReckonedVelocityENU;
    flightLog.deadReckonedEuler = deadReckonedEuler;
    flightLog.lioENU = lioENU;
    flightLog.lioVelocityENU = lioVelocityENU;
    flightLog.lioPositionStdENU = lioPositionStdENU;
    flightLog.lidarMatchedLandmarkCount = lidarMatchedLandmarkCount;
    flightLog.latitude = latitude;
    flightLog.longitude = longitude;
    flightLog.altitude = altitude;
    flightLog.lioLatitude = lioLatitude;
    flightLog.lioLongitude = lioLongitude;
    flightLog.lioAltitude = lioAltitude;
    flightLog.referenceLatitude = referenceLatitude;
    flightLog.referenceLongitude = referenceLongitude;
    flightLog.referenceAltitude = referenceAltitude;
end

function [time, referenceENU, referenceVelocityENU, referenceAccelerationENU, measuredAngularVelocityBody, ...
    measuredAccelerationBody, deadReckonedENU, deadReckonedVelocityENU, deadReckonedEuler, ...
    lioENU, lioVelocityENU, lioPositionStdENU, lidarMatchedLandmarkCount] = ...
    logFlightSample(sampleIndex, currentTime, referencePosition, referenceVelocity, referenceAcceleration, ...
    imuMeasurement, deadReckonedPosition, deadReckonedVelocity, deadReckonedAttitude, lioState, ...
    lioCovariance, matchedCount, time, referenceENU, referenceVelocityENU, referenceAccelerationENU, ...
    measuredAngularVelocityBody, measuredAccelerationBody, deadReckonedENU, deadReckonedVelocityENU, ...
    deadReckonedEuler, lioENU, lioVelocityENU, lioPositionStdENU, lidarMatchedLandmarkCount)
%LOGFLIGHTSAMPLE Store one reference, IMU, pure-IMU, and LIO state sample.
    time(sampleIndex) = currentTime;
    referenceENU(sampleIndex,:) = referencePosition;
    referenceVelocityENU(sampleIndex,:) = referenceVelocity;
    referenceAccelerationENU(sampleIndex,:) = referenceAcceleration;
    measuredAngularVelocityBody(sampleIndex,:) = imuMeasurement.angularVelocityBody;
    measuredAccelerationBody(sampleIndex,:) = imuMeasurement.accelerationBody;
    deadReckonedENU(sampleIndex,:) = deadReckonedPosition;
    deadReckonedVelocityENU(sampleIndex,:) = deadReckonedVelocity;
    deadReckonedEuler(sampleIndex,:) = attitudeToEulerZYX(deadReckonedAttitude);
    lioENU(sampleIndex,:) = lioState(1:3).';
    lioVelocityENU(sampleIndex,:) = lioState(4:6).';
    lioPositionStdENU(sampleIndex,:) = sqrt(diag(lioCovariance(1:3,1:3))).';
    lidarMatchedLandmarkCount(sampleIndex) = matchedCount;
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

    if imuParams.accelerometerIncludesGravity
        trueAccelerationBody = (currentAttitude.' * (accelerationENU - imuParams.gravityENU).').';
    else
        trueAccelerationBody = (currentAttitude.' * accelerationENU.').';
    end

    imuMeasurement.angularVelocityBody = trueAngularVelocityBody + imuParams.gyroBiasBody + ...
        imuParams.gyroNoiseStd .* randn(1, 3);
    imuMeasurement.accelerationBody = trueAccelerationBody + imuParams.accelBiasBody + ...
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
