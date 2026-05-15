# dead-reckoning

This repository contains a MATLAB example of curved, time-stepped 3-D IMU and lidar-inertial odometry (LIO) dead reckoning for a UAV.

## What the script does

`dead_reckoning.m` treats the route as a timestamped trajectory rather than a list of untimed 2-D waypoints:

- input trajectory control-point columns are `[time_s, latitude_deg, longitude_deg, altitude_m]`;
- latitude, longitude, and altitude are converted into a local ENU (`east`, `north`, `up`) frame;
- timed control points are connected with a cubic Hermite curve, so the UAV follows a smooth trajectory rather than straight line segments;
- the script checks that the curved trajectory evaluates exactly at the predefined timed waypoints;
- the simulated flight advances one `sampleTime` at a time in `flyImuDeadReckoningLoop`, instead of generating and integrating the whole route in one vectorized step;
- the reference trajectory supplies position, velocity, and acceleration for a simple IMU sensor model;
- gyroscope measurements propagate body-to-ENU attitude, and gravity-compensated body-frame accelerometer measurements propagate velocity and position;
- predefined 3-D landmarks are generated around the trajectory and used for lidar scan matching;
- a lidar-inertial EKF keeps position, velocity, and attitude in the state and fuses landmark residuals directly with IMU propagation for comparison against pure IMU dead reckoning;
- the UAV flight is replayed as a 3-D animation that advances according to the trajectory timestamps;
- results are converted back to latitude, longitude, and altitude for display and map plotting.

The implementation is self-contained and does not require MATLAB's `dreckon` function.  For short UAV routes it uses a spherical-earth local tangent approximation.  If you need survey-grade accuracy over larger areas, replace the helper conversions with Mapping Toolbox functions such as `geodetic2enu` and `enu2geodetic`.

## IMU propagation model

The script uses a strapdown-style inertial update at each sample:

1. simulate body-frame gyroscope angular velocity `[p, q, r]` in rad/s from the reference attitude change;
2. simulate body-frame accelerometer acceleration `[a_x, a_y, a_z]` in m/s²; by default this is gravity-compensated linear acceleration, so it does not include a constant 1-g offset;
3. add configurable gyroscope and accelerometer bias/noise;
4. integrate gyroscope angular velocity to update attitude;
5. rotate the accelerometer measurement back into ENU and integrate acceleration into velocity and position. If `accelerometerIncludesGravity = true`, the script treats the measurement as raw specific force and restores gravity during propagation.

Tune the IMU error model near the top of `dead_reckoning.m`:

```matlab
imuParams.gyroBiasBody = deg2rad([0.001, -0.0015, 0.002]);
imuParams.gyroNoiseStd = deg2rad(0.002);
imuParams.accelBiasBody = [0.002, -0.0015, 0.003];
imuParams.accelNoiseStd = 0.005;
imuParams.accelerometerIncludesGravity = false;
```


## Lidar-inertial odometry comparison

The script generates deterministic 3-D landmarks around the reference route with `generatePredefinedLandmarks`.  During each sample, `updateLioWithLandmarkResiduals` simulates a lidar scan by observing visible landmarks in the body frame and updates the EKF with each landmark residual directly, instead of first converting the scan to a matched position.

`propagateLioState` predicts a 9-state error-state EKF `[east, north, up, v_e, v_n, v_u, attitude_error]` with IMU gyro and accelerometer measurements.  When enough landmark correspondences are visible, `updateLioWithLandmarkResiduals` applies raw landmark residual updates that correct both position/velocity and attitude, so the plots compare:

- the curved reference trajectory;
- pure IMU propagation;
- lidar-inertial odometry (LIO) EKF fusion.

Tune the LIO defaults near the top of `dead_reckoning.m`:

```matlab
lioParams.lidarRange = 180;
lioParams.lidarNoiseStd = 0.35;
lioParams.minLandmarksForUpdate = 3;
lioParams.accelProcessNoiseStd = 0.08;
lioParams.gyroProcessNoiseStd = deg2rad(0.01);
```

## Animation and real-time playback

After the dead-reckoning loop, the script opens an animated 3-D flight window.  The reference UAV marker, pure IMU estimate, LIO EKF estimate, and predefined landmarks are shown sample by sample, while animated trails show where each estimate has already flown.

The animation uses `animationSpeedup` to control playback speed:

```matlab
animationSpeedup = 12;  % faster demo playback
animationSpeedup = 1;   % true timestamp playback
```

By default, the simulation loop itself advances in simulation time without waiting, so the script computes quickly before replaying the animation.  To make the simulation loop wait like a real onboard loop too, set:

```matlab
playbackInRealTime = true;
```

With that option enabled, each simulation update calls `pause(dt)` before the next sample is processed.

## Run

Open MATLAB in this directory and run:

```matlab
dead_reckoning
```

The script prints the waypoint consistency check, the first time-stamped IMU/LIO dead-reckoned samples, and a final pure-IMU-vs-LIO error comparison, then opens:

1. an animated 3-D local ENU view of the UAV flying over time;
2. optional static summary plots comparing the curved reference trajectory with pure IMU and LIO EKF trajectories;
3. a geographic map view where sample colour represents altitude;
4. a time-history plot comparing pure IMU and LIO 3-D position error against the reference trajectory;
5. an IMU measurement plot showing gyroscope angular velocity and body-frame accelerometer output.
