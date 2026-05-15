# dead-reckoning

This repository contains a MATLAB example of curved, time-stepped 3-D IMU dead reckoning for a UAV.

## What the script does

`dead_reckoning.m` treats the route as a timestamped trajectory rather than a list of untimed 2-D waypoints:

- input trajectory control-point columns are `[time_s, latitude_deg, longitude_deg, altitude_m]`;
- latitude, longitude, and altitude are converted into a local ENU (`east`, `north`, `up`) frame;
- timed control points are connected with a cubic Hermite curve, so the UAV follows a smooth trajectory rather than straight line segments;
- the simulated flight advances one `sampleTime` at a time in `flyImuDeadReckoningLoop`, instead of generating and integrating the whole route in one vectorized step;
- the reference trajectory supplies position, velocity, and acceleration for a simple IMU sensor model;
- gyroscope measurements propagate body-to-ENU attitude, and accelerometer specific-force measurements propagate velocity and position;
- the UAV flight is replayed as a 3-D animation that advances according to the trajectory timestamps;
- the UAV flight is replayed as a 3-D animation that advances according to the trajectory timestamps;
- results are converted back to latitude, longitude, and altitude for display and map plotting.

The implementation is self-contained and does not require MATLAB's `dreckon` function.  For short UAV routes it uses a spherical-earth local tangent approximation.  If you need survey-grade accuracy over larger areas, replace the helper conversions with Mapping Toolbox functions such as `geodetic2enu` and `enu2geodetic`.

## IMU propagation model

The script uses a strapdown-style inertial update at each sample:

1. simulate body-frame gyroscope angular velocity `[p, q, r]` in rad/s from the reference attitude change;
2. simulate body-frame accelerometer specific force `[f_x, f_y, f_z]` in m/s² from reference acceleration minus gravity;
3. add configurable gyroscope and accelerometer bias/noise;
4. integrate gyroscope angular velocity to update attitude;
5. rotate accelerometer specific force back into ENU, restore gravity, and integrate acceleration into velocity and position.

Tune the IMU error model near the top of `dead_reckoning.m`:

```matlab
imuParams.gyroBiasBody = deg2rad([0.01, -0.015, 0.02]);
imuParams.gyroNoiseStd = deg2rad(0.02);
imuParams.accelBiasBody = [0.015, -0.010, 0.020];
imuParams.accelNoiseStd = 0.025;
```

## Animation and real-time playback

After the dead-reckoning loop, the script opens an animated 3-D flight window.  The reference UAV marker and IMU dead-reckoned estimate move along the trajectory sample by sample, while animated trails show where each has already flown.
## Animation and real-time playback

After the dead-reckoning loop, the script opens an animated 3-D flight window.  The reference UAV marker and dead-reckoned estimate move along the trajectory sample by sample, while animated trails show where each has already flown.

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
With that option enabled, each simulation update calls `pause(dt)` before the next sample is processed.


## Run

Open MATLAB in this directory and run:

```matlab
dead_reckoning
```

The script prints the first time-stamped IMU dead-reckoned samples and opens:

1. an animated 3-D local ENU view of the UAV flying over time;
2. optional static summary plots comparing the curved reference trajectory with the IMU dead-reckoned trajectory;
3. a geographic map view where sample colour represents altitude;
4. an IMU measurement plot showing gyroscope angular velocity and accelerometer specific force.
1. an animated 3-D local ENU view of the UAV flying over time;
2. optional static summary plots comparing the curved reference trajectory with the dead-reckoned trajectory;
3. a geographic map view where sample colour represents altitude.

