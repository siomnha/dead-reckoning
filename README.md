# dead-reckoning

This repository contains a MATLAB example of curved, time-stepped 3-D dead reckoning for a UAV.

## What the script does

`dead_reckoning.m` treats the route as a timestamped trajectory rather than a list of untimed 2-D waypoints:

- input trajectory control-point columns are `[time_s, latitude_deg, longitude_deg, altitude_m]`;
- latitude, longitude, and altitude are converted into a local ENU (`east`, `north`, `up`) frame;
- timed control points are connected with a cubic Hermite curve, so the UAV follows a smooth trajectory rather than straight line segments;
- the simulated flight advances one `sampleTime` at a time in `flyDeadReckoningLoop`, instead of generating and integrating the whole route in one vectorized step;
- velocity measurements are integrated over time to estimate the dead-reckoned 3-D position;
- results are converted back to latitude, longitude, and altitude for display and map plotting.

The implementation is self-contained and does not require MATLAB's `dreckon` function.  For short UAV routes it uses a spherical-earth local tangent approximation.  If you need survey-grade accuracy over larger areas, replace the helper conversions with Mapping Toolbox functions such as `geodetic2enu` and `enu2geodetic`.

## Real-time playback

By default, the loop advances in simulation time without waiting, so the script runs quickly.  To make the demo wait like a real onboard loop, set:

```matlab
playbackInRealTime = true;
```

With that option enabled, each update calls `pause(dt)` before the next sample is processed.

## Run

Open MATLAB in this directory and run:

```matlab
dead_reckoning
```

The script prints the first time-stamped dead-reckoned samples and opens:

1. a 3-D local ENU plot comparing the curved reference trajectory with the dead-reckoned trajectory;
2. a geographic map view where sample colour represents altitude.
