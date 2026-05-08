# dead-reckoning

This repository contains a MATLAB example of time-based 3-D dead reckoning for a UAV.

## What the script does

`dead_reckoning.m` now treats the route as a timestamped trajectory rather than a list of untimed 2-D waypoints:

- input trajectory columns are `[time_s, latitude_deg, longitude_deg, altitude_m]`;
- latitude, longitude, and altitude are converted into a local ENU (`east`, `north`, `up`) frame;
- the timed trajectory is sampled at a fixed update interval to mimic a real-time onboard loop;
- velocity measurements are integrated over time to estimate the dead-reckoned 3-D position;
- results are converted back to latitude, longitude, and altitude for display and map plotting.

The implementation is self-contained and does not require MATLAB's `dreckon` function.  For short UAV routes it uses a spherical-earth local tangent approximation.  If you need survey-grade accuracy over larger areas, replace the helper conversions with Mapping Toolbox functions such as `geodetic2enu` and `enu2geodetic`.

## Run

Open MATLAB in this directory and run:

```matlab
dead_reckoning
```

The script prints the first time-stamped dead-reckoned samples and opens:

1. a 3-D local ENU plot comparing the reference trajectory with the dead-reckoned trajectory;
2. a geographic map view where sample colour represents altitude.
