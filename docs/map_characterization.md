# AutoDRIVE MAP characterization

The [2026 RoboRacer technical guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/#13-vehicle) describes a two-piece cubic tire-friction spline. Its listed lateral peak is at slip 0.01 with force 1.00, followed by an asymptote at slip 0.10 with force 0.50. These are **not** Pacejka B/C/D/E coefficients. The guide also lists 0.324 m wheelbase, 3.906 kg mass, center of mass x=0.15532 m, steering limit 0.5236 rad, and steering rate 3.2 rad/s. Confirm the installed simulator build before copying these numbers into configuration; the repo currently uses 0.330 m wheelbase measured from its vehicle frame transforms and a provisional Pacejka LUT.

## Reproduce

Connect AutoDRIVE to the running ROS bridge on `127.0.0.1:4567` in Autonomous mode. In the ROS container, from `/root/ws`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/characterize_map.py record --track map_trial --scale 0.5 --laps 2
python3 scripts/characterize_map.py analyze log/map_characterization_<timestamp>.csv
```

The recorder resets the car, launches MAP, collects simulator body velocity/yaw rate, wheel odometry, IMU, actual steering feedback, `/drive`, and lap/collision counters, then stops at the requested lap count or first collision. It writes CSV and a launch log under `log/`. `--laps 2` includes a complete flying lap because the first count can occur shortly after the car starts from its reset pose. Ground-truth topics are used **only** for offline calibration; the controller continues to use `/odom`. The analysis uses samples with fresh paired data, speed in LUT coverage, small speed and steering changes, and nontrivial steering. Its observed lateral acceleration is `d(v_y)/dt + v_x*yaw_rate`; the model value is interpolated from the LUT at the simulator's reported steering angle and ground-truth forward speed. It does not fit or overwrite the model.

## Observations on `map_trial`

| Speed scale | Result | Paired / steady samples | Median absolute lateral acceleration error | 95th percentile absolute wheel versus ground-truth speed error |
|---|---|---:|---:|---:|
| 0.5 | Two lap-count increments, no collision | 639 / 112 | 0.101 m/s² | 0.883 m/s |
| 0.6 | Two lap-count increments, no collision | 747 / 118 | 0.095 m/s² | 1.148 m/s |

The median reported steering divided by commanded steering was 0.995 in both runs. `V1 Steering` is already an angle in radians; the extra multiplication by steering lock in `wheel_odom.py` was wrong and has been removed. The live bridge process started before this code change, so restart it when convenient after disconnecting AutoDRIVE first. It normally uses IMU yaw rate, so its incorrect steering conversion affected the kinematic fallback, not the measured-speed signal used for these runs.

The median wheel-speed error was near zero (-0.046 and -0.030 m/s), but its tails were large. Peak wheel speeds were 4.04 and 5.05 m/s while peak ground-truth body speeds were 3.52 and 4.37 m/s. The peaks were not necessarily simultaneous; do not interpret their differences as instantaneous slip. Check time-aligned wheel/ground-truth errors during acceleration and cornering before changing controller speed inputs. The LUT agrees reasonably in the observed steady region, which reached about 4 m/s and 4.6 m/s² lateral acceleration. It has **not** been validated near the guide's tire peak, the model's stability boundary, or speeds near the 7 m/s LUT ceiling. The simulator's friction spline and simplified single-track/Pacejka model remain different models, even if their effective response agrees over this limited range.
