# Direct check of the lateral slip coordinate — 2026-10-31

Historical investigation and implementation record. Current conclusions, driving
results and regression checks are consolidated in [VALIDATION.md](VALIDATION.md).

## Installed simulator scene settings

After the driving-data check, we inspected the **running Windows practice build's**
Unity `Data/level0` scene read-only. Its SHA-256 is
`5edbd1bacc15647746e6e89d9bcf1bb9e202d62d47e76ae352497be1404b3f8d`.
All four enabled RoboRacer `WheelCollider` components have the same serialized settings;
the RoboRacer `VehicleController` component references those four colliders.

| Direction | Peak slip | Stored peak value | Asymptote slip | Stored asymptote value | Stiffness | Effective peak / asymptote values |
|---|---:|---:|---:|---:|---:|---:|
| Forward | 0.15 | 0.90 | 0.25 | 0.58 | 0.80 | 0.72 / 0.464 |
| Sideways | 0.01 | 1.00 | 0.10 | 0.50 | 1.00 | 1.00 / 0.50 |

The effective values match the 2026 guide exactly. Unity documents `stiffness` as a
multiplier of the extremum and asymptote values. The wheel radius is `0.059 m` in
all four scene components. The exact extraction is saved in
`docs/autodrive_wheel_settings_20261031.json` and is reproducible with
`scripts/inspect_autodrive_wheels.py` and UnityPy. These are **serialized defaults**;
we have not established whether runtime code changes friction properties, or how
Unity's internal contact-patch slip compares with our wheel-centre estimate. The
current bridge payload has no per-wheel internal slip or separate tire-force fields.

## Driving-data check

The installed Windows AutoDRIVE practice simulator was connected to the current ROS
bridge. We recorded one new `map_trial` MAP run at speed scale 0.5, with two lap-count
increments and no collision. Raw telemetry is
`ws/log/friction_curve_map_trial_05_20261031.csv` (git-ignored). We also analyzed two
earlier `map_trial` recordings at scales 0.5 and 0.6. This driving-data check preceded
the scene extraction above; no new tire model was used.

The 2026 [AutoDRIVE guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/#132-vehicle-dynamics)
lists the lateral friction curve's extremum at slip `0.01` and asymptote at `0.10`.
`check_friction_curve.py` estimates wheel-centre lateral slip as `|tan(alpha)|` using
body velocity, yaw rate, steering feedback, wheel geometry, and the measured centre of
velocity. It estimates body lateral acceleration as `d(v_y)/dt + v_x * yaw_rate`.
Intervals are screened for relatively steady speed and steering. These are **not**
the simulator's internal contact-patch slip or tire forces; total body acceleration
cannot isolate the four individual tire-force curves.

| Recording | Selected samples | Front slip P95 | At/above 0.01 | Max selected lateral acceleration | Current LUT median absolute error | Kinematic bicycle median absolute error |
|---|---:|---:|---:|---:|---:|---:|
| New map_trial 0.5 | 321 | 0.00611 | 0 | 2.46 m/s² | 0.087 m/s² | 0.086 m/s² |
| Earlier map_trial 0.5 | 113 | 0.00756 | 1 isolated sample | 4.57 m/s² | 0.089 m/s² | 0.096 m/s² |
| Earlier map_trial 0.6 | 120 | 0.00983 | 6 | 3.95 m/s² | 0.095 m/s² | 0.103 m/s² |

In the earlier 0.6 recording, the six selected samples at estimated slip
`0.0101–0.0120` had lateral accelerations of `3.03–3.95 m/s²`. The corresponding
current-LUT absolute errors were `0.06–0.36 m/s²`. The median absolute acceleration
increases from `2.74 m/s²` in the six samples at slip `0.008–0.010` to `3.44 m/s²`
in the six at `0.010–0.015`. That is **no observed downturn at the guide coordinate**
in this small, uncontrolled-by-slip sample. It does not refute the guide: estimated
wheel-centre slip may differ from the simulator's internal slip, speed and load vary,
and a tire-force peak cannot be inferred from total acceleration alone.

The new 0.5 run did not reach estimated slip `0.01`; its maximum selected value was
`0.00838`. The single earlier 0.5 sample at `0.0599` is isolated and cannot define
a force curve. The current LUT and a no-slip kinematic bicycle prediction perform
similarly in the observed range. There is no measured basis yet for replacing MAP's
Pacejka LUT with a guide-shaped spline LUT. The installed scene confirms the guide's
configured coordinates, but those coordinates alone do not establish the exact
runtime force law or identify per-wheel tire saturation.

Reproduce from `/root/ws` in the sourced ROS container:

```bash
python3 scripts/check_friction_curve.py \
  log/friction_curve_map_trial_05_20261031.csv \
  log/map_characterization_1793335882.csv \
  log/map_characterization_1793336031.csv \
  --output log/friction_curve_empirical_check_20261031.json
```

The next empirical step, if a peak estimate is needed, is a dedicated constant-speed,
slow steering sweep with known clear space and direct simulator tire-force/slip
telemetry if exposed. Repeating racing laps at scale 0.5 mainly re-samples below the
coordinate. A 0.6 racing lap alone still supplies few steady samples at/above it.
