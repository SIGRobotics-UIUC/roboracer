# Hairpin timing and recovery fix

Goal: eliminate the recorded failure in which a host-clock step caused false low wheel
speed and a large localization heading error, followed by wrong-way full-lock steering.
See [hairpin_failure.md](hairpin_failure.md) for the recorded evidence and offline queries.

## Implementation plan and current state

1. Implement a shared monotonic-derived transport clock for the simulator stack: done.
2. Gate MAP recovery on clock/heading consistency and distinct fresh pose/odom updates: done.
3. Retain steering history and enforce monotonic slew limiting through safety stops: done.
4. Verify with clock-step, recovery, ROS integration and process-lifecycle tests: passing
   focused checks; full checks and live results are recorded below when available.
5. Run four identical MAP repeats at speed scale 0.5 and steering limit 3.2 rad/s. Target:
   eight completed laps, no collision or timing-induced wrong-way steering, and no ROS
   clock steps even while host wall time continues to step. Live validation pending
   simulator reconnection after the bridge restart.

## Shared clock

`autodrive_bridge.steady_time.SteadyEpoch` samples wall time only once for readable epoch
stamps, then advances using monotonic nanoseconds. `fast_bridge` publishes this as `/clock`
from a steady-clock timer at 100 Hz and immediately before sensor frames. Writes from the
timer and websocket threads are serialized to prevent older clock values being published
out of order. Sensors use exactly this clock. The timer does not depend on ROS time, so
clock publication continues without incoming frames and bootstraps when ROS time is zero.

The bridge, adapter, wheel odometry, localization/mapping and control launches default to
`use_sim_time:=true`. Race launch propagates the setting into its included launches. This
is a monotonic **transport clock**, not simulator physics time: it fixes host-wall-clock
steps but does not remove actual long scheduling stalls or the earlier simulation/wall
rate discrepancy. Ground truth and competition-restricted lap counters are not used as
the production clock or as controller inputs.

For hardware or legacy wall-clock operation, pass `use_sim_time:=false` consistently to
all launches. The mapping/racing wrapper scripts accept `USE_SIM_TIME=false`:

```bash
USE_SIM_TIME=false scripts/map_track.sh TRACK --wall-follow
USE_SIM_TIME=false scripts/race.sh TRACK controller:=map
ros2 launch control race.launch.py track:=TRACK controller:=map use_sim_time:=false
```

Direct `ros2 run` nodes do not inherit launch parameters. In simulator operation, pass
`--ros-args -p use_sim_time:=true` (in addition to their normal parameters). ROS bag record
should use `--use-sim-time` and include `/clock`. The experiment runner now does both.
Do not run two clock publishers or replay a bag's `/clock` alongside the live bridge.
Changing bridge clock mode/restarting it also requires restarting time-dependent consumers.

## MAP recovery

`control.input_health.InputHealth` compares increments of the node's ROS clock with
monotonic time. A disagreement above 0.25 s latches recovery. The heading reference is
propagated from fresh measured odometry yaw rate on monotonic time; a new pose more than
0.25 rad from that prediction is rejected. Repeating a stable but wrong heading does not
make it the new reference. Startup also establishes a heading reference before re-arming.

After a fault, re-arming needs at least 0.25 s of consistent inputs and three distinct
updates from **both** pose and odometry, as well as the existing frame/stamp/speed checks.
Explicit simulator-reset events clear the trajectory and input references; the localization
gate continues withholding poses until relocalized. Long real sensor outages still stop
on stale inputs. Recovery is conservative: an inconsistent estimate can keep the car
stopped rather than automatically accepting a new heading. This is not a guarantee of
accurate localization after arbitrary outages.

A safety stop requests zero speed while retaining the last steering request. It no longer
clears steering-rate history. Slew limits use monotonic elapsed time capped at 0.1 s per
update, so a clock step or long callback delay cannot authorize a full-lock reversal.
The simulator actuator still applies its own limits. No raceline, target speed scale,
LUT, tire coefficients or normal steering gains were retuned.

## Checks completed before live validation

- Package build through `control` and `autodrive_bridge` succeeded.
- Focused math/odometry/bridge/recovery checks: 24 passed in the first run.
- ROS MAP command/recovery integration: 2 passed after adapting expectations to the
  intended recovery hold and retained steering.
- Shared clock publication + launch/clean-shutdown checks: 5 passed, including publishing
  `/clock` with no sensor frames and a simulated 2.5-second wall-clock step.
- Further recovery checks cover one-stream-only updates and initial heading jumps.

A ROS Humble compatibility error (`qos_profile_clock` unavailable) was caught by the launch
regression tests and corrected to an explicit best-effort/depth-1 clock QoS before restarting
the live bridge. Earlier failed runs and reports remain unchanged.

The updated live bridge was restarted with asynchronous frame tracing to
`log/hairpin_fixed_clock_trace.jsonl`. Its adapter and wheel odometry parameters were
verified as `use_sim_time=true`. The initial live attempt in
`log/map_hairpin_fixed_20261031` failed preflight because the simulator did not reconnect;
no driving trial started. Do not interpret that directory as validation results.

## Regenerate validation

Inside the sourced ROS container, with the updated bridge connected to Porto:

```bash
python3 scripts/map_control_experiment.py --repeat-current 4 --output log/NEW_FIXED_VALIDATION
python3 scripts/compare_map_experiment.py log/NEW_FIXED_VALIDATION
```

The manifest records fixed input hashes, `use_sim_time`, controller arguments and all
attempts. Reports distinguish ROS-clock discontinuities from host wall-clock steps; the
latter may continue while the shared ROS clock remains stable. Failed attempts remain
visible. Live before/after comparisons must account for the newly added recovery hold
and corrected timing, rather than treating this as a steering-gain experiment.
