# Localization benchmark: Cartographer vs MCL on Porto (2026-11-08)

Both localizers race the same track in the same simulator session, and their pose is scored against the
simulator's ground truth. Open **[report.html](report.html)** in a browser for the charts and every run.

The team had already chosen MCL for racing; this is the measured case for that choice.

## Result (clean session, 3 runs × 60 s per localizer and speed)

| speed_scale | Crashes: Cartographer / MCL | Mean error (cm) | Pose jitter (mm) | Worst error (cm) |
|---|---|---|---|---|
| 0.5 | 0 / 0 | 7.5 / 7.6 | 20 / 4 | 59 / 24 |
| 0.7 | **4 / 0** | 7.2 / 8.0 | 20 / 8 | 34 / 20 |
| 1.0 | **11 / 1** | 7.5 / 9.4 | 31 / 15 | 72 / 65 |

Cartographer includes the stall fix ([#25](https://github.com/SIGRobotics-UIUC/roboracer/pull/25), below).
Values are Cartographer / MCL.

- **Mean accuracy is a tie** (7–9 cm for both, about half of it the map's own placement error).
- **MCL is far safer at speed:** 1 crash in 9 runs against Cartographer's 15.
- **The difference is jitter, not accuracy.** Cartographer's pose wobbles 2–5× more from one moment to the next
  (*jitter*: the RMS change of the error over 50 ms). Pure pursuit steers on that wobble: 14 of Cartographer's 15
  crashes were at the same two corners, around (5.4, −2.6) and (1.4, 0.4) in the map frame, with only 0.1–0.2 m
  of position error and no simulator stall. MCL's one crash followed two simulator stalls.

## Stall fix (Cartographer)

The simulator sometimes freezes for 0.1–0.8 s on a loaded machine. `/clock` keeps running, and Cartographer
extrapolates `/tracked_pose` to the current time, so its pose ran 0.8–3 m ahead of a car that stood still, then
snapped back. [#25](https://github.com/SIGRobotics-UIUC/roboracer/pull/25) makes `localization_gate` hold back
poses stamped more than 60 ms after the newest `/odom` (normally the lead is 6 ms, 99 % under 30 ms).
[stall_fix_comparison.html](stall_fix_comparison.html) has the runs before and after (speed 0.7 and 1.0, 4 runs each):

| Cartographer | before | stall fix | odom composition (rejected) |
|---|---|---|---|
| Pose jumps | 14 | 1 | 4 |
| Worst error | 0.8–3.2 m | 0.19–0.91 m | 0.3–1.7 m |
| Jitter | 41–110 mm | 23–27 mm | 43–69 mm |
| Crashes | 10 | 4 | 7 |

The fix removes the jumps; the remaining crashes come from the jitter (above). Composing Cartographer's
`map → odom` with the wheel odometry (as MCL builds its pose) was tried and was worse, because the `map → odom`
correction itself is noisy.

## Method

- **Simulator:** AutoDRIVE multi-track build `f1tenth_multitrack_v3`, track Porto, restarted before the session.
  Sensor frames 40–60/s, median 52 ([sim_fps.csv](sim_fps.csv)).
- **Stack:** `scripts/race.sh porto localizer:=<cartographer|mcl> speed_scale:=<s>`, pure pursuit with adaptive
  lookahead, the Porto map and minimum-time raceline from 2026-10-29.
- **Runs:** for each speed, 3 repeats of 60 s; Cartographer and MCL alternate so slow drifts in the simulator affect both.
- **Scoring:** `/localized_pose` against `/ground_truth/pose` (rear axle, both in the map frame via
  `map_to_world.yaml`), interpolated to each pose's stamp. Ground truth is only used for scoring.
- Machine and code versions: [provenance.json](provenance.json).

## Files

| File | Content |
|---|---|
| `report.html` | clean session: charts by speed, every run (self-contained, open in a browser) |
| `stall_fix_comparison.html` | earlier session: Cartographer before / with the stall fix / odom composition, and MCL |
| `summary.json` | clean session: per-run scores and per-localizer, per-speed aggregates |
| `raw_runs.tar.xz` | every run of both sessions: recorded poses, ground truth, lap and collision counts (scan rows dropped; their rate is in each `summary.json`) |
| `sim_fps.csv` | simulator frame rate every 10 s during the clean session |
| `provenance.json` | code commit, simulator build, machine, raceline |

## Reproduce

In the container, with the bridge running and the simulator connected on Porto in Autonomous mode:

```bash
python3 scripts/localization_benchmark.py run --track porto --duration 60 --repeats 3 --speed-scale 0.7
python3 scripts/localization_report.py log/localization_benchmark/<folder> ... -o report.html
# re-score archived runs
tar -xJf raw_runs.tar.xz -C /tmp/runs && python3 scripts/localization_benchmark.py analyze /tmp/runs/20261108-233959
```

## Caveats

- One track, 3 runs per setting: treat the crash counts as a strong signal, not exact rates.
- Simulator stalls come from the host machine; the real car's localization runs under different conditions.
- Crashes are the simulator's collision count; after each, the car respawns and the localizer reinitializes.
