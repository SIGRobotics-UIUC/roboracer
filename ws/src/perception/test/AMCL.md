KLD particle adaptation
=======================

The ROS node/config enable experimental adaptation by default. The reusable
ParticleFilter constructor defaults to fixed mode for compatibility. In adaptive
mode the initial `particles` count is clamped to min/max. Global recovery uses
`recovery_min_particles` (experimental default 1000), capped at max_particles and
never reducing an already larger population. Global find/verify/relocation,
alternative switches and /initialpose request this floor. Ordinary local Gaussian
re-anchors retain the current population. Fixed mode uses `particles` throughout.
Convergence thresholds, recovery decisions, sensor models, and topics are unchanged.

After rebuilding/sourcing perception (and control for race launch changes):

```bash
ros2 launch perception mcl.launch.py track:=NAME adaptive_particles:=false particles:=1000
ros2 launch perception mcl.launch.py track:=NAME adaptive_particles:=true
ros2 launch control race.launch.py track:=NAME localizer:=mcl adaptive_particles:=false particles:=1000
ros2 launch control race.launch.py track:=NAME localizer:=mcl adaptive_particles:=true
```

Tune bounds, error, confidence and bin widths in installed `config/mcl.yaml`
(edit the source and rebuild). `pf_z` is a probability, not a z score; 0.99
becomes 2.32635. Invalid settings fail construction. These are experimental values.
The race.sh wrapper still requires a pbstream even when selecting MCL.

ESS is compared against 0.5 times the *current* count. If resampling is skipped,
the count remains unchanged. On resampling, draws come from the weighted input
cloud. Each newly occupied (x,y,yaw) bin updates the Wilson-Hilferty requirement;
the algorithm stops once the sampled count meets it, clamped to min/max. For
zero/one bins the minimum applies. Orientation uses ceil(2pi/requested_width)
equal periodic bins, centered on the +/-pi seam. Spatial bins use floor, including
negative coordinates. Output weights are uniform and normalized.

Each output particle is selected by a fresh independent uniform variate through
the source weighted CDF. The source cloud/weights stay fixed while the output bins
control stopping. Independent draws have more sampling variance than stratified
resampling; timings and filter trajectories can change. The KLD formula remains an
asymptotic approximation, can be limited by max_particles, and describes the
represented discrete support rather than true pose accuracy. A small mode never
sampled cannot affect the bound. Adaptation cannot invent lost hypotheses or
replace global recovery.

Repeatable benchmark (inside the ROS container, from /root/ws):

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=src/perception:$PYTHONPATH
python3 src/perception/test/benchmark_mcl.py --seed 1
python3 src/perception/test/benchmark_mcl.py --seed 1 --force-resample
ROS_DOMAIN_ID=42 python3 -m pytest src/perception/test/test_mcl.py src/perception/test/test_mcl_node.py -q
```

Inputs are generated once and replayed identically for both filters. PF timings
include predict, weighting, resampling, estimate and spread; ray marching is
excluded. Scan-match fit timing is separate. Global/local search overhead and
ROS transport are not benchmarked. For node benchmarking, the launch argument
`force_adaptive_resample:=true` bypasses ESS only in adaptive mode, at each eligible
filter update (motion gating still applies).

Before racing, validate recorded sensor bags and AutoDRIVE laps with a new track
or preserved artifacts. Compare several seeds, particle bounds, confidence/bin
settings, recovery frequency, latency tails, error and collisions. Synthetic
tracking and callback tests cannot establish safe simulator laps. No ground-truth
input is added; existing bridge/reset/localization-gate ground-truth review debt
remains outside this change.

Historical stratified-sampler benchmark result (superseded; 2026-11-08, existing ROS Humble Docker image, seed 1,
210 updates, concurrent callback test load; single run, no warm-up):

| Resampling | Mode | Count min/mean/max | Position mean/max m | PF mean/p95 ms | Fit mean ms |
|---|---|---|---|---|---|
| ESS | fixed | 1000/1000/1000 | .0967/.2737 | 3.283/6.378 | .148 |
| ESS | adaptive | 200/201.0/263 | .0912/.2844 | .574/1.085 | .089 |
| forced | fixed | 1000/1000/1000 | .0901/.2363 | 1.907/1.995 | .076 |
| forced | adaptive | 200/201.1/263 | .1018/.2991 | .762/.815 | .076 |

Mean yaw errors were .00444/.00450 rad (ESS fixed/adaptive) and
.00445/.00472 rad (forced fixed/adaptive). Runtime numbers are illustrative;
repeat without concurrent tests, alternate mode order, warm up, and use several
seeds before drawing performance conclusions.

Current independent-sampler multi-seed results and comparison with the historical
sampler are in [docs/mcl_validation.md](../../../../docs/mcl_validation.md).
Recovery floor changes do not alter Gaussian widths, convergence thresholds,
localization gating, or the state machine. The configured floor may be lower than
min_particles (the current population still respects that limit) or higher than
max_particles (it is capped). It must be a positive integer.
