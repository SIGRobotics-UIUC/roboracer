# Overtaking roadmap: what has to exist before FSDP

Status: proposal, October 2026. Owner: planning and control.

## Why FSDP does not drop in

[FSDP](https://github.com/ZJU-DDRX/FSDP) ([paper](https://arxiv.org/abs/2503.06075)) is a
data-driven overtaking trajectory planner: a sparse Gaussian process learns the opponent's line
online, a bi-level QP plans the evasion, and differential flatness recovers the states and
controls from the planned geometry. It reports 8.93% higher overtake success than Predictive
Spliner at 74% less compute.

It cannot be merged into this stack as it stands, for four reasons, in increasing order of
severity.

**It is ROS 1.** Its `package.xml` declares `<buildtool_depend>catkin</buildtool_depend>` and it
depends on `message_generation`, `dynamic_reconfigure`, `rospy` and `tf`. Every package here is
`ament_python` on Humble. `catkin build` does not exist in our container, and
`dynamic_reconfigure` has no ROS 2 equivalent; it would become parameter callbacks.

**It is a ForzaETH race_stack plugin, not a standalone package.** It imports `f110_msgs` for
`Wpnt`, `WpntArray`, `Obstacle`, `ObstacleArray`, `OpponentTrajectory`, `OppWpnt` and
`ProjOppTraj`, calls `frenet_conversion.srv.Frenet2GlobArr` as a ROS 1 service, and pulls the
`ccma` smoothing library from race_stack's `f110_utils`. Adopting FSDP means adopting a
meaningful slice of race_stack with it.

**We had no Frenet frame.** FSDP is written entirely in Frenet coordinates and leans on
`frenet_converter` throughout. Before this proposal there were zero references to Frenet
anywhere in `ws/src`. This is not a defect in the stack; a Cartesian raceline with pure pursuit
is a standard and perfectly good F1TENTH design. It simply was not built for local planning yet.

**There is no opponent.** FSDP plans overtakes, which presupposes something to overtake. The
only obstacle-adjacent code we have is LiDAR gap-following in `follow_the_gap_node.py`, which is
reactive and does not track anything. Nothing publishes an obstacle array, and the simulator is
configured for a single car (`/autodrive/roboracer_1/*`). A perfect port would run and have
nothing to plan against.

## FSDP and Predictive Spliner are the same codebase

FSDP is a fork of ForzaETH's Predictive Spliner, not an independent
implementation. Diffing the two repos file by file:

| File | Predictive Spliner | FSDP | Differing lines |
|---|---|---|---|
| `collision_prediction.py` | 303 | 303 | **0** |
| `opponent_trajectory.py` | 492 | 492 | **0** |
| `predictor_opponent_trajectory.py` | 271 | 271 | **0** |
| `update_waypoints.py` | 135 | 135 | **0** |
| `dynamic_collision_server.py` | 24 | 24 | 2 |
| `dynamic_sqp_server.py` | 21 | 22 | 4 |
| `gaussian_process_opp_traj.py` | 466 | 1158 | 767 |
| `sqp_avoidance_node.py` | 549 | 1383 | 1065 |

Around 1,250 lines are byte-identical. The entire algorithmic difference lives
in two files: FSDP swaps the dense GP for a sparse one and replaces the SQP
avoidance node with a bi-level QP plus differential flatness.

This decides how the work is split. Porting Predictive Spliner on one branch and
FSDP on another means rewriting the same 1,250 lines of ROS 1 twice, and
rewriting them differently, after which any comparison between the branches
measures two porting styles as much as it measures two planners. The shared
infrastructure should be ported once into a common package that both branches
import, leaving them to diverge only where the papers diverge. That is also how
the FSDP paper runs its own comparison, which is what makes its numbers mean
anything.

## The ladder

Each rung is useful on its own, which is the point. None of this is wasted if FSDP never lands.

**1. Frenet conversion layer.** Done, see `ws/src/planning/planning/frenet.py`. Converts between
map-frame `(x, y)` and `(s, d)` against the centerline or raceline, with signed lap distance that
behaves correctly across the start line. Needed by everything below.

**2. The opponent interface, and a synthetic opponent behind it.** Done. `f1t_msgs/Obstacle`
and `ObstacleArray` describe obstacles entirely in Frenet, with field names taken from
ForzaETH's `f110_msgs` so their planner code ports unedited. `frenet_state_node` publishes our
own car on `/car_state/odom_frenet`, which every planner subscribes to and nothing previously
produced. `fake_opponent_node` publishes a synthetic opponent on `/perception/obstacles`, in
static, constant-speed, weaving or replay modes, behind a sensor model with configurable range,
rear blind spot, dropout, noise and latency.

The point of the sensor model deserves stating plainly, because publishing ground truth is the
tempting shortcut. The GP inside both planners exists to turn noisy, intermittent sightings into
a predicted trajectory; given clean data it has nothing to do, and a comparison between FSDP's
sparse GP and Predictive Spliner's dense one would measure nothing at all. The noise is the
independent variable, not decoration. The models are in `opponent_models.py` with no ROS imports,
and are unit tested.

**3. Real opponent detection and tracking.** Segment the LiDAR scan, reject returns that belong
to known track boundaries using the map, cluster what remains, and track clusters across frames
to get velocity rather than just position. This is the largest single piece and the one with the
least existing code to build on. Because rung 2 fixed the interface first, it can be built and
swapped in without touching anything downstream.

**4. Multi-agent simulation.** Run two vehicles in AutoDRIVE under separate topic namespaces, one
on a baseline line and one running our stack, so detection and overtaking can be tested at all.
Needs a second bridge instance and namespacing throughout.

**5. A baseline local planner.** Something simple that avoids a tracked obstacle by deviating in
`d` and rejoining the raceline. Not competitive, but it closes the loop end to end and gives a
reference for whether anything fancier is actually better.

**6. FSDP, or the parts of it worth having.** Only meaningful once 1 through 5 exist.

## What is reusable from FSDP today

Seven files have no ROS dependency at all and would run as plain Python once a Frenet layer
exists. Roughly 2,100 lines:

| File | Lines | What it does |
|---|---|---|
| `gp/Sparse_GP_data_selection.py` | 465 | Filters and prunes GP training data by range, confidence and variance; k-means clustering around inducing points keeps the set bounded as laps accumulate |
| `mpc/mpc_tracking_controller_ca.py` | 327 | Linear MPC with QP assembly pushed into the C++ extension |
| `mpc/mpc_tracking_controller_relaxed_ca.py` | 252 | The same with slack variables, so the QP stays solvable outside the constraint set |
| `common/qp_fit.py` | 193 | Fits a polynomial through waypoints with fixed end velocities, as a QP |
| `gp/Sparse_GP_function.py` | 187 | The sparse GP itself, gpytorch with an inducing-point kernel |
| `mpc/mpc_tracking_controller.py` | 179 | Pure numpy/OSQP version of the same MPC |
| `mpc/linear_model.py` | 105 | CasADi kinematic bicycle model in Frenet coordinates, with Jacobians |

`common/differential_flatness.py` (407 lines) is the conceptually interesting one, fitting a
quintic through evasion points and differentiating it analytically to recover orientation, speed,
acceleration and steering angle. It imports `FrenetConverter`, so it needs a shim against our own
before it will run.

Against this, roughly 5,100 lines are thoroughly ROS 1 bound and would need rewriting rather than
porting. `sqp_avoidance_node.py` alone is 1,383 lines with 54 rospy references.

Do not vendor the reusable files yet. Without rung 1 in place and something to test against, they
are dead code that nobody can run.

## Notes for whoever attempts the port

FSDP's `setup.py` compiles its pybind11 extension with `-march=native`, which produces binaries
tied to the compiling machine's processor. Our image targets both amd64 and arm64, so that flag
has to go or the extension has to be built per-architecture inside the image.

The ML dependency set is heavy: torch, gpytorch, scikit-learn, osqp, and Eigen for the extension.
That is a real addition to image build time and size.

If anyone wants to see FSDP running before any of this, the honest way is to stand up ForzaETH's
ROS 1 race_stack separately and run it there as upstream intends, as a reference implementation
rather than an integration.
