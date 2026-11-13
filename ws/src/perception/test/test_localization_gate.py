"""localization_gate: sim_reset trigger, debounce, constraint counting and the settle gate, world -> map."""
import math

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from perception.localization_gate import INTER_NS, LocalizationGate, count_map_constraints, world_to_map


@pytest.fixture
def node():
    rclpy.init()
    n = LocalizationGate()
    n.tid = 1
    n.min_count = 0
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def markers(inter, same=0):
    def m(ns, k):
        return Marker(ns=ns, points=[Point() for _ in range(2 * k)])
    return MarkerArray(markers=[m(INTER_NS, inter), m('Inter constraints, same trajectory', same)])


def test_count_only_constraints_to_other_trajectories():
    assert count_map_constraints(markers(3, same=7)) == 3
    assert count_map_constraints(MarkerArray()) == 0


def test_world_to_map_inverts_map_to_world():
    origin = (2.0, -1.0, math.pi / 2)   # map x axis = world y axis
    x, y, yaw = world_to_map(origin, 2.0, 1.0, math.pi / 2 + 0.3)
    assert (x, y, yaw) == pytest.approx((2.0, 0.0, 0.3))
    assert world_to_map(origin, *origin) == pytest.approx((0.0, 0.0, 0.0))


def test_sim_reset_subscribed_first(node):
    # fast_bridge publishes <vehicle_ns>/sim_reset; first, so a reset is handled before a pose ready with it
    assert next(node.subscriptions).topic_name == '/autodrive/roboracer_1/sim_reset'


def test_gate_opens_on_message_after_count_rises(node):
    pub = []
    node.pub.publish = pub.append
    node.pose_cb(PoseStamped())
    for n in (0, 0, 1):
        node.constraints_cb(markers(n))
    assert not node.localized   # constraint added; optimization not yet in
    node.constraints_cb(markers(1))
    assert node.localized
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    node.pose_cb(msg)
    assert pub == [msg]


def stamped(cls, t):
    msg = cls()
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    return msg


def test_pose_extrapolated_past_the_odometry_is_held(node):
    # Cartographer extrapolates /tracked_pose to now; in a sim stall no /odom arrives and it runs ahead
    pub = []
    node.pub.publish = pub.append
    node.localized = True
    node.pose_cb(stamped(PoseStamped, 100.0))          # no /odom yet: relayed
    node.odom_cb(stamped(Odometry, 100.00))
    node.pose_cb(stamped(PoseStamped, 100.03))          # normal lead: relayed
    node.pose_cb(stamped(PoseStamped, 100.07))          # 70 ms ahead of the sensors: held
    node.pose_cb(stamped(PoseStamped, 100.40))          # deep in the stall: held
    assert [round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 2) for m in pub] == [100.0, 100.03]
    assert node.held == 2
    node.odom_cb(stamped(Odometry, 100.41))            # the sim resumes
    node.pose_cb(stamped(PoseStamped, 100.42))
    assert len(pub) == 3 and node.held == 0


def test_odometry_is_subscribed(node):
    assert '/odom' in [s.topic_name for s in node.subscriptions]


def test_stale_constraints_do_not_open_gate(node):
    node.min_count = None   # old trajectory not confirmed deleted
    for n in (4, 4, 4):
        node.constraints_cb(markers(n))
    assert not node.localized
    for n in (0, 1, 1):
        node.constraints_cb(markers(n))
    assert node.localized


def test_triggers_close_gate_and_debounce(node):
    node.localized = True
    node.reset_cb(String(data='reset command'))
    assert not node.localized and node.pending == ('reset command', node.pending[1])
    t = node.pending[1]
    assert not node.restart_due(t + 0.5 * node.debounce)
    node.reset_cb(String(data='position jumped 6.8 m'))   # a later event restarts the debounce
    t2 = node.pending[1]
    assert t2 >= t and node.pending[0] == 'position jumped 6.8 m'
    assert not node.restart_due(t2 + 0.5 * node.debounce)
    assert node.restart_due(t2 + node.debounce)   # no ground truth: no stillness check
    node.gt = (0.0, 0.0, 0.0, 2.0, 0.0)           # still moving
    assert not node.restart_due(t2 + node.debounce)
    assert node.restart_due(t2 + node.timeout + 0.1)
    node.constraints_cb(markers(1))
    node.constraints_cb(markers(1))
    assert not node.localized                     # constraints ignored until relocalized


def test_no_trajectory_while_restarting(node):
    node.tid = None   # start(): old trajectory finished, new one not started yet
    for n in (0, 1, 2, 3):
        node.constraints_cb(markers(n))
    assert not node.localized


def fake_cartographer(node, fail_first):
    """node.call replaced by a fake Cartographer that raises once for fail_first, then answers."""
    from cartographer_ros_msgs.msg import TrajectoryStates
    from cartographer_ros_msgs.srv import GetTrajectoryStates, StartTrajectory, FinishTrajectory
    calls, failed, state = [], [], {0: TrajectoryStates.FROZEN}

    def call(srv_type, name, request):
        calls.append(name)
        if name == fail_first and not failed:
            failed.append(name)
            raise RuntimeError(f'service {name} not available')
        if srv_type is GetTrajectoryStates:
            res = GetTrajectoryStates.Response()
            res.trajectory_states.trajectory_id = list(state)
            res.trajectory_states.trajectory_state = list(state.values())
            return res
        if srv_type is FinishTrajectory:
            state[request.trajectory_id] = TrajectoryStates.DELETED
            return FinishTrajectory.Response()
        tid = max(state) + 1
        state[tid] = TrajectoryStates.ACTIVE
        return StartTrajectory.Response(trajectory_id=tid)
    node.call = call
    return calls


def test_startup_retries_until_cartographer_answers(node):
    node.tid = node.frozen = None
    fake_cartographer(node, '/get_trajectory_states')   # pbstream still loading
    assert not node.try_start()
    assert node.tid is None
    assert node.try_start()
    assert node.tid == 1 and node.frozen == 0


def test_failed_relocalize_is_retried(node):
    node.frozen = 0
    calls = fake_cartographer(node, '/finish_trajectory')
    node.pending = ('test', 0.0)
    node.relocalize()
    assert node.pending and node.tid == 1   # still the old trajectory, retried after the debounce
    node.relocalize()
    assert node.pending is None and node.tid == 2
    assert calls.count('/finish_trajectory') == 2
