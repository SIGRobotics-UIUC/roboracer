"""Tests for disparity_gap_node: find_max_gap and find_best_point on hand-checkable arrays,
lidar_callback on ray-cast scenes (a pillar ahead, a corridor with the car near one wall, a dead end
with a side exit), each also mirrored left/right, and stopping after `laps` laps.

The car sits at the origin facing +x; +y is left, so a positive steering angle turns left.

Needs a sourced ROS 2 environment with the control package built:
    python3 -m pytest ws/src/control/test
"""
import os
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from control.disparity_gap_node import DisparityGapFollow  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
VEHICLE = os.path.join(HERE, '..', 'config', 'vehicle.yaml')

# Same LiDAR as test_follow_the_gap.py
ANGLE_MIN = -2.35619
ANGLE_INCREMENT = 0.004363323
NUM_BEAMS = 1081
RANGE_MAX = 10.0


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def create(**parameters):
        node = DisparityGapFollow(
            parameter_overrides=[Parameter(name, value=value) for name, value in parameters.items()]
        )
        nodes.append(node)
        return node

    try:
        yield create
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def drive(node, scan):
    """Run one scan through lidar_callback, return the published AckermannDrive."""
    msgs = []
    node.pub = SimpleNamespace(publish=msgs.append)
    node.lidar_callback(scan)
    assert len(msgs) == 1
    return msgs[0].drive


# Scenes are lists of wall segments ((x1, y1), (x2, y2)) in metres

def box(cx, cy, half):
    corners = [(cx - half, cy - half), (cx + half, cy - half),
               (cx + half, cy + half), (cx - half, cy + half)]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def corridor(left_y, right_y):
    return [((-5.0, left_y), (30.0, left_y)), ((-5.0, right_y), (30.0, right_y))]


def pillar_scene():
    # walls 2 m either side; a pillar 2.5 m ahead, shifted right: the left gap is wider
    return corridor(2.0, -2.0) + box(2.5, -0.3, 0.4)


def near_wall_scene():
    # car 0.4 m from the right wall, 2 m from the left wall
    return corridor(2.0, -0.4)


def dead_end_scene():
    # 2 m wide corridor ending 2 m ahead; a 1.5 m wide side corridor opens to the left
    return [
        ((-2.0, -1.0), (2.0, -1.0)),   # right wall
        ((2.0, -1.0), (2.0, 10.0)),    # end wall, continues up the side corridor
        ((-2.0, 1.0), (0.5, 1.0)),     # left wall, stops where the exit opens
        ((0.5, 1.0), (0.5, 10.0)),     # near wall of the side corridor
        ((-2.0, -1.0), (-2.0, 1.0)),   # wall behind the car
    ]


def mirror(segments):
    return [((x1, -y1), (x2, -y2)) for (x1, y1), (x2, y2) in segments]


def cast_rays(segments, angles):
    """Distance along each beam to the nearest segment, inf where nothing is hit."""
    dirs = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    ranges = np.full(len(angles), np.inf)
    for p, q in segments:
        p = np.asarray(p, dtype=float)
        e = np.asarray(q, dtype=float) - p
        denom = dirs[:, 0] * e[1] - dirs[:, 1] * e[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (p[0] * e[1] - p[1] * e[0]) / denom
            u = (p[0] * dirs[:, 1] - p[1] * dirs[:, 0]) / denom
        hit = (np.abs(denom) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
        ranges = np.where(hit, np.minimum(ranges, t), ranges)
    return ranges


def make_scan(ranges):
    scan = LaserScan()
    scan.angle_min = ANGLE_MIN
    scan.angle_max = -ANGLE_MIN
    scan.angle_increment = ANGLE_INCREMENT
    scan.range_min = 0.06
    scan.range_max = RANGE_MAX
    scan.ranges = [float(r) for r in ranges]
    return scan


def scene_scan(segments):
    angles = ANGLE_MIN + np.arange(NUM_BEAMS) * ANGLE_INCREMENT
    return make_scan(cast_rays(segments, angles))


@pytest.mark.parametrize('ranges, expected', [
    ([0, 0, 2.0, 2.0, 0], (2, 4)),                    # one gap in the middle
    ([2.0, 2.0, 0, 0, 0], (0, 2)),                    # gap at the start
    ([0, 0, 0, 2.0, 2.0], (3, 5)),                    # gap at the end
    ([2.0, 0, 3.0, 3.0, 3.0, 0, 1.0, 1.0], (2, 5)),   # longest of three gaps
    ([1.0, 1.0, 1.0, 1.0], (0, 4)),                   # no zeros at all
    ([0, 5.0, 0], (1, 2)),                            # single-beam gap
    ([5.0, 0, 1.0, 1.0, 1.0, 1.0], (0, 1)),           # a deep gap beats a wide shallow one
])
def test_find_max_gap(make_node, ranges, expected):
    assert make_node().find_max_gap(np.array(ranges)) == expected


def test_find_max_gap_none_when_all_zero(make_node):
    assert make_node().find_max_gap(np.zeros(5)) is None


def test_best_point_is_middle_of_deepest_run(make_node):
    ranges = np.array([1., 5., 5., 5., 5., 5., 2.])
    assert make_node().find_best_point(0, 7, ranges) == 3


@pytest.mark.parametrize('scene, sign', [
    (pillar_scene, +1),
    (near_wall_scene, +1),
    (dead_end_scene, +1),
], ids=['pillar', 'near_wall', 'dead_end'])
@pytest.mark.parametrize('mirrored', [False, True], ids=['', 'mirrored'])
def test_scene_steers_toward_the_way_out(make_node, scene, sign, mirrored):
    segments = mirror(scene()) if mirrored else scene()
    msg = drive(make_node(), scene_scan(segments))

    assert np.sign(msg.steering_angle) == (-sign if mirrored else sign)
    assert 0.0 < msg.speed <= 1.0


def test_stops_on_all_nan_scan(make_node):
    msg = drive(make_node(), make_scan(np.full(NUM_BEAMS, np.nan)))

    assert msg.speed == 0.0
    assert msg.steering_angle == 0.0


@pytest.mark.parametrize('bad', [np.nan, 0.0], ids=['nan', 'below_range_min'])
def test_dropout_beams_do_not_block_the_road(make_node, bad):
    # a straight corridor with a few invalid beams, one dead ahead: drive as if they weren't there
    node = make_node(max_speed=5.0)
    angles = ANGLE_MIN + np.arange(NUM_BEAMS) * ANGLE_INCREMENT
    clean = cast_rays(corridor(1.0, -1.0), angles)
    dropped = clean.copy()
    dropped[[np.argmin(np.abs(angles)), 400, 700]] = bad

    expected = drive(node, make_scan(clean))
    msg = drive(node, make_scan(dropped))

    assert msg.steering_angle == pytest.approx(expected.steering_angle, abs=0.02)
    assert msg.speed == pytest.approx(expected.speed, rel=0.05)


def test_steering_clipped_to_lock(make_node):
    node = make_node()
    # wall 0.5 m ahead, open only far to the left: the target is past the steering lock
    angles = ANGLE_MIN + np.arange(NUM_BEAMS) * ANGLE_INCREMENT
    ranges = np.where(angles > np.radians(70), 8.0, 0.5 / np.maximum(np.cos(angles), 0.1))

    msg = drive(node, make_scan(ranges))

    assert msg.steering_angle == pytest.approx(node.param('max_steering_angle'))


def test_speed_cap_wins_over_min_speed(make_node):
    msg = drive(make_node(max_speed=0.3), scene_scan(corridor(1.0, -1.0)))

    assert msg.speed == pytest.approx(0.3)


def test_speed_drops_when_road_ahead_is_short(make_node):
    node = make_node(max_speed=5.0)
    open_road = drive(node, scene_scan(corridor(1.0, -1.0))).speed
    wall_ahead = drive(node, scene_scan(corridor(1.0, -1.0) + [((1.5, -1.0), (1.5, 1.0))])).speed

    assert wall_ahead < open_road


def drive_laps(node, laps, radius=4.0, n=100):
    """Feed poses around a circle of the given radius, starting and ending at (radius, 0)."""
    for k in range(int(laps * n) + 1):
        msg = PoseStamped()
        msg.pose.position.x = radius * np.cos(2 * np.pi * k / n)
        msg.pose.position.y = radius * np.sin(2 * np.pi * k / n)
        node.pose_callback(msg)


def test_stops_driving_after_laps(make_node):
    node = make_node(laps=2)
    drive_laps(node, 1)
    assert node.lap_counter.laps == 1 and not node.done

    drive_laps(node, 1)
    assert node.done

    msgs = []
    node.pub = SimpleNamespace(publish=msgs.append)
    node.lidar_callback(scene_scan(corridor(1.0, -1.0)))
    assert msgs == []   # done: no more drive commands until stop()


def test_laps_zero_counts_no_laps(make_node):
    topics = [sub.topic_name for sub in make_node().subscriptions]
    assert topics == ['/scan']   # no pose subscription, so it drives until stopped


def test_stop_publishes_zero(make_node):
    node = make_node()
    msgs = []
    node.pub = SimpleNamespace(publish=msgs.append)

    node.stop()

    assert msgs and all(m.drive.speed == 0.0 and m.drive.steering_angle == 0.0 for m in msgs)


def test_max_steering_angle_default_matches_vehicle_yaml(make_node):
    with open(VEHICLE) as f:
        lock = yaml.safe_load(f)['/**']['ros__parameters']['max_steering_angle']
    assert make_node().param('max_steering_angle') == pytest.approx(lock)
