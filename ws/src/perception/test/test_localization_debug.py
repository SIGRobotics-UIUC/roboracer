"""localization_debug: the error in the car's frame, world -> map, distances and the withheld share, callbacks fed
directly (no executor)."""
import math

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from perception.localization_debug import LocalizationDebug, Path, pose_error, summary


def odom(x, y, yaw=0.0, speed=0.0):
    m = Odometry()
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.twist.twist.linear.x = speed
    return m


def pose(x, y, yaw=0.0):
    m = PoseStamped()
    m.pose.position.x, m.pose.position.y = x, y
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return m


@pytest.fixture
def node(tmp_path, monkeypatch):
    d = tmp_path / 'synth'
    d.mkdir()
    (d / 'map_to_world.yaml').write_text('frame_id: world\nx_m: 1.0\ny_m: 2.0\nyaw_rad: 1.5707963\n')
    monkeypatch.setenv('TRACKS_DIR', str(tmp_path))
    rclpy.init(args=['--ros-args', '-p', 'track:=synth'])
    n = LocalizationDebug()
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def test_error_is_in_the_cars_frame():
    err, long, lat, yaw = pose_error((1.0, 2.5, 0.1), (1.0, 2.0, math.pi / 2))   # 0.5 m ahead of a car facing +y
    assert err == pytest.approx(0.5)
    assert long == pytest.approx(0.5)
    assert lat == pytest.approx(0.0)
    assert yaw == pytest.approx(0.1 - math.pi / 2)


def test_lateral_error_positive_left():
    assert pose_error((0.0, 0.3, 0.0), (0.0, 0.0, 0.0))[2] == pytest.approx(0.3)


def test_yaw_error_wraps():
    assert pose_error((0.0, 0.0, 3.1), (0.0, 0.0, -3.1))[3] == pytest.approx(3.1 + 3.1 - 2 * math.pi)


def test_path_ignores_a_teleport():
    path = Path()
    for x, y in ((0.0, 0.0), (0.5, 0.0), (50.0, 0.0), (50.5, 0.0)):
        path.add(x, y)
    assert path.length == pytest.approx(1.0)


def test_summary_without_poses():
    assert summary([]) == 'no pose'


def test_truth_is_compared_in_the_map_frame(node):
    node.truth_cb(odom(1.0, 3.0, math.pi / 2))         # 1 m along the map's +x from an origin rotated 90 deg
    node.pose_cb(pose(1.0, 0.0, 0.0))
    assert node.errors == pytest.approx([0.0], abs=1e-5)
    node.pose_cb(pose(1.2, 0.1, 0.0))
    assert node.last_error[1] == pytest.approx(0.2, abs=1e-5)
    assert node.last_error[2] == pytest.approx(0.1, abs=1e-5)


def test_a_pose_before_any_truth_is_not_scored(node):
    node.pose_cb(pose(0.0, 0.0))
    assert node.errors == []


def test_withheld_counts_only_a_moving_car_without_a_pose(node):
    node.truth_cb(odom(1.0, 3.0))                      # standing: not expected to have a pose
    assert (node.moving, node.withheld) == (0, 0)
    node.truth_cb(odom(1.0, 3.0, speed=2.0))           # moving, no pose yet
    assert (node.moving, node.withheld) == (1, 1)
    node.pose_cb(pose(1.0, 0.0))
    node.truth_cb(odom(1.0, 3.1, speed=2.0))           # moving, fresh pose
    assert (node.moving, node.withheld) == (2, 1)


def test_odometry_over_truth(node):
    for k in range(11):
        node.truth_cb(odom(0.1 * k, 0.0))
        node.odom_cb(odom(0.105 * k, 0.0))
    assert 'odom/truth 1.05 over 1.0 m' in node.totals()


def test_a_missing_map_to_world_says_so(tmp_path, monkeypatch):
    (tmp_path / 'synth').mkdir()
    monkeypatch.setenv('TRACKS_DIR', str(tmp_path))
    rclpy.init(args=['--ros-args', '-p', 'track:=synth'])
    try:
        with pytest.raises(RuntimeError, match='map_to_world.yaml'):
            LocalizationDebug()
    finally:
        rclpy.try_shutdown()
