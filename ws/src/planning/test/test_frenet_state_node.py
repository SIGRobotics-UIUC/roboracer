"""frenet_state_node's pose source, on a synthetic circular track. No topics are exchanged."""
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter  # noqa: E402

from planning.frenet_state_node import FrenetStateNode  # noqa: E402

R, N = 4.0, 200


@pytest.fixture(scope='module')
def tracks(tmp_path_factory):
    root = tmp_path_factory.mktemp('tracks')
    d = root / 'ring'
    d.mkdir()
    th = np.linspace(0, 2 * np.pi, N, endpoint=False)
    xy = np.column_stack((R * np.cos(th), R * np.sin(th)))
    np.savetxt(d / 'centerline.csv', np.column_stack((xy, np.full(N, 1.0), np.full(N, 1.0))),
               delimiter=',', header='x_m,y_m,w_tr_right_m,w_tr_left_m')
    np.savetxt(d / 'raceline.csv',
               np.column_stack((np.arange(N) * (2 * np.pi * R / N), xy[:, 0], xy[:, 1],
                                th + np.pi / 2, np.full(N, 1 / R), np.full(N, 3.0), np.zeros(N))),
               delimiter=';', header='s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2')
    return root


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(tracks, **params):
    overrides = [Parameter('tracks_dir', value=str(tracks)), Parameter('track_name', value='ring')]
    overrides += [Parameter(k, value=v) for k, v in params.items()]
    return FrenetStateNode(parameter_overrides=overrides)


def subscribed(node):
    return {s.topic_name for s in node.subscriptions}


def test_default_pose_is_the_gated_localized_pose(ros, tracks):
    """AGENTS.md: racing consumes /localized_pose, never /tracked_pose, which runs
    0.8-3 m ahead in a sim stall and is withheld by the gate until localized."""
    node = make_node(tracks)
    try:
        assert subscribed(node) == {'/localized_pose', '/odom'}
    finally:
        node.destroy_node()


def test_pose_topic_can_still_be_overridden(ros, tracks):
    """For runs without a localizer, such as mapping."""
    node = make_node(tracks, pose_topic='/tracked_pose')
    try:
        assert subscribed(node) == {'/tracked_pose', '/odom'}
    finally:
        node.destroy_node()
