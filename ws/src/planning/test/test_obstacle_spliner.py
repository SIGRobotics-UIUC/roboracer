"""obstacle_spliner_node on a synthetic ring track (walls r = 3 and 5 m, raceline r = 4 m, counter-clockwise),
with scans ray-cast against the walls and a round obstacle. tick() runs directly; no topics."""
import itertools
import math

import numpy as np
import pytest
from PIL import Image

rclpy = pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from planning.obstacle_spliner_node import ObstacleSpliner  # noqa: E402

R_IN, R_OUT, R_LINE = 3.0, 5.0, 4.0
RES, HALF = 0.05, 6.0   # map: 12 x 12 m around the origin
HEADER = 's_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2'
HALF_WIDTH = 0.17   # the node's vehicle_half_width default
POCKET = (5.3, 5.8)
CLOCK = itertools.count(100)   # s: a stamp per tick, later each time
R_OUTER_LINE = 4.55   # outer_node's raceline


@pytest.fixture(scope='module')
def track(tmp_path_factory):
    d = tmp_path_factory.mktemp('ring')
    n = int(2 * HALF / RES)
    c = (np.arange(n) + 0.5) * RES - HALF
    x, y = np.meshgrid(c, c[::-1])   # image row 0 is the top
    r = np.hypot(x, y)
    img = np.full((n, n), 205, np.uint8)   # unknown, as map_saver writes it
    img[(r > R_IN) & (r < R_OUT)] = 254
    img[(r > POCKET[0]) & (r < POCKET[1])] = 254   # free space the lidar saw beyond the outer wall
    img[((r > R_IN - 0.1) & (r <= R_IN)) | ((r >= R_OUT) & (r < R_OUT + 0.1))] = 0
    Image.fromarray(img).save(d / 'map.pgm')
    (d / 'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {RES}\norigin: [{-HALF}, {-HALF}, 0.0]\n'
                                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    t = np.linspace(0, 2 * np.pi, 252)   # the last row repeats the first, as TUM writes it
    rows = np.column_stack([R_LINE * t, R_LINE * np.cos(t), R_LINE * np.sin(t), t, np.full_like(t, 0.25),
                            np.full_like(t, 3.0), np.zeros_like(t)])
    np.savetxt(d / 'raceline.csv', rows, delimiter='; ', fmt='%.7f', header=HEADER)
    return d


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def outer_node(ros, track, tmp_path):
    """The ring with the raceline 0.45 m from the outer wall."""
    (tmp_path / 'map.pgm').write_bytes((track / 'map.pgm').read_bytes())
    (tmp_path / 'map.yaml').write_text((track / 'map.yaml').read_text())
    t = np.linspace(0, 2 * np.pi, 400)
    rows = np.column_stack([R_OUTER_LINE * t, R_OUTER_LINE * np.cos(t), R_OUTER_LINE * np.sin(t), t,
                            np.full_like(t, 1 / R_OUTER_LINE), np.full_like(t, 3.0), np.zeros_like(t)])
    np.savetxt(tmp_path / 'raceline.csv', rows, delimiter='; ', fmt='%.7f', header=HEADER)
    n = ObstacleSpliner(parameter_overrides=[Parameter('raceline_path', value=str(tmp_path / 'raceline.csv')),
                                             Parameter('map_yaml', value=str(tmp_path / 'map.yaml'))])
    n.lidar = (0.27, 0.0, 0.0)
    n.published = []
    n.publish = lambda xy, valid: n.published.append((np.array(xy), valid))
    yield n
    n.destroy_node()


@pytest.fixture
def node(ros, track):
    n = ObstacleSpliner(parameter_overrides=[Parameter('raceline_path', value=str(track / 'raceline.csv')),
                                             Parameter('map_yaml', value=str(track / 'map.yaml'))])
    n.lidar = (0.0, 0.0, 0.0)   # no TF here; test_lidar_mount_offset sets its own
    n.published = []
    n.publish = lambda xy, valid: n.published.append((np.array(xy), valid))
    yield n
    n.destroy_node()


def on_ring(angle, r=R_LINE):
    return np.array([r * math.cos(angle), r * math.sin(angle)])


def ray_circle(p, u, c, radius):
    """Smallest t > 0 with |p + t u - c| = radius, or inf."""
    b = np.dot(u, p - c)
    disc = b * b - (np.dot(p - c, p - c) - radius ** 2)
    if disc < 0:
        return math.inf
    roots = [t for t in (-b - math.sqrt(disc), -b + math.sqrt(disc)) if t > 1e-6]
    return min(roots, default=math.inf)


def stamp(t):
    return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def pose_msg(x, y, yaw, t, frame='map'):
    pose = PoseStamped()
    pose.header.frame_id, pose.header.stamp = frame, stamp(t)
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


def tick(node, car_angle, obstacle=None, frame='map', car_r=R_LINE, pose_error=(0.0, 0.0), t=None):
    """Car at car_angle, car_r from the origin, heading along the ring; obstacle: (angle, r, radius).
    The scan is cast from the lidar, node.lidar[0] m ahead of the car; the pose and scan share stamp t.
    pose_error: (m outward, deg) the pose is off by (localization error)."""
    t = next(CLOCK) if t is None else t
    pos, yaw = on_ring(car_angle, car_r), car_angle + math.pi / 2
    seen = on_ring(car_angle, car_r + pose_error[0])
    pose = pose_msg(seen[0], seen[1], yaw + math.radians(pose_error[1]), t, frame)
    scan = scan_msg(node, pos, yaw, obstacle, t)
    node.pose_callback(pose)
    node.scan_callback(scan)
    count = len(node.published)
    node.tick()
    assert len(node.published) == count + 1   # every tick publishes: the pose at the scan's stamp is there
    return node.published[-1]


def scan_msg(node, pos, yaw, obstacle, t):
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = 'laser', stamp(t)
    scan.angle_min, scan.angle_max, scan.angle_increment = -2.35, 2.35, math.radians(0.25)
    scan.range_min, scan.range_max = 0.05, 30.0
    circles = [(np.zeros(2), R_IN), (np.zeros(2), R_OUT)]
    if obstacle:
        circles.append((on_ring(obstacle[0], obstacle[1]), obstacle[2]))
    angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
    lidar = pos + node.lidar[0] * np.array([math.cos(yaw), math.sin(yaw)])
    scan.ranges = [float(min(ray_circle(lidar, np.array([math.cos(yaw + a), math.sin(yaw + a)]), c, rad)
                             for c, rad in circles)) for a in angles]
    return scan


def radii(xy):
    return np.hypot(xy[:, 0], xy[:, 1])


def test_clear_track_is_the_raceline(node):
    xy, valid = tick(node, 0.0)
    assert valid
    assert node.scan_clusters() == []   # the walls are in the map: not obstacles
    assert np.allclose(xy, node.base_xy)


def test_free_space_beyond_the_walls_is_off_the_track(node):
    assert node.clearances(on_ring(0.3, R_LINE))[0] > 0.9
    assert node.clearances(on_ring(0.3, sum(POCKET) / 2))[0] == 0.0


def test_pose_in_another_frame_is_ignored(node):
    _, valid = tick(node, 0.0, frame='world')
    assert not valid


def test_passes_an_obstacle_on_the_line(node):
    obstacle = (2.0 / R_LINE, R_LINE, 0.15)   # 2 m ahead along the line, 0.3 m across
    xy, valid = tick(node, 0.0, obstacle)
    assert valid
    centre = on_ring(obstacle[0])
    # the car's centre line keeps clear of it by at least half the car's width, all along
    assert np.min(np.linalg.norm(xy - centre, axis=1)) > obstacle[2] + HALF_WIDTH
    # beside it the path is shifted for the obstacle's length and more, not only at its centre
    beside = np.abs(np.arctan2(xy[:, 1], xy[:, 0]) - obstacle[0]) * R_LINE < 0.4
    assert np.all(np.abs(radii(xy[beside]) - R_LINE) > 0.4)
    # inside the walls, and the raceline elsewhere
    assert np.all((radii(xy) > R_IN + HALF_WIDTH) & (radii(xy) < R_OUT - HALF_WIDTH))
    far = np.abs(np.angle(np.exp(1j * (np.arctan2(xy[:, 1], xy[:, 0]) - obstacle[0])))) > math.pi / 2
    assert np.allclose(xy[far], node.base_xy[far])


def test_keeps_the_path_while_beside_the_obstacle(node):
    obstacle = (2.0 / R_LINE, R_LINE, 0.15)
    approach, _ = tick(node, 0.0, obstacle)
    side = np.sign(radii(approach)[np.argmax(np.abs(radii(approach) - R_LINE))] - R_LINE)
    car_angle = obstacle[0] + 0.2 / R_LINE   # on the shifted path, 0.2 m past the obstacle's centre
    xy, valid = tick(node, car_angle, obstacle, car_r=R_LINE + side * 0.45)
    assert valid
    near_car = np.abs(np.arctan2(xy[:, 1], xy[:, 0]) - car_angle) * R_LINE < 0.2
    assert np.all(side * (radii(xy[near_car]) - R_LINE) > 0.3)   # still shifted, to the same side


def test_obstacle_next_to_a_wall_is_seen(node):
    # 0.13 m from the outer wall: its returns run on into the wall's, and must not be taken for the wall
    obstacle = (2.0 / R_LINE, R_OUT - 0.28, 0.15)
    for car_angle in np.arange(0.0, obstacle[0] - 0.1, 0.05):   # every tick of the approach
        _, valid = tick(node, car_angle, obstacle)
        assert valid
        seen = node.scan_clusters()
        assert len(seen) == 1
        # to within 0.03 m: the scan-to-map alignment may move it ~half a map cell along the ring
        assert np.max(np.linalg.norm(seen[0] - on_ring(obstacle[0], obstacle[1]), axis=1)) < obstacle[2] + 0.03


def test_long_cluster_beside_the_line_does_not_block(node, monkeypatch):
    # 1.5 m of wall seen 0.6 m left of the raceline where the map has it farther out (a stretched map):
    # its points are out of the car's way, though a circle around them would reach across the line
    t = np.linspace(1.5, 3.0, 60) / R_LINE
    wall = np.column_stack(((R_LINE + 0.6) * np.cos(t), (R_LINE + 0.6) * np.sin(t)))
    tick(node, 0.0)
    monkeypatch.setattr(node, 'scan_clusters', lambda *args: [wall])
    node.tick()
    xy, valid = node.published[-1]
    assert valid
    assert np.allclose(xy, node.base_xy)


def test_far_returns_need_more_clearance_from_the_walls(node):
    # within 0.24 m of the outer wall: a map that far off would put a wall there, so it counts only up close
    node.heading_tolerance = math.radians(2.0)
    obstacle = (1.0, R_OUT - 0.14, 0.1)
    tick(node, obstacle[0] - 4.0 / R_LINE, obstacle)    # ~4 m away: 0.12 + 4 tan(2 deg) = 0.26 m needed
    assert node.scan_clusters() == []
    tick(node, obstacle[0] - 1.0 / R_LINE, obstacle)    # ~1 m away: 0.16 m needed
    assert len(node.scan_clusters()) == 1


def test_lidar_mount_offset(node):
    # the RoboRacer's lidar is 0.27 m ahead of base_link: returns land where they hit, not 0.27 m short
    node.lidar = (0.27, 0.0, 0.0)
    obstacle = (2.0 / R_LINE, R_LINE, 0.15)
    tick(node, 0.0, obstacle)
    seen = node.scan_clusters()
    assert len(seen) == 1
    assert np.all(np.abs(np.linalg.norm(seen[0] - on_ring(obstacle[0]), axis=1) - obstacle[2]) < 0.03)


def test_blocked_track_is_not_valid(node):
    _, valid = tick(node, 0.0, (2.0 / R_LINE, R_LINE, 0.8))   # leaves 0.2 m each side
    assert not valid


def test_pose_at_interpolates_between_the_poses_around_a_stamp(node):
    node.pose_callback(pose_msg(1.0, 2.0, math.pi - 0.1, 10.00))
    node.pose_callback(pose_msg(1.2, 2.0, -math.pi + 0.1, 10.02))   # across +-pi: turned 0.2 rad, not 2 pi
    x, y, yaw = node.pose_at(10.01)
    assert (x, y) == pytest.approx((1.1, 2.0))
    assert math.cos(yaw) == pytest.approx(-1.0)
    assert node.pose_at(10.024) == pytest.approx((1.2, 2.0, -math.pi + 0.1))   # within max_pose_extrapolation
    assert node.pose_at(10.03) is None   # newer than every pose: not extrapolated
    assert node.pose_at(9.99) is None
    node.pose_callback(pose_msg(1.4, 2.0, 0.0, 10.20))
    assert node.pose_at(10.1) is None   # the poses around it are too far apart
    node.pose_callback(pose_msg(0.0, 0.0, 0.0, 5.0))   # time went back: the older run's poses are dropped
    assert node.pose_at(10.01) is None and node.pose_at(5.0) == pytest.approx((0.0, 0.0, 0.0))


def test_scan_is_placed_with_the_pose_at_its_stamp(node, monkeypatch):
    # the newest pose is 30 ms newer than the scan, and the car turned and moved out in between (as at
    # /odom rate in a hairpin): placed with it, the walls would land 0.15 m and 5 deg off
    monkeypatch.setattr(node, 'align', lambda *args: (0.0, 0.0, 0.0))   # the stamps alone
    yaw = math.pi / 2
    pos = on_ring(0.0)
    t = next(CLOCK)
    scan = scan_msg(node, pos, yaw, None, t)
    later = on_ring(0.0, R_LINE + 0.15)
    for dt, p, a in [(-0.015, pos, yaw), (0.0, pos, yaw), (0.03, later, yaw + math.radians(5))]:
        node.pose_callback(pose_msg(p[0], p[1], a, t + dt))
    node.scan_callback(scan)
    assert node.scan_clusters(scan, (later[0], later[1], yaw + math.radians(5)))   # the latest pose: walls
    node.tick()
    xy, valid = node.published[-1]
    assert valid
    assert np.allclose(xy, node.base_xy)


def test_scan_newer_than_the_poses_waits_for_them(node, monkeypatch):
    tick(node, 0.0)
    clock = [node.now()]
    monkeypatch.setattr(node, 'now', lambda: clock[0])
    t = next(CLOCK)
    node.pose_callback(pose_msg(*on_ring(0.0), math.pi / 2, t))
    node.scan_callback(scan_msg(node, on_ring(0.0), math.pi / 2, None, t + 0.03))   # no pose at it yet
    count = len(node.published)
    node.tick()
    assert len(node.published) == count   # nothing published: the last path stands
    clock[0] += node.max_pose_wait + 0.1   # still no pose at a scan's stamp: stale, not a frozen path
    node.pose_callback(pose_msg(*on_ring(0.0), math.pi / 2, t + 0.001))
    node.scan_callback(scan_msg(node, on_ring(0.0), math.pi / 2, None, t + 0.1))
    node.tick()
    assert len(node.published) == count + 1 and not node.published[-1][1]


@pytest.mark.parametrize('pose_error', [(0.15, 0.0), (-0.15, 0.0), (0.0, 2.0), (-0.1, -2.0), (0.15, 2.0)])
def test_localization_error_does_not_make_the_walls_obstacles(node, monkeypatch, pose_error):
    # the pose 0.1-0.15 m / 2 deg off (MCL at full speed, Cartographer): the walls the car runs beside land
    # inside the track, more than wall_clearance from the mapped edge, unless the scan is aligned to the map
    for car_angle in np.arange(0.0, 2 * math.pi, 0.5):
        xy, valid = tick(node, car_angle, pose_error=pose_error)
        assert valid
        assert node.scan_clusters() == []
        assert np.allclose(xy, node.base_xy)
    if abs(pose_error[0]) >= 0.15:   # 2 deg alone stays within the margin on this short ring
        monkeypatch.setattr(node, 'align', lambda *args: (0.0, 0.0, 0.0))
        node.last_align = None
        assert node.scan_clusters() != []   # what the alignment is for


def test_passes_an_obstacle_through_a_localization_error(node):
    obstacle = (2.0 / R_LINE, R_LINE, 0.15)
    xy, valid = tick(node, 0.0, obstacle, pose_error=(-0.12, 1.5))
    assert valid
    assert len(node.scan_clusters()) == 1
    assert np.min(np.linalg.norm(xy - on_ring(obstacle[0]), axis=1)) > obstacle[2] + HALF_WIDTH


def test_obstacle_split_in_two_is_passed_as_one(node, monkeypatch):
    # an obstacle the scan saw as two clusters, the second starting before the first's pass ends: the pass
    # holds beside both, beyond both
    def arc(s0, s1, d0, d1):
        t = np.linspace(s0, s1, 8) / R_LINE
        return np.vstack([np.column_stack(((R_LINE + d) * np.cos(t), (R_LINE + d) * np.sin(t))) for d in (d0, d1)])
    first, second = arc(2.0, 2.2, 0.0, 0.1), arc(2.6, 3.5, -0.1, 0.0)
    tick(node, 0.0)
    monkeypatch.setattr(node, 'scan_clusters', lambda *args: [first, second])
    node.tick()
    xy, valid = node.published[-1]
    assert valid
    reach = node.half_width + node.margin
    s_path = np.arctan2(xy[:, 1], xy[:, 0]) * R_LINE
    for part in (first, second):
        s_part = np.arctan2(part[:, 1], part[:, 0]) * R_LINE
        beside = (s_path >= s_part.min()) & (s_path <= s_part.max())
        gap = np.abs(radii(xy[beside])[:, None] - radii(part)[None, :])
        assert np.all(gap >= reach - 0.01)


def test_returns_in_front_of_the_walls_are_shadowed(node):
    # an obstacle's returns are short of the map's ray to the wall behind it; the walls' are not
    obstacle = (2.0 / R_LINE, R_LINE, 0.15)
    tick(node, 0.0, obstacle)
    scan = node.scan
    r = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(len(r)) * scan.angle_increment + math.pi / 2
    good = np.isfinite(r) & (r < 9.0)
    shadowed = node.shadowed(on_ring(0.0), angles, np.where(good, r, 0.0), good)
    hits = on_ring(0.0) + np.where(good, r, 0.0)[:, None] * np.column_stack((np.cos(angles), np.sin(angles)))
    on_obstacle = good & (np.linalg.norm(hits - on_ring(obstacle[0]), axis=1) < obstacle[2] + 0.01)
    assert on_obstacle.sum() > 10
    assert np.all(shadowed[on_obstacle])
    assert not np.any(shadowed[good & ~on_obstacle])


def map_odom(node, t, x, y, yaw=0.0):
    """TF map -> odom at stamp t, as the localizer broadcasts it."""
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id, tf.header.stamp = 'map', 'odom', stamp(t)
    tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    node.tf_buffer.set_transform(tf, 'test')


@pytest.mark.parametrize('steps', [[0.15], [0.15, 0.15], [0.15, -0.15], [0.12, 0.06, 0.06], [0.05, 0.05, 0.05],
                                   [0.09, 0.0, 0.0, 0.0, 0.09], [0.0]])
def test_correction_is_carried_across_re_anchors(node, steps):
    # 4 m/s along x, poses and map -> odom at 50 Hz, +-3 ms stamp jitter; the localizer re-anchors sideways by
    # steps, one per pose: the last tick's correction is carried across all of them, exactly
    rng = np.random.default_rng(2)
    off, t0 = 0.0, 500.0
    node.pose_callback(pose_msg(0.0, 0.0, 0.0, t0))
    for k in range(1, 20):
        t = t0 + 0.02 * k + rng.uniform(-0.003, 0.003)
        if 5 <= k < 5 + len(steps):
            off += steps[k - 5]
        map_odom(node, t, 0.0, off)
        node.pose_callback(pose_msg(4.0 * 0.02 * k, off, 0.0, t))
        if k == 3:
            node.last_align = (t, np.array([0.02, 0.1, 0.01]), node.map_odom(t), np.array([4.0 * 0.06, 0.0]))
    scan_t = t0 + 0.02 * 18
    carried = node.recent_correction(scan_t, node.map_odom(scan_t), np.array([4.0 * 0.02 * 18, off]))
    assert carried[1] == pytest.approx([0.02, 0.1 - sum(steps), 0.01], abs=1e-6)


def test_a_turned_map_frame_moves_the_correction_at_the_car(node):
    # map -> odom turns 2 deg about the map origin: at the car, 10 m out along x, the pose moves 0.35 m sideways
    node.pose_callback(pose_msg(10.0, 0.0, 0.0, 600.0))
    map_odom(node, 600.0, 0.0, 0.0)
    node.last_align = (600.0, np.zeros(3), node.map_odom(600.0), np.array([10.0, 0.0]))
    turn = math.radians(2.0)
    map_odom(node, 600.05, 0.0, 0.0, turn)
    node.pose_callback(pose_msg(10.0 * math.cos(turn), 10.0 * math.sin(turn), turn, 600.05))
    carried = node.recent_correction(600.05, node.map_odom(600.05), np.array([10.0, 0.0]))
    assert carried[1] == pytest.approx([10.0 - 10.0 * math.cos(turn), -10.0 * math.sin(turn), -turn], abs=1e-6)


def test_nothing_is_carried_across_a_gap_a_relocalization_or_a_reset(node):
    for k in range(5):
        map_odom(node, 700 + 0.02 * k, 0.0, 0.0)
        node.pose_callback(pose_msg(0.08 * k, 0.0, 0.0, 700 + 0.02 * k))
    node.last_align = (700.04, np.zeros(3), node.map_odom(700.04), np.array([0.16, 0.0]))
    assert node.recent_correction(700.08, node.map_odom(700.08), np.array([0.32, 0.0])) is not None
    assert node.recent_correction(700.08, None, np.array([0.32, 0.0])) is None   # no map -> odom at the stamp
    assert node.recent_correction(700.08, node.map_odom(700.08), np.array([3.0, 0.0])) is None   # teleported
    map_odom(node, 700.1, 0.0, 0.6)   # relocalized 0.6 m
    assert node.recent_correction(700.1, node.map_odom(700.1), np.array([0.4, 0.0])) is None
    assert node.recent_correction(701.0, node.map_odom(700.08), np.array([0.32, 0.0])) is None   # too old
    node.pose_callback(pose_msg(0.6, 0.0, 0.0, 700.3))   # the pose stream starts over after a gap
    assert node.recent_correction(700.3, node.map_odom(700.08), np.array([0.6, 0.0])) is None
    node.last_align = None   # the start: nothing before
    assert node.recent_correction(700.32, node.map_odom(700.08), np.array([0.6, 0.0])) is None


def test_correction_does_not_jump_without_the_pose(node):
    keep, cost = (lambda seed: tuple(seed)), (lambda c: 10.0 + 100 * abs(c[0] - 0.12))
    last = (10.0, np.array([0.12, 0.01, 0.002]))
    assert node.gate((0.10, 0.0, 0.0), 10.0, keep, cost, None) == (0.10, 0.0, 0.0)     # nothing before it
    assert node.gate((0.12, 0.03, 0.002), 10.07, keep, cost, last) == (0.12, 0.03, 0.002)   # near: taken
    # jumps 0.27 m, fitting no better than the correction it jumps from: the fit started there is kept
    assert node.gate((-0.15, 0.0, 0.0), 10.13, keep, lambda c: 10.0, last) == (0.12, 0.01, 0.002)
    # clearly better: taken
    assert node.gate((-0.15, 0.0, 0.0), 10.2, keep, lambda c: 1.0 if c[0] < 0 else 10.0, last) == (-0.15, 0.0, 0.0)
    node.gate((0.3, 0.0, 0.0), 10.3, keep, cost, None, (1.0, 2.0, 0.1), (5.0, 6.0))
    assert node.last_align[0] == 10.3 and node.last_align[2] == (1.0, 2.0, 0.1)
    assert np.allclose(node.last_align[1], (0.3, 0.0, 0.0)) and np.allclose(node.last_align[3], (5.0, 6.0))


def board_scan(node, pos, yaw, board, t):
    """scan_msg's ring walls, plus a board: the annulus sector r in board[0:2], angles board[2:4]."""
    scan = scan_msg(node, pos, yaw, None, t)
    lidar = pos + node.lidar[0] * np.array([math.cos(yaw), math.sin(yaw)])
    a = yaw + scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    steps = np.arange(0.02, 6.0, 0.005)
    q = lidar + steps[None, :, None] * np.stack((np.cos(a), np.sin(a)), axis=1)[:, None, :]
    r, th = np.hypot(q[..., 0], q[..., 1]), np.arctan2(q[..., 1], q[..., 0])
    hit = (r >= board[0]) & (r <= board[1]) & (th >= board[2]) & (th <= board[3])
    first = np.where(hit.any(axis=1), steps[np.argmax(hit, axis=1)], np.inf)
    scan.ranges = [float(min(w, b)) for w, b in zip(scan.ranges, first)]
    return scan


@pytest.mark.parametrize('lateral_error, v, reanchor', [
    (0.0, 2.0, []), (0.15, 2.0, []), (-0.15, 2.0, []), (0.15, 4.0, []), (0.15, 6.0, []),
    (0.0, 4.0, [0.15]), (0.0, 4.0, [0.15, 0.15]), (0.15, 4.0, [-0.15, 0.15]), (0.0, 4.0, [0.12, 0.06, 0.06])])
def test_drives_past_a_long_board_beside_the_wall(outer_node, lateral_error, v, reanchor):
    # a 3 m board 0.15 m off the outer wall, hiding it, beside a raceline 0.45 m from that wall; closed loop
    # with real timing (ticks at 15 Hz, poses at 50 Hz with +-3 ms stamp jitter, the scan 10 ms older than the
    # newest pose) at v m/s and the pose lateral_error m outward; 1 m beside the board the localizer re-anchors
    # outward by reanchor, a step per pose (map -> odom on TF follows, as from the localizer). The board's
    # returns, short of the map's ray, must not be fitted onto the wall (it would vanish and the car run into
    # it); with the pose 0.15 m toward the wall a single scan cannot tell it from the wall, but the correction
    # does not jump unless map -> odom does
    node, rl = outer_node, R_OUTER_LINE
    board = (4.7, 4.85, 1.0, 1.0 + 3.0 / rl)
    t, jitter = 1000.0, np.random.default_rng(3)
    error, reanchor = lateral_error, list(reanchor)
    angle, radius = board[2] - 6.0 / rl, rl
    prev = (angle, radius)
    worst = math.inf
    while angle < board[3] + 0.5 / rl:
        for k in range(1, 4):   # poses up to the tick, the car moving from prev
            u = k / 3
            a, r = prev[0] + u * (angle - prev[0]), prev[1] + u * (radius - prev[1])
            if reanchor and (a - board[2]) * rl >= 1.0:
                error += reanchor.pop(0)
            seen = on_ring(a, r + error)
            ts = t + u / 15 + jitter.uniform(-0.003, 0.003)
            offset = seen - on_ring(a, r)   # map -> odom: the pose's error (odom -> base_link the car)
            map_odom(node, ts, offset[0], offset[1])
            node.pose_callback(pose_msg(seen[0], seen[1], a + math.pi / 2, ts))
        t += 1 / 15
        a = prev[0] + 0.85 * (angle - prev[0])
        r = prev[1] + 0.85 * (radius - prev[1])
        node.scan_callback(board_scan(node, on_ring(a, r), a + math.pi / 2, board, t - 0.01))
        count = len(node.published)
        node.tick()
        assert len(node.published) == count + 1
        xy, valid = node.published[-1]
        if board[2] <= angle <= board[3]:
            worst = min(worst, board[0] - radius - HALF_WIDTH)
        prev = (angle, radius)
        if valid:   # follow the path: its radius 0.3 m ahead
            th = np.arctan2(xy[:, 1], xy[:, 0])
            radius = float(np.hypot(*xy[np.argmin(np.abs(np.angle(np.exp(1j * (th - angle - 0.3 / rl)))))]))
        angle += v / 15 / rl
    assert worst > 0.08   # the car's side keeps clear of the board all along
