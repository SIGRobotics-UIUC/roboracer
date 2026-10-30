"""mcl_node on the synthetic track of test_mcl (its lap as the raceline): callbacks fed directly (no executor),
searches waited for.

Finding the car (raceline first then verified, falling back to the whole area, unique, ambiguous with a guess
checked by driving, after a sim reset), and staying localized through an odometry jump, an odometry frame jump
and a stretch the map does not have (a low fit never stops it).
"""
import math
import threading

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from PIL import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from perception.mcl import Alternatives, compose, inverse, relative
from perception.mcl_node import MclNode
from test_mcl import ANGLE_INC, ANGLE_MIN, LASER, RANGE_MAX, RES, lap, make_track, pose_error, scan_from


@pytest.fixture(scope='module')
def lmap():
    return make_track()


@pytest.fixture
def tracks(tmp_path, lmap, monkeypatch):
    d = tmp_path / 'synth'
    d.mkdir()
    pix = np.full(lmap.occupied.shape, 205, dtype=np.uint8)   # unknown
    pix[lmap.free] = 254
    pix[lmap.occupied] = 0
    Image.fromarray(pix).save(d / 'map.pgm')
    (d / 'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {RES}\norigin: [0.0, 0.0, 0.0]\n'
                                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    rows = ['# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2']   # TUM format, as build_raceline writes it
    rows += [f'{0.08 * k:.4f}; {p[0]:.4f}; {p[1]:.4f}; 0.0; 0.0; 1.0; 0.0' for k, p in enumerate(lap())]
    (d / 'raceline.csv').write_text('\n'.join(rows) + '\n')
    monkeypatch.setenv('TRACKS_DIR', str(tmp_path))


class Car:
    """Drives the node: odometry (true pose seen through a movable odometry frame) and scans."""

    def __init__(self, lmap, **params):
        args = ['--ros-args', '-p', 'track:=synth', '-p', 'seed:=1', '-p', 'publish_particles:=false',
                '-p', 'search_period:=0.0', '-p', 'search_period_max:=0.0', '-p', 'debounce:=0.0']
        for k, v in params.items():
            args += ['-p', f'{k}:={v}']
        rclpy.init(args=args)
        self.node = MclNode()
        self.node.sensor_pose['laser'] = LASER
        self.node.tf_broadcaster.sendTransform = lambda t: None
        self.poses = []
        self.node.pose_pub.publish = self.poses.append
        self.lmap = lmap
        self.rng = np.random.default_rng(0)
        self.odom_frame = (0.0, 0.0, 0.0)   # odom pose = odom_frame * true pose
        self.stamp = 0.0                     # s; each step one 40 Hz odometry message and scan
        self.dt = 0.025
        self.searches = []                   # kinds of global searches the node started
        submit = self.node.submit_search

        def spy(kind, *a, **kw):
            self.searches.append(kind)
            submit(kind, *a, **kw)
        self.node.submit_search = spy

    def close(self):
        self.node.pool.shutdown(wait=True)
        self.node.destroy_node()
        rclpy.try_shutdown()

    def odom(self, true, speed=0.0):
        x, y, yaw = compose(self.odom_frame, true)
        m = Odometry()
        m.header.stamp.sec = int(self.stamp)
        m.header.stamp.nanosec = int(round((self.stamp - int(self.stamp)) * 1e9)) % 1_000_000_000
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.twist.twist.linear.x = speed
        return m

    def scan(self, true, block=None):
        m = LaserScan()
        m.header.frame_id = 'laser'
        m.angle_min, m.angle_increment = ANGLE_MIN, ANGLE_INC
        m.range_min, m.range_max = 0.1, RANGE_MAX
        r = scan_from(self.lmap, true, self.rng)
        if block is not None:   # something the map does not have, right in front: these beams end at 0.4 m
            r[block] = 0.4
        m.ranges = r.astype(np.float32).tolist()
        return m

    def step(self, true, block=None, speed=0.0):
        """One odometry message and one scan at the true pose; waits for a search the scan started."""
        self.stamp += self.dt
        self.node.odom_cb(self.odom(true, speed))
        self.node.scan_cb(self.scan(true, block))
        if self.node.job is not None:
            self.node.job[0].result()

    def stand(self, true, n=12):
        for _ in range(n):
            self.step(true)

    def pose(self):
        p = self.poses[-1].pose
        return p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w)


@pytest.fixture
def car(tracks, lmap):
    cars = []

    def make(**params):
        cars.append(Car(lmap, **params))
        return cars[-1]
    yield make
    for c in cars:
        c.close()


def drive(car, poses, check=None, **kw):
    for p in poses:
        car.step(p, **kw)
        if check:
            check(p)


def test_finds_the_car_at_the_start_and_drives(car):
    c = car()
    P = lap()
    c.stand(P[92])                      # beside the notch: unique; the map-origin prior is irrelevant
    assert c.node.localized and pose_error(P[92], c.pose())[0] < 0.1
    drive(c, P[93:160])
    assert pose_error(P[159], c.pose())[0] < 0.1


def test_ambiguous_start_guesses_then_driving_settles_it(car):
    """On the top straight nothing tells where along it the car is, and the map-origin prior is far away: after
    force_after failed searches the node guesses, keeps the other places, and driving toward the corner settles
    it on the true pose."""
    c = car()
    P = lap()
    start = 395
    c.stand(P[start], n=4 + 8)
    assert c.node.localized, 'never guessed'
    drive(c, P[start + 1:start + 140])   # through the corner and down the left side
    assert pose_error(P[start + 139], c.pose())[0] < 0.15


def test_odometry_jump_is_corrected_without_stopping(car):
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:110])
    # a 0.4 m jump the odometry did not see (a stalled sensor stream): from here on the odometry reads the car
    # 0.4 m further back along the track than it is (odom = J * true, J * P[110] = P[110] moved back 0.4 m)
    c.odom_frame = compose(compose(P[110], (-0.4, 0.0, 0.0)), inverse(P[110]))
    errors = []
    for p in P[110:150]:
        before = len(c.poses)
        c.step(p)
        assert c.node.localized and len(c.poses) == before + 1, 'stopped the car'
        errors.append(pose_error(p, c.pose())[0])
    assert errors[0] > 0.3 and max(errors[-20:]) < 0.1


def test_a_wrong_guess_is_switched_once_a_feature_comes_into_view(car):
    """The last known pose 1.2 m off along the featureless top straight makes the node pick the wrong place;
    the true one is kept as an alternative, and the stub on the island ahead settles it while driving."""
    c = car()
    P = lap()
    start = 395
    c.node.priors = [compose(P[start], (-1.2, 0.0, 0.0))]
    c.stand(P[start])
    assert c.node.localized and pose_error(P[start], c.pose())[0] > 0.9, 'did not pick the wrong place'
    drive(c, P[start + 1:start + 70])
    assert pose_error(P[start + 69], c.pose())[0] < 0.15


def test_a_start_with_the_view_blocked_is_found_and_corrected_once_it_clears(car):
    """Two thirds of the beams end at 0.4 m (the car stopped against something the map does not have): nothing
    reaches min_fit, so after force_after tries the relaxed floor takes a guess instead of waiting forever. The
    guess can be wrong; once the car drives on with a clear view, the pose must end up right."""
    c = car()
    P = lap()
    block = np.arange(45, 225)
    for _ in range(12):
        c.step(P[92], block=block)
    assert c.node.localized, 'still waiting'
    for p in P[93:160]:
        c.step(p)
    assert c.node.localized and pose_error(P[159], c.pose())[0] < 0.15


def test_a_teleport_the_odometry_did_not_see_is_found_by_a_background_search(car):
    """The car is picked up and put down on the far side of the track while the odometry runs on smoothly
    (nothing for the local searches to reach): a global search must find it."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:120])
    # from now on the true pose follows P[300:], the odometry continues from where P[119] left it
    odom_at = compose(c.odom_frame, P[119])
    c.odom_frame = compose(odom_at, inverse(P[299]))
    moved_at = None
    for k, p in enumerate(P[300:360]):
        c.step(p)
        if moved_at is None and pose_error(p, c.pose())[0] < 0.5:
            moved_at = k
            checks = c.searches.count('check')
    assert {'check', 'find'} & set(c.searches[1:]), 'no global search'
    assert moved_at is not None and ('find' in c.searches[1:] or checks >= 2), 'moved on one check search'
    assert c.node.localized and pose_error(P[359], c.pose())[0] < 0.15   # on the featureless top straight


def test_a_stretch_the_map_does_not_have_never_stops_the_car(car):
    """Two thirds of the beams blocked at 0.4 m for 60 updates (a parked car, a person): the fit stays low the
    whole time and a background search checks the pose; the pose must keep flowing and stay right."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    block = np.arange(45, 225)
    published = []
    for p in P[100:160]:
        before = len(c.poses)
        c.step(p, block=block)
        published.append(len(c.poses) > before)
        assert c.node.localized, 'stopped the car'
    assert all(published)
    assert 'check' in c.searches
    assert pose_error(P[159], c.pose())[0] < 0.15


def test_odometry_frame_jump_keeps_the_pose(car):
    """The odometry restarts (frame jumps 3 m): the published pose must not jump with it."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    c.odom_frame = (3.0, -2.0, 0.5)
    drive(c, P[100:110])
    assert c.node.localized and pose_error(P[109], c.pose())[0] < 0.1


def test_a_big_odometry_step_the_speed_explains_is_driven_not_dropped(car):
    """The streams stall for 0.7 s at 3.8 m/s (a sim hiccup): the next odometry is 2.5 m on, more than max_jump.
    That is the car driving, not the odometry frame jumping: the pose must follow it at once (dropping it left the
    pose 2.6 m behind on iros2026_v2, and the car crashed)."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:110], speed=3.8)     # 0.08 m per 40 Hz message
    c.stamp += 0.67                    # nothing for 0.67 s, then the car is 31 x 0.08 = 2.5 m on
    c.step(P[141], speed=3.8)
    for p in P[142:150]:
        c.step(p, speed=3.8)
        assert c.node.localized and pose_error(p, c.pose())[0] < 0.2, 'the pose stayed behind'
    assert pose_error(P[149], c.pose())[0] < 0.1


def frame_jump_keeps_the_pose(c, true, speed):
    """The odometry restarts (its frame jumps 3.6 m, 29 deg) between two 40 Hz messages: the published pose must
    not move with it, the filter must not drive the cloud by it, and the car must keep going."""
    c.odom_frame = compose((3.0, -2.0, 0.5), c.odom_frame)
    for _ in range(5):
        c.step(true, speed=speed)
        assert c.node.localized and pose_error(true, c.pose())[0] < 0.1


def test_an_odometry_frame_jump_while_driving_keeps_the_pose(car):
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:110], speed=3.8)
    frame_jump_keeps_the_pose(c, P[109], 3.8)   # the car "stands" at P[109]: the jump is all the step there is
    drive(c, P[110:130], speed=3.8)
    assert pose_error(P[129], c.pose())[0] < 0.1


def test_an_odometry_frame_jump_after_standing_still_keeps_the_pose(car):
    """Standing, the filter does not update (no motion), so the time since its last update grows: 20 s x the
    last crawl speed must not make a 3.6 m jump look like driving (it did: the cloud was moved 3.3 m)."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:110], speed=0.3)
    for _ in range(800):                 # 20 s standing
        c.step(P[109])
    frame_jump_keeps_the_pose(c, P[109], 0.0)
    drive(c, P[110:130], speed=1.0)
    assert pose_error(P[129], c.pose())[0] < 0.1


def test_an_odometry_frame_jump_drops_a_search_result_carried_across_it(car):
    """The odometry restarts while the verify behind the start runs: its result is carried by
    relative(odometry then, now), across the jump that is the jump itself (it moved the car 2.7 m)."""
    c = car()
    P = lap()
    for _ in range(40):
        c.step(P[92])
        if c.node.job is not None and c.node.job[1] == 'verify':
            break
    assert c.node.job is not None and c.node.job[1] == 'verify'
    c.node.job[0].result()   # finished, not yet polled
    c.odom_frame = (3.0, -2.0, 0.5)
    for _ in range(6):
        c.step(P[92])
        if c.poses:
            assert pose_error(P[92], c.pose())[0] < 0.1
    assert c.node.localized and pose_error(P[92], c.pose())[0] < 0.1


def test_an_odometry_frame_jump_carries_the_alternatives_into_the_new_frame(car):
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    c.node.alternatives = Alternatives([P[300]], c.node.odom[0])
    c.odom_frame = compose((3.0, -2.0, 0.5), c.odom_frame)
    c.step(P[99])                        # the jump (between two messages: the car stood still for it)
    c.step(P[100])
    alt = c.node.alternatives
    moved = relative(alt.odom0, c.node.odom[0])
    assert pose_error(compose(P[300], moved), compose(P[300], relative(P[99], P[100])))[0] < 0.02


def test_sim_reset_finds_the_car_where_it_was_put(car):
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    c.node.reset_cb(String(data='collision count 0 -> 1'))
    assert not c.node.localized
    c.odom_frame = (5.0, 5.0, 1.0)      # the bridge's odometry restarts too
    c.node.odom_cb(c.odom(P[300]))
    c.node.tick()                        # debounce 0, car still: search
    c.stand(P[300])
    assert c.node.localized and pose_error(P[300], c.pose())[0] < 0.1


def test_reanchor_state_is_cleared_by_a_restart(car):
    c = car()
    P = lap()
    c.stand(P[92])
    c.node.recheck_offset = (0.3, 0.0, 0.0)
    c.node.rechecks = 3
    c.node.init_filter(P[92], 'test')
    assert c.node.recheck_offset is None and c.node.rechecks == 0


def fake_local_search(c, offsets, fit=1.0):
    """Low-fit local searches (radius != 0.1; the 0.1 m refinement keeps its start) answer with the next offset
    from offsets, fitting this well (default perfectly): what a badly fitting part of the map can do."""
    k = [0]

    def search_local(pose, *a, radius=0.8, **kw):
        if radius == 0.1:
            return pose, fit
        k[0] += 1
        return compose(pose, offsets[(k[0] - 1) % len(offsets)]), fit
    c.node.matcher.search_local = search_local
    anchors = []
    reanchor = c.node.reanchor
    c.node.reanchor = lambda *a: (anchors.append(a[0]), reanchor(*a))
    return anchors


def test_a_low_fit_search_re_anchors_only_when_the_next_one_agrees(car):
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    block = np.arange(45, 225)   # the fit drops: every fit_bad_updates updates a local search
    anchors = fake_local_search(c, [(0.5, 0.0, 0.0), (-0.5, 0.0, 0.0)])
    drive(c, P[100:112], block=block)
    assert not anchors, 'one search moved the pose'
    anchors = fake_local_search(c, [(0.5, 0.0, 0.0)])
    drive(c, P[112:118], block=block)
    assert len(anchors) == 1


def test_a_local_search_never_re_anchors_to_a_pose_that_still_fits_badly(car):
    """Where the map itself is wrong (iros2026_v2, world (47-50, 17-27)) no pose near the car fits better than
    ~0.65; a local search there found a pose 0.9 m away fitting 0.66 (the filter's 0.47), the next one agreed (the
    map's error is the same scan after scan), and the re-anchors walked the pose 2.5 m along the track into a
    crash. A better fit that is still a bad one is not a re-anchor."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    block = np.arange(45, 225)   # the fit drops: every fit_bad_updates updates a local search
    anchors = fake_local_search(c, [(0.5, 0.0, 0.0)], fit=0.66)
    drive(c, P[100:130], block=block)
    assert not anchors, 're-anchored to a pose fitting 0.66'
    anchors = fake_local_search(c, [(0.5, 0.0, 0.0)], fit=0.9)   # a slip on a good map: re-anchors
    drive(c, P[130:140], block=block)
    assert anchors


def test_a_recheck_never_re_anchors_to_a_pose_that_still_fits_badly(car):
    """The recheck (fit normal, every recheck_updates updates) on a map that fits ~0.6 everywhere here: a pose
    0.4 m along fitting 0.75 (> 0.6 + recheck_gain) is no better, it must not move the pose; 0.85 does."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    c.node.matcher.fit = lambda *a, **kw: 0.6
    c.node.fit_level = 0.6
    anchors = fake_local_search(c, [(0.4, 0.0, 0.0)], fit=0.75)
    drive(c, P[100:140])
    assert not anchors, 're-anchored to a pose fitting 0.75'
    anchors = fake_local_search(c, [(0.4, 0.0, 0.0)], fit=0.85)
    drive(c, P[140:160])
    assert anchors


def test_a_background_search_moves_the_car_only_when_the_next_one_finds_the_same_place(car):
    """A check search (a burst, a long low fit) found the corridor across the wall, 2 m away, where the map is
    wrong (iros2026_v2) and moved the car there. Check searches only take places fitting relocate_floor() (the
    worker returns no choice below it), and a place found must be found again by the next search; a finding that
    disagrees with the last becomes the one the next search has to confirm."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    here = c.node.pf.estimate()
    moved = (0.0, 0.0, 0.0)
    c.node.on_check((P[300], [], 'fit 0.97'), moved, 1.0, 0.97, 0.3, c.node.pf.estimate())
    assert pose_error(here, c.node.pf.estimate())[0] < 0.05, 'relocated on one search'
    assert c.node.burst, 'no second search asked for'
    c.node.burst = False
    c.node.on_check((P[200], [], 'fit 0.97'), moved, 1.0, 0.97, 0.3, c.node.pf.estimate())
    assert pose_error(here, c.node.pf.estimate())[0] < 0.05, 'relocated on two searches that disagree'
    assert c.node.burst, 'the second finding was dropped instead of checked'
    c.node.on_check((P[200], [], 'fit 0.97'), moved, 1.0, 0.97, 0.3, c.node.pf.estimate())
    assert c.node.localized and pose_error(P[200], c.node.pf.estimate())[0] < 0.1


def test_check_searches_only_take_places_that_fit_well(car):
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    floors = []
    submit = c.node.submit_search
    c.node.submit_search = lambda kind, *a, **kw: (floors.append(a[5]) if kind == 'check' else None,
                                                   submit(kind, *a, **kw))
    c.node.burst = True
    drive(c, P[100:103])
    assert floors and min(floors) >= 0.85
    c.node.good_level = 0.6          # a lidar/map that fits less everywhere: the gates scale down with it
    assert c.node.relocate_floor() < 0.6 and c.node.reanchor_floor() < 0.55


def test_findings_that_keep_disagreeing_stop_asking_for_searches(car):
    """Where the map is wrong two places can keep winning in turn: after RELOCATE_TRIES searches in a row the
    node stops asking for more (each costs ~1-2 s of the worker) and waits for the next trigger."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    asked = []
    for k in range(6):
        c.node.burst = False
        c.node.on_check((P[200] if k % 2 else P[300], [], 'fit 0.97'), (0.0, 0.0, 0.0), 1.0, 0.97, 0.3,
                        c.node.pf.estimate())
        asked.append(c.node.burst)
    assert asked[:3] == [True] * 3 and not any(asked[3:])
    assert pose_error(P[99], c.node.pf.estimate())[0] < 0.1


def test_the_good_fit_level_is_not_dragged_down_by_a_bad_stretch(car):
    """100 updates (2.5 s) where the true pose fits only 0.74 (iros2026_v2's bad stretch fits <= 0.74) after a
    good stretch: the gates must stay above the 0.66 re-anchor that went 0.9 m wrong there and the 0.84
    look-alike across the wall (a level that followed the fit put them at 0.66 and 0.72)."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:110])
    assert c.node.good_level > 0.9
    c.node.matcher.fit = lambda *a, **kw: 0.74
    drive(c, P[110:210])
    assert c.node.reanchor_floor() > 0.66 and c.node.relocate_floor() > 0.84


def test_the_good_fit_level_follows_a_lidar_that_fits_less_everywhere(car):
    """A real lidar (or map) fitting ~0.7 everywhere: the gates must come down to it, not stay at the sim's
    absolute values and disable re-anchors. Starting there: at once; after a good start: within ~70 m."""
    c = car()
    c.node.track_good_level(0.7)
    for _ in range(50):
        c.node.track_good_level(0.68 + 0.04 * c.rng.random())
    assert c.node.reanchor_floor() < 0.65 and c.node.relocate_floor() < 0.7
    c.node.good_level = None
    c.node.track_good_level(0.97)
    for _ in range(3000):
        c.node.track_good_level(0.68 + 0.04 * c.rng.random())
    assert c.node.reanchor_floor() < 0.66 and c.node.relocate_floor() < 0.75
    assert min(c.node.reanchor_floor(), c.node.relocate_floor()) >= c.node.min_fit


def test_a_burst_of_re_anchors_pauses_them_and_checks_in_the_background(car):
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    for _ in range(3):
        c.node.reanchor(c.node.pf.estimate(), None, 'test')
    assert c.node.burst and c.node.hold_until > 0.0
    n = len(c.searches)
    anchors = fake_local_search(c, [(0.5, 0.0, 0.0)])
    published = []
    for p in P[100:115]:
        before = len(c.poses)
        c.step(p, block=np.arange(45, 225))
        published.append(len(c.poses) > before)
    assert 'check' in c.searches[n:] and not c.node.burst
    assert not anchors, 're-anchored during the pause'
    assert all(published), 'the car waited'


def settle(c, n=6):
    """Let a background (verify) search finish and be handled."""
    for _ in range(n):
        if c.node.job is not None:
            c.node.job[0].result()
        c.node.poll_search()


def test_raceline_first_finds_the_car_and_the_whole_area_search_confirms(car):
    c = car()
    P = lap()
    c.stand(P[92])
    settle(c)
    assert c.searches[:2] == ['find', 'verify']
    assert c.node.localized and c.node.alternatives is None and pose_error(P[92], c.pose())[0] < 0.1


def test_a_car_turned_across_the_track_falls_back_to_the_whole_area(car):
    c = car()
    pose = (10.0, 1.5, math.radians(100))
    c.stand(pose)
    settle(c)
    assert 'verify' not in c.searches     # answered by the whole-area search itself
    assert c.node.localized and pose_error(pose, c.pose())[0] < 0.1 and pose_error(pose, c.pose())[1] < 0.05


def test_a_car_facing_backwards_is_never_confidently_misplaced(car):
    """On the right side facing down it sees what a car driving down the left side sees: the node must either
    place it right or keep the right place as an alternative to settle by driving."""
    c = car()
    pose = (18.55, 6.0, -math.pi / 2)
    c.stand(pose)
    settle(c)
    assert c.node.localized
    if pose_error(pose, c.pose())[0] > 0.15:
        alts = c.node.alternatives
        assert alts is not None and any(pose_error(pose, q)[0] < 0.15 for q in alts.poses)


def test_the_expected_start_beats_a_look_alike_the_search_found_elsewhere(car):
    """The raceline-first search only looks near the raceline: a start further from it is not among its
    places, and a look-alike far away fitting about as well was picked (iros2026). The expected pose (the map
    origin race.sh puts the car at) is scored directly and wins the tie."""
    c = car()
    P = lap()
    start = P[92]
    c.node.priors = [start]
    search_global = c.node.matcher.search_global
    injected = []

    def look_alike_only(ranges, angles, sensor, **kw):
        far = [h for h in search_global(ranges, angles, sensor, **kw) if pose_error(start, h[0])[0] > 3.0]
        injected.append(far[0][0])
        return [(far[0][0], 1.0)]   # only a look-alike, fitting at least as well as the true place
    c.node.matcher.search_global = look_alike_only
    c.stand(start)
    assert injected, 'no look-alike'
    assert c.node.localized and pose_error(start, c.pose())[0] < 0.1


def test_a_re_anchor_does_not_discard_the_verify_search(car):
    """A re-anchor while driving, before the verify behind the start has finished, used to make the verify stale,
    so a wrong start was never corrected by it."""
    c = car()
    P = lap()
    release = threading.Event()
    search_global = c.node.matcher.search_global

    def slow_whole_area(*a, track_only=False, **kw):
        if not track_only:          # the whole-area search (the verify) waits until the car is driving
            assert release.wait(30), 'never released'
        return search_global(*a, track_only=track_only, **kw)
    c.node.matcher.search_global = slow_whole_area
    for _ in range(12):
        c.stamp += 1
        c.node.odom_cb(c.odom(P[92]))
        c.node.scan_cb(c.scan(P[92]))
        if c.node.job is not None and c.node.job[1] != 'verify':
            c.node.job[0].result()   # the raceline-first find
        if c.node.localized:
            break
    assert c.node.localized and c.node.job is not None and c.node.job[1] == 'verify'
    verified = []
    on_verify = c.node.on_verify
    c.node.on_verify = lambda *a, **kw: (verified.append(a), on_verify(*a, **kw))
    c.node.reanchor(c.node.pf.estimate(), None, 'test')
    release.set()
    settle(c)
    assert verified, 'the verify search was discarded'


def test_without_hybrid_the_whole_area_search_finds_the_car(car):
    c = car(hybrid='false')
    P = lap()
    c.stand(P[92])
    assert 'verify' not in c.searches and c.node.localized and pose_error(P[92], c.pose())[0] < 0.1


def half_turn(pose):
    return 20.0 - pose[0], 12.0 - pose[1], math.atan2(math.sin(pose[2] + math.pi), math.cos(pose[2] + math.pi))


def test_a_guessed_look_alike_is_corrected_even_when_the_true_place_was_not_among_the_alternatives(car):
    """A guess picked the half-turn look-alike and the true place was not among the alternatives (a guess made
    with most of the view blocked): the alternatives cannot fix that, and nothing else did (18.6 m off for
    over two laps). After a guess, whole-area searches every unsure_period m must find the true place."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    c.node.init_filter(half_turn(P[99]), 'test guess', keep_localized=True, matched=True)
    c.node.make_unsure()
    errs = []
    for p in P[100:600]:
        c.step(p)
        errs.append(pose_error(p, c.pose())[0])
    assert errs[0] > 5.0
    assert max(errs[-100:]) < 0.15, f'still {errs[-1]:.1f} m off'
    assert c.node.localized


def test_doubt_fires_although_the_fit_level_followed_the_fit_down(car):
    """While the pose was wrong every failed local search pulled fit_level 2 % toward the bad fit (0.97 -> 0.04 in
    500 updates): doubt, relative to it, never fired. It is relative to the good-fit level now."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:110])
    c.node.fit_level = 0.04                                  # where 500 bad updates left it
    c.node.matcher.fit = lambda *a, **kw: 0.03               # the scan does not fit here at all
    c.node.local_search = lambda pose, *a: (pose, 0.0)       # and nothing nearby does better
    n = len(c.searches)
    drive(c, P[110:125])
    assert 'check' in c.searches[n:]


def test_a_pose_off_the_map_is_withdrawn_and_the_car_found_again(car):
    """The cloud drifted off the map where no scan contradicts it (y = -15 m on a 12 m map, still localized for
    45 m): in a wall or outside the map for doubt_updates updates it is lost, and the car is found again."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    c.node.init_filter((-3.0, -5.0, 0.0), 'test drift', keep_localized=True, matched=True)
    lost = False
    for p in P[100:160]:
        c.step(p)
        lost = lost or not c.node.localized
    assert lost and 'find' in c.searches[1:]
    assert c.node.localized and pose_error(P[159], c.pose())[0] < 0.15


def test_the_verify_moves_a_driving_car_only_to_a_place_that_fits_clearly_better(car):
    """The verify behind a start used to move a driving car on one whole-area search with a 0.5 floor, without
    looking at how the scan fits where the car is (the search can miss the true pose). Now it needs
    relocate_floor() and relocate_margin over the filter's fit; otherwise the place becomes an alternative."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    here, far, moved = c.node.pf.estimate(), P[300], (0.0, 0.0, 0.0)
    c.node.alternatives = None
    c.node.on_verify([(far, 0.6)], (far, [], 'fit 0.60'), moved, 1.0, 0.6, 0.9, here)
    assert pose_error(here, c.node.pf.estimate())[0] < 0.05, 'moved to a place fitting 0.6'
    assert c.node.alternatives is not None and any(pose_error(far, q)[0] < 0.2 for q in c.node.alternatives.poses)
    c.node.on_verify([(far, 0.97)], (far, [], 'fit 0.97'), moved, 1.0, 0.97, 0.95, here)
    assert pose_error(here, c.node.pf.estimate())[0] < 0.05, 'moved to a place fitting no better'
    c.node.on_verify([(far, 0.97)], (far, [], 'fit 0.97'), moved, 1.0, 0.97, 0.4, here)
    assert pose_error(far, c.node.pf.estimate())[0] < 0.1


@pytest.mark.parametrize('interpolate', [True, False])
def test_each_scan_uses_the_odometry_at_its_stamp(car, interpolate):
    """Lidar latency: each scan arrives with the /odom of two messages later (50 ms, 0.16 m here; 0.4 m at 8 m/s
    with 50 ms on a real car). Taking the newest /odom for it puts the car where it was 50 ms ago."""
    c = car(interpolate_odom=str(interpolate).lower())
    P = lap()
    c.stand(P[92])
    errs = []
    for k in range(93, 200):
        c.stamp += c.dt
        c.node.odom_cb(c.odom(P[k + 2], speed=3.2))            # /odom runs 2 messages ahead of the scans
        m = c.scan(P[k])
        t = c.stamp - 2 * c.dt
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1_000_000_000
        c.node.scan_cb(m)
        if c.node.job is not None:
            c.node.job[0].result()
        if k > 120:
            errs.append(pose_error(P[k + 2], c.pose())[0])   # the published pose, at the newest /odom
    if interpolate:
        assert np.mean(errs) < 0.05
    else:
        assert np.mean(errs) > 0.1


def test_a_re_anchor_to_the_filters_own_pose_changes_nothing_under_lidar_latency(car):
    """Scans 50 ms behind /odom: a re-anchor to the scan-time pose must be tagged with the scan's odometry. Tagged
    with the newest it put the filter speed x latency behind (0.045 -> 0.111 m here, ~0.4 m at 8 m/s)."""
    c = car()
    P = lap()
    c.stand(P[92])
    before, after = [], []
    for k in range(93, 200):
        c.stamp += c.dt
        c.node.odom_cb(c.odom(P[k + 2], speed=3.2))
        m = c.scan(P[k])
        t = c.stamp - 2 * c.dt
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1_000_000_000
        c.node.scan_cb(m)
        if c.node.job is not None:
            c.node.job[0].result()
        if k == 150:
            c.node.reanchor(c.node.pf.estimate(), c.node.scan_odom, 'test')
        (before if k <= 150 else after).append(pose_error(P[k + 2], c.pose())[0])
    assert np.mean(after[:10]) < np.mean(before[-20:]) + 0.02


def test_the_searches_after_a_guess_have_one_budget_per_find(car):
    """A look-alike that every search finds again re-armed the searches (make_unsure on each finding): a 1.6 s
    whole-area search every unsure_period m forever. The budget is per find."""
    c = car(unsure_searches=3)
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    submitted = []
    c.node.submit_search = lambda kind, *a, **kw: submitted.append(kind)
    c.node.make_unsure()
    for _ in range(10):
        c.node.recheck_guess(None, None, None, c.node.unsure_period)
        c.node.make_unsure()            # each finding again
    assert submitted == ['reverify'] * 3


def test_a_pose_that_fits_is_never_withdrawn_for_sitting_in_an_occupied_blob(car):
    """A person or box captured in a real-car map can sit where the car drives: a pose that fits the scan there is
    a good pose. Only one in a wall or off the map that does not fit is withdrawn (no false stops)."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    cell = c.node.lmap.cell
    c.node.lmap.cell = lambda x, y: np.full(np.shape(x), 2) if np.ndim(x) == 0 else cell(x, y)
    for p in P[100:140]:
        c.step(p)
        assert c.node.localized, 'withdrew a pose that fits'
    c.node.lmap.cell = cell


def test_a_re_anchor_keeps_the_places_still_to_check(car):
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    c.node.alternatives = Alternatives([P[300]], c.node.odom[0])
    c.node.reanchor(c.node.pf.estimate(), c.node.odom[0], 'test')
    assert c.node.alternatives is not None and len(c.node.alternatives.poses) == 1


def test_a_reverify_while_driving_never_moves_the_car_on_one_search(car):
    """The searches after a guess run while the car drives: a place one of them finds, however well it fits,
    becomes an alternative for the evidence to decide (on_check needs two agreeing searches for the same)."""
    c = car()
    P = lap()
    c.stand(P[92])
    drive(c, P[93:100])
    here, far = c.node.pf.estimate(), P[300]
    c.node.on_verify([(far, 0.99)], (far, [], 'fit 0.99'), (0.0, 0.0, 0.0), 1.0, 0.99, 0.3, here, may_move=False)
    assert pose_error(here, c.node.pf.estimate())[0] < 0.05
    assert c.node.alternatives is not None and any(pose_error(far, q)[0] < 0.2 for q in c.node.alternatives.poses)
