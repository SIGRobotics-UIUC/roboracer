"""mcl: map loading, SE(2) helpers, motion model, and localization on a synthetic track with a simulated lidar."""
import math

import numpy as np
import pytest
from PIL import Image

from perception.mcl import (Alternatives, LikelihoodMap, ParticleFilter, ScanMatcher, choose_pose, compose, inverse,
                            relative, select_beams, wrap)

RES = 0.05
LASER = (0.27, 0.0, 0.0)          # laser in base_link (rear axle)
ANGLE_MIN, ANGLE_INC, N_SCAN, RANGE_MAX = -math.radians(135), math.radians(1.0), 271, 10.0


def make_track(features=True):
    """20 x 12 m loop corridor (2.5 m wide) with an asymmetric wall notch and two wall stubs. Image order.
    features=False: the bare loop, symmetric under a half turn about its centre."""
    h, w = int(12 / RES), int(20 / RES)
    occ = np.zeros((h, w), dtype=bool)

    def box(x0, y0, x1, y1):   # metres, y up
        occ[h - int(y1 / RES):h - int(y0 / RES), int(x0 / RES):int(x1 / RES)] = True

    box(0, 0, 20, 0.2), box(0, 11.8, 20, 12), box(0, 0, 0.2, 12), box(19.8, 0, 20, 12)   # outer walls
    box(2.7, 2.7, 17.3, 9.3)                                                             # island
    if features:
        box(9.0, 0.2, 10.5, 1.0)                                                         # notch
        box(15.0, 10.2, 15.4, 11.0)                                                      # wall stub
        box(3.0, 9.3, 3.4, 10.0)                                                         # stub on the island
    free = ~occ
    free[:int(0.2 / RES)] = free[-int(0.2 / RES):] = False
    return LikelihoodMap(occ, free, RES, (0.0, 0.0))


def lap(step=0.08):
    """Poses (x, y, yaw) along the corridor centre line (rounded rectangle), counter-clockwise."""
    x0, y0, x1, y1, r = 1.45, 1.45, 18.55, 10.55, 0.9
    pts = []

    def seg(a, b):
        n = max(1, int(math.dist(a, b) / step))
        pts.extend((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n))

    def arc(cx, cy, a0):
        n = max(1, int(r * math.pi / 2 / step))
        pts.extend((cx + r * math.cos(a0 + math.pi / 2 * i / n), cy + r * math.sin(a0 + math.pi / 2 * i / n))
                   for i in range(n))

    seg((x0 + r, y0), (x1 - r, y0)), arc(x1 - r, y0 + r, -math.pi / 2)
    seg((x1, y0 + r), (x1, y1 - r)), arc(x1 - r, y1 - r, 0.0)
    seg((x1 - r, y1), (x0 + r, y1)), arc(x0 + r, y1 - r, math.pi / 2)
    seg((x0, y1 - r), (x0, y0 + r)), arc(x0 + r, y0 + r, math.pi)
    pts.append(pts[0])
    return [(p[0], p[1], math.atan2(q[1] - p[1], q[0] - p[0])) for p, q in zip(pts[:-1], pts[1:])]


def scan_from(lmap, pose, rng):
    """Simulated lidar at a base_link pose: ray march through the occupancy grid (+ 1 cm noise)."""
    x, y, th = compose(pose, LASER)
    angles = th + ANGLE_MIN + ANGLE_INC * np.arange(N_SCAN)
    t = np.arange(0.1, RANGE_MAX, 0.02)
    px = x + t[None, :] * np.cos(angles)[:, None]
    py = y + t[None, :] * np.sin(angles)[:, None]
    col = np.floor(px / RES).astype(int)
    row = lmap.h - 1 - np.floor(py / RES).astype(int)
    inside = (col >= 0) & (col < lmap.w) & (row >= 0) & (row < lmap.h)
    hit = np.where(inside, lmap.occupied[np.clip(row, 0, lmap.h - 1), np.clip(col, 0, lmap.w - 1)], False)
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(t) - 1)
    r = t[first] + rng.normal(0.0, 0.01, N_SCAN)
    r[~hit.any(axis=1)] = RANGE_MAX
    return r


def drive(pf, lmap, poses, rng, odom_bias=1.03, every=3):
    """Run the filter along a lap with biased, noisy wheel odometry; yields (true pose, estimate)."""
    odom_prev = poses[0]
    odom = poses[0]
    for i in range(every, len(poses), every):
        true_delta = relative(poses[i - every], poses[i])
        noisy = (true_delta[0] * odom_bias + rng.normal(0, 0.003), true_delta[1] + rng.normal(0, 0.003),
                 true_delta[2] + rng.normal(0, 0.003))
        odom = compose(odom, noisy)
        pf.predict(relative(odom_prev, odom))
        odom_prev = odom
        r, a = select_beams(scan_from(lmap, poses[i], rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
        pf.update_sensor(r, a, LASER, lmap)
        pf.resample()
        yield poses[i], pf.estimate()


def pose_error(true, est):
    return math.hypot(true[0] - est[0], true[1] - est[1]), abs(float(wrap(true[2] - est[2])))


@pytest.fixture(scope='module')
def lmap():
    return make_track()


def test_se2_helpers():
    a, b = (1.0, 2.0, 0.7), (-0.5, 0.3, -1.2)
    assert compose(a, inverse(a)) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert relative(a, compose(a, b)) == pytest.approx(b)
    assert compose((0, 0, math.pi / 2), (1, 0, 0)) == pytest.approx((0, 1, math.pi / 2))
    assert abs(float(wrap(3 * math.pi))) == pytest.approx(math.pi)
    assert float(wrap(0.1 + 2 * math.pi)) == pytest.approx(0.1)


def test_map_yaml_matches_save_map_format(tmp_path):
    pix = np.full((20, 40), 254, dtype=np.uint8)   # 2 x 1 m at 0.05
    pix[:, 0] = 0                                  # wall at x in [0, 0.05)
    pix[0, :] = 205                                # unknown top row
    Image.fromarray(pix).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text('image: map.pgm\nmode: trinary\nresolution: 0.05\n'
                                       'origin: [-1.0, 3.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n'
                                       'free_thresh: 0.196\n')
    m = LikelihoodMap.from_yaml(tmp_path / 'map.yaml')
    assert m.distance(np.array([-0.975]), np.array([3.5]))[0] == 0.0          # in the wall column
    assert m.distance(np.array([-0.475]), np.array([3.5]))[0] == pytest.approx(0.5, abs=0.06)
    assert m.distance(np.array([50.0]), np.array([3.5]))[0] == m.max_dist     # outside the map
    assert not m.free[0].any() and m.free[5, 5]                               # unknown is not free
    grid = m.occupancy_grid().reshape(20, 40)                                 # row 0 = min y (image flipped)
    assert grid[0, 0] == 100 and grid[0, 5] == 0 and (grid[-1] == -1).all()


def test_map_with_rotated_origin_is_rejected(tmp_path):
    Image.fromarray(np.zeros((4, 4), dtype=np.uint8)).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0.5]\nnegate: 0\n')
    with pytest.raises(ValueError):
        LikelihoodMap.from_yaml(tmp_path / 'map.yaml')


def test_select_beams_drops_invalid_and_max_range():
    r = np.array([np.nan, 0.05, 2.0, np.inf, 9.5, 3.0])
    ranges, angles = select_beams(r, -1.0, 0.1, 6, 0.15, 9.5)
    assert ranges.tolist() == [2.0, 3.0]
    assert angles == pytest.approx([-1.0 + 0.2, -1.0 + 0.5])


def test_predict_without_motion_adds_no_noise_and_with_motion_moves_the_cloud():
    pf = ParticleFilter(n=2000, seed=1)
    pf.init_gaussian((1.0, 1.0, math.pi / 2), 0.0, 0.0)
    pf.predict((0.0, 0.0, 0.0))
    assert np.allclose(pf.p, [1.0, 1.0, math.pi / 2])
    pf.predict((1.0, 0.0, 0.0))                    # 1 m forward along +y
    assert pf.estimate()[:2] == pytest.approx((1.0, 2.0), abs=0.02)
    assert 0.02 < pf.spread()[0] < 0.3


def test_stationary_updates_converge_from_a_known_start(lmap):
    """What the node does before driving: the start pose is known to ~15 cm, the car waits for 'localized'."""
    rng = np.random.default_rng(0)
    start = lap()[0]
    pf = ParticleFilter(n=1000, seed=0)
    pf.init_gaussian(start, 0.15, 0.1)
    r, a = select_beams(scan_from(lmap, start, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    for _ in range(8):
        pf.update_sensor(r, a, LASER, lmap)
        pf.resample()
    pos, yaw = pf.spread()
    assert pos < 0.1 and yaw < 0.08
    assert pose_error(start, pf.estimate())[0] < 0.1


def test_tracks_a_lap(lmap):
    rng = np.random.default_rng(1)
    poses = lap()
    pf = ParticleFilter(n=1000, seed=1)
    pf.init_gaussian(poses[0], 0.15, 0.1)
    errors = [pose_error(t, e) for t, e in drive(pf, lmap, poses, rng, odom_bias=1.0)]
    assert max(e[0] for e in errors) < 0.2 and max(e[1] for e in errors) < 0.1
    assert np.mean([e[0] for e in errors]) < 0.08


def test_tracks_a_lap_with_3_percent_odometry_scale_error(lmap):
    """The corridor straights only constrain the lateral position, so the along-track error grows between
    features (the notch, the corners) and is pulled back there: bounded, not zero."""
    rng = np.random.default_rng(1)
    poses = lap()
    pf = ParticleFilter(n=1000, seed=1)
    pf.init_gaussian(poses[0], 0.15, 0.1)
    errors = [pose_error(t, e) for t, e in drive(pf, lmap, poses, rng, odom_bias=1.03)]
    assert max(e[0] for e in errors) < 0.35 and max(e[1] for e in errors) < 0.15
    assert np.mean([e[0] for e in errors]) < 0.12


@pytest.fixture(scope='module')
def matcher(lmap):
    return ScanMatcher(lmap)


AMBIGUOUS = 0.97   # a second hypothesis fitting this well relative to the best one: the scan cannot tell


def contenders(hyps):
    return [h for h in hyps if h[1] >= AMBIGUOUS * hyps[0][1]]


@pytest.mark.parametrize('i', [0, 60, 140, 230, 330])
def test_global_search_never_misses_the_true_pose(lmap, matcher, i):
    """Standing still on a uniform straight with one corner in range, several places on the loop give the same
    scan (the caller breaks that tie with the last known pose); the true pose must be among them."""
    rng = np.random.default_rng(i)
    pose = lap()[i]
    r, a = select_beams(scan_from(lmap, pose, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    hyps = matcher.search_global(r, a, LASER)
    assert hyps[0][1] > 0.8
    assert any(pose_error(pose, h)[0] < 0.15 and pose_error(pose, h)[1] < math.radians(3)
               for h, _ in contenders(hyps))


@pytest.mark.parametrize('pose', [(9.75, 1.45, 0.0), (10.0, 1.5, math.radians(100)), (16.53, 10.55, math.pi)])
def test_global_search_is_unambiguous_with_a_feature_in_view(lmap, matcher, pose):
    """Beside the notch (also turned across the track, as after a crash), and at the stub on the top straight."""
    rng = np.random.default_rng(2)
    r, a = select_beams(scan_from(lmap, pose, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    hyps = matcher.search_global(r, a, LASER)
    assert len(contenders(hyps)) == 1
    assert pose_error(pose, hyps[0][0])[0] < 0.1 and pose_error(pose, hyps[0][0])[1] < math.radians(3)


def test_global_search_reports_a_symmetric_track_as_ambiguous():
    bare = make_track(features=False)
    rng = np.random.default_rng(4)
    pose = lap()[200]   # in a corner: on a straight, every position along it looks the same
    r, a = select_beams(scan_from(bare, pose, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    hyps = ScanMatcher(bare).search_global(r, a, LASER)
    assert len(contenders(hyps)) >= 2
    twin = (20.0 - pose[0], 12.0 - pose[1], float(wrap(pose[2] + math.pi)))   # the half-turn image
    assert any(pose_error(p, h)[0] < 0.15 for h, _ in hyps[:2] for p in (pose, twin))


def test_fit_drops_when_the_pose_is_off(lmap, matcher):
    rng = np.random.default_rng(5)
    pose = lap()[100]
    r, a = select_beams(scan_from(lmap, pose, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    good = matcher.fit(pose, r, a, LASER)
    off = matcher.fit(compose(pose, (0.0, 0.4, 0.0)), r, a, LASER)
    assert good > 0.8 and off < 0.6 * good


def test_local_search_reanchors_a_filter_thrown_off_by_the_odometry(lmap, matcher):
    """A 40 cm along-track jump the odometry did not see (wheel slip, a stalled sensor stream): the cloud
    sits on the old pose, the local search finds the true one."""
    rng = np.random.default_rng(6)
    poses = lap()
    true = poses[300]                       # on a straight: the hard direction for the filter
    stale = compose(true, (-0.4, 0.05, math.radians(4)))
    r, a = select_beams(scan_from(lmap, true, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    assert matcher.fit(stale, r, a, LASER) < 0.6 * matcher.fit(true, r, a, LASER)
    best, fit = matcher.search_local(stale, r, a, LASER)
    assert pose_error(true, best)[0] < 0.08 and pose_error(true, best)[1] < math.radians(2)
    assert fit > 0.8


MAP_ORIGIN = (0.0, 0.0, 0.0)


def global_hyps(lmap, matcher, pose, seed):
    """search_global's hypotheses for a scan at pose, and the refine callable choose_pose takes."""
    rng = np.random.default_rng(seed)
    r, a = select_beams(scan_from(lmap, pose, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    return matcher.search_global(r, a, LASER), lambda q: matcher.search_local(q, r, a, LASER, radius=1.0,
                                                                              yaw_range=math.radians(10))


@pytest.mark.parametrize('i', [0, 100, 200, 300, 340, 360, 380, 400, 420])
def test_choose_pose_resolves_ties_with_a_good_prior(lmap, matcher, i):
    """The review's stuck cases (340: the same spot facing the other way; 360-400: along the top straight):
    with the car's last pose as the prior, a pick is made and it is the right one."""
    pose = lap()[i]
    hyps, refine = global_hyps(lmap, matcher, pose, i)
    choice = choose_pose(hyps, [pose, MAP_ORIGIN], refine=refine)
    assert choice is not None
    assert pose_error(pose, choice[0])[0] < 0.15 and pose_error(pose, choice[0])[1] < math.radians(3)


def test_choose_pose_waits_on_a_useless_prior_then_guesses_with_alternatives(lmap, matcher):
    pose = lap()[380]                                  # top straight: many places fit
    hyps, refine = global_hyps(lmap, matcher, pose, 7)
    far = (10.0, 6.0, 0.0)                             # inside the island: near nothing
    assert choose_pose(hyps, [far], refine=refine) is None
    pick, alternatives, _ = choose_pose(hyps, [far], force=True)
    assert alternatives
    assert any(pose_error(pose, p)[0] < 0.15 for p in [pick] + alternatives)


def test_choose_pose_takes_a_unique_best_regardless_of_the_prior(lmap, matcher):
    pose = (9.75, 1.45, 0.0)                           # beside the notch
    pick, alternatives, _ = choose_pose(global_hyps(lmap, matcher, pose, 2)[0], [(18.0, 10.0, 3.0)])
    assert pose_error(pose, pick)[0] < 0.1 and alternatives == []


def test_alternatives_switch_to_the_true_pose_once_the_car_moves(lmap, matcher):
    """A wrong guess 1.2 m along the bottom straight, with the notch coming up: carried along with the true
    alternative, the scans rule it out within a few metres of driving."""
    rng = np.random.default_rng(8)
    poses = lap()
    start = 45
    true = poses[start]
    wrong = compose(true, (1.2, 0.0, 0.0))             # 1.2 m further along the same straight
    alts = Alternatives([true], odom_pose=true)        # odometry frame = map frame here
    switched = None
    for k in range(start + 3, start + 60, 3):
        moved = relative(poses[start], poses[k])
        r, a = select_beams(scan_from(lmap, poses[k], rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
        switched = alts.update(matcher, compose(wrong, moved), poses[k], r, a, LASER)
        if switched or alts.done:
            break
    assert switched is not None
    assert pose_error(poses[k], switched[0])[0] < 0.1


def test_alternatives_keep_a_correct_pick(lmap, matcher):
    rng = np.random.default_rng(9)
    poses = lap()
    start = 45
    alts = Alternatives([compose(poses[start], (1.2, 0.0, 0.0))], odom_pose=poses[start])
    for k in range(start + 3, start + 100, 3):
        r, a = select_beams(scan_from(lmap, poses[k], rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
        assert alts.update(matcher, poses[k], poses[k], r, a, LASER) is None
        if alts.done:
            break
    assert alts.done


HYBRID_MIN_FIT = 0.8   # mcl_node's defaults: a track-only result fitting worse falls back to the full search
TRACK_SEEDS = 15


@pytest.fixture(scope='module')
def track_matcher(lmap):
    m = ScanMatcher(lmap)
    m.set_track([p[:2] for p in lap()])
    return m


def test_track_search_scores_far_fewer_poses(track_matcher):
    full = len(track_matcher.candidates) * len(np.arange(-math.pi, math.pi, track_matcher.yaw_step))
    assert track_matcher.track_poses < full / 5


@pytest.mark.parametrize('i', [0, 92, 200, 300, 395, 480])
def test_track_search_finds_a_car_on_the_raceline(lmap, track_matcher, i):
    pose = lap()[i]
    rng = np.random.default_rng(i)
    r, a = select_beams(scan_from(lmap, pose, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    hyps = track_matcher.search_global(r, a, LASER, track_only=True, seeds=TRACK_SEEDS)
    assert hyps[0][1] >= HYBRID_MIN_FIT
    assert any(pose_error(pose, h)[0] < 0.15 and pose_error(pose, h)[1] < math.radians(3)
               for h, _ in contenders(hyps))


@pytest.mark.parametrize('pose', [(10.0, 1.5, math.radians(100)),       # turned across the bottom straight
                                  (9.75, 1.45, math.pi),                # facing backwards beside the notch
                                  (18.55, 6.0, -math.pi / 2)])           # facing backwards on the right side
def test_track_search_does_not_confidently_misplace_a_turned_car(lmap, track_matcher, pose):
    """After a crash the car can face any way. The track-only search must then either fit poorly (the node falls
    back to the full search) or still contain the true pose among its ties, never a confident wrong pose."""
    rng = np.random.default_rng(11)
    r, a = select_beams(scan_from(lmap, pose, rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    hyps = track_matcher.search_global(r, a, LASER, track_only=True, seeds=TRACK_SEEDS)
    if hyps and hyps[0][1] >= HYBRID_MIN_FIT:
        assert any(pose_error(pose, h)[0] < 0.15 and pose_error(pose, h)[1] < math.radians(5)
                   for h, _ in contenders(hyps)), f'confident wrong pose {hyps[0]}'


def half_turn(pose):
    """The pose a half turn about the track's centre: on the bare loop it sees exactly what pose sees."""
    return 20.0 - pose[0], 12.0 - pose[1], float(wrap(pose[2] + math.pi))


def test_alternatives_keep_a_look_alike_the_scans_cannot_tell_apart():
    """Bare (symmetric) loop: the pick and its half-turn look-alike fit the same all the way. That is no evidence
    either way: the look-alike must still be checked after 20 m (it used to be dropped as 'ruled out' after 5)."""
    bare = make_track(features=False)
    m = ScanMatcher(bare)
    rng = np.random.default_rng(3)
    poses = lap()
    start = 30
    alts = Alternatives([half_turn(poses[start])], odom_pose=poses[start])
    for k in range(start + 3, start + 250, 3):        # ~20 m
        r, a = select_beams(scan_from(bare, poses[k], rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
        assert alts.update(m, poses[k], poses[k], r, a, LASER) is None
    assert not alts.done and len(alts.poses) == 1 and alts.ruled_out == 0


def test_alternatives_switch_from_a_half_turn_look_alike_at_the_first_feature(lmap, matcher):
    """The pick is the half-turn look-alike, the true pose an alternative: driving past the notch decides it."""
    rng = np.random.default_rng(4)
    poses = lap()
    start = 40                                         # bottom straight, the notch (x 9-10.5) ahead
    alts = Alternatives([poses[start]], odom_pose=poses[start])
    wrong = half_turn(poses[start])
    switched = None
    for k in range(start + 3, start + 150, 3):
        moved = relative(poses[start], poses[k])
        r, a = select_beams(scan_from(lmap, poses[k], rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
        switched = alts.update(matcher, compose(wrong, moved), poses[k], r, a, LASER)
        if switched:
            break
    assert switched is not None and pose_error(poses[k], switched[0])[0] < 0.15


def test_particles_off_the_map_or_on_unseen_ground_are_penalized(lmap):
    """Nothing in a scan contradicts a cloud outside the map (its beams end in no-man's land): the position itself
    must count. Half the cloud on the track, half 3 m outside the map: the outside half loses its weight."""
    pf = ParticleFilter(n=200, seed=1)
    pf.init_gaussian((1.45, 6.0, math.pi / 2), 0.01, 0.01)
    pf.p[100:, 0] -= 4.0                               # x = -2.55: outside the 20 x 12 m map
    rng = np.random.default_rng(1)
    r, a = select_beams(scan_from(lmap, (1.45, 6.0, math.pi / 2), rng), ANGLE_MIN, ANGLE_INC, 60, 0.15, RANGE_MAX)
    pf.update_sensor(r, a, LASER, lmap)
    assert pf.w[100:].sum() < 0.01
    assert list(lmap.cell(np.array([1.45, -2.0, 0.1, 10.0]), np.array([6.0, 6.0, 6.0, 6.0]))) == [0, 2, 2, 2]


def test_track_search_candidates_match_the_brute_force_nearest_line_point(lmap):
    """set_track uses a k-d tree (the candidates x line matrix took 470 MB on iros2026_v2): same groups."""
    line = np.array([p[:2] for p in lap()])
    m = ScanMatcher(lmap)
    m.set_track(line, radius=0.6)
    d = np.hypot(m.candidates[:, None, 0] - line[None, :, 0], m.candidates[:, None, 1] - line[None, :, 1])
    near = d.min(axis=1) <= 0.6
    assert m.track_poses == int(near.sum()) * len(m.track_groups[0][1])
    assert sum(len(g[0]) for g in m.track_groups) == int(near.sum())
