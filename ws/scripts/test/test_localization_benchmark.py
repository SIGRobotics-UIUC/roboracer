"""Tests of localization_benchmark's scoring (numpy only: no ROS, simulator or Docker)."""
import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np

SPEC = importlib.util.spec_from_file_location('localization_benchmark',
                                              Path(__file__).parents[1] / 'localization_benchmark.py')
b = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(b)

ORIGIN = (0.8, 3.15, -1.57)      # map origin in the world, as in tracks/porto/map_to_world.yaml


def circle(n=1000, r=4.0, rate=50.0):
    """(stamps, map-frame poses) of a car driving a circle counter-clockwise at 2 m/s."""
    t = np.arange(n) / rate
    a = 2.0 * t / r
    return t, np.column_stack([r * np.cos(a), r * np.sin(a), b.wrap(a + math.pi / 2)])


def to_world(p, origin=ORIGIN):
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    return np.column_stack([ox + c * p[:, 0] - s * p[:, 1], oy + s * p[:, 0] + c * p[:, 1], b.wrap(p[:, 2] + oyaw)])


def rows(t, loc, gt_t, gt_world, extra=()):
    out = [{'kind': 'loc', 'stamp': s, 'recv': s, 'a': x, 'b': y, 'c': h} for s, (x, y, h) in zip(t, loc)]
    out += [{'kind': 'gt', 'stamp': s, 'recv': s, 'a': x, 'b': y, 'c': h} for s, (x, y, h) in zip(gt_t, gt_world)]
    return out + list(extra)


class ScoringTests(unittest.TestCase):
    def test_world_to_map_inverts_the_origin(self):
        _, p = circle(50)
        np.testing.assert_allclose(b.to_map(to_world(p), ORIGIN), p, atol=1e-9)

    def test_perfect_localization_scores_zero(self):
        t, p = circle()
        s = b.score(rows(t[::2] + 0.003, p[::2] + [0, 0, 0], t, to_world(p)), ORIGIN)
        self.assertGreater(s['paired'], 450)
        self.assertLess(s['err_max'], 0.01)          # interpolated between ground-truth frames
        self.assertLess(s['head_p95'], 0.5)
        self.assertEqual((s['jumps'], s['gaps']), (0, 0))

    def test_constant_offset_is_measured(self):
        t, p = circle()
        s = b.score(rows(t, p + [0.1, 0.0, 0.0], t, to_world(p)), ORIGIN)
        self.assertAlmostEqual(s['err_mean'], 0.1, places=6)
        self.assertAlmostEqual(s['far_share'], 0.0)
        self.assertLess(s['aligned_mean'], 0.01)     # a pure shift is map placement, aligned away

    def test_rotated_map_is_aligned_away(self):
        t, p = circle()
        c, s_ = math.cos(0.05), math.sin(0.05)
        rotated = np.column_stack([c * p[:, 0] - s_ * p[:, 1], s_ * p[:, 0] + c * p[:, 1], b.wrap(p[:, 2] + 0.05)])
        s = b.score(rows(t, rotated, t, to_world(p)), ORIGIN)
        self.assertGreater(s['err_mean'], 0.15)      # 4 m x 0.05 rad
        self.assertLess(s['aligned_mean'], 1e-6)
        self.assertAlmostEqual(abs(s['fit_rot_deg']), math.degrees(0.05), places=3)

    def test_jitter_ignores_a_constant_offset_and_measures_a_wobble(self):
        t, p = circle()
        steady = b.score(rows(t, p + [0.1, 0.0, 0.0], t, to_world(p)), ORIGIN)
        self.assertLess(steady['jitter'], 1e-6)
        wobble = p.copy()
        wobble[:, 0] += 0.02 * np.where(np.arange(len(p)) % 2, 1.0, -1.0)   # +-2 cm sawtooth at 50 Hz
        s = b.score(rows(t, wobble, t, to_world(p)), ORIGIN)
        self.assertAlmostEqual(s['jitter'], 0.04, delta=0.005)

    def test_jumps_and_gaps(self):
        t, p = circle()
        loc = p.copy()
        loc[400:, 0] += 0.8                          # the localizer snaps 0.8 m between two poses
        keep = np.r_[0:600, 640:1000]                # and withholds the pose for 0.8 s
        s = b.score(rows(t[keep], loc[keep], t, to_world(p)), ORIGIN)
        self.assertEqual(s['jumps'], 1)
        self.assertAlmostEqual(s['max_jump'], 0.8, delta=0.05)
        self.assertEqual(s['gaps'], 1)
        self.assertAlmostEqual(s['gap_s'], 0.82, delta=0.01)

    def test_no_interpolation_across_a_reset(self):
        gt_t = np.array([0.0, 0.02, 0.04])
        gt = np.array([[0.0, 0.0, 0.0], [0.04, 0.0, 0.0], [5.0, 0.0, 0.0]])   # teleported back after 0.02 s
        truth, ok = b.interpolate(gt_t, gt, np.array([0.01, 0.03, 0.5]))
        self.assertEqual(ok.tolist(), [True, False, False])
        self.assertAlmostEqual(truth[0, 0], 0.02)

    def test_extrapolating_through_a_sim_stall_is_scored(self):
        # the sim freezes for 0.4 s with the car standing; the localizer extrapolates 0.8 m ahead meanwhile
        gt_t = np.r_[np.arange(0, 1.0, 0.02), np.arange(1.4, 2.0, 0.02)]
        gt = np.zeros((len(gt_t), 3))
        gt[:, 0] = np.where(gt_t < 1.0, gt_t, 1.0 + (gt_t - 1.4)) * 2.0     # 2 m/s, standing during the stall
        t = np.arange(0, 2.0, 0.01)
        loc = np.zeros((len(t), 3))
        loc[:, 0] = np.interp(t, gt_t, gt[:, 0])
        stall = (t > 1.0) & (t < 1.4)
        loc[stall, 0] = 2.0 * t[stall]                                     # runs on at 2 m/s
        rs = rows(t, loc, gt_t, to_world(gt))
        s = b.score(rs, ORIGIN)
        self.assertAlmostEqual(s['err_max'], 0.8, delta=0.03)
        self.assertEqual(s['jumps'], 1)
        self.assertEqual(s['stalls'], 1)
        self.assertAlmostEqual(s['stall_s'], 0.42, delta=0.01)

    def test_counters_ignore_resets(self):
        self.assertEqual(b.counter_delta([0, 1, 2, 0, 1]), 3)
        self.assertEqual(b.counter_delta([]), 0)

    def test_table_lists_every_run(self):
        t, p = circle(200)
        s = b.score(rows(t, p, t, to_world(p)), ORIGIN)
        text = b.table([('cartographer_1', s), ('mcl_1', {'jumps': 0})])
        self.assertIn('cartographer_1', text)
        self.assertEqual(len(text.splitlines()), 3)


if __name__ == '__main__':
    unittest.main()
