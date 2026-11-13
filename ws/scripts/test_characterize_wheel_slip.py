"""Synthetic checks of the offline slip measurement geometry and time independence."""
import numpy as np
import pytest
from characterize_wheel_slip import intervals


def straight(excess=0.):
    n = 30
    stamps = np.arange(n, dtype=np.int64)*50_000_000
    gt = np.zeros((n, 6))
    gt[:, 0] = np.arange(n)*.1
    gt[:, 3] = 2.
    angles = np.column_stack((gt[:, 0], gt[:, 0]))*(1+excess)/.0585
    return stamps, gt, angles


def test_no_slip_independent_of_wall_clock_pacing():
    stamps, gt, angles = straight()
    for scale in (1, 2):
        result, _ = intervals(stamps*scale, gt, angles)
        assert result['groups']['all']['distance_weighted_excess_pct'] == pytest.approx(0., abs=1e-10)


def test_known_overrotation():
    result, _ = intervals(*straight(.15))
    assert result['groups']['all']['median_excess_pct'] == pytest.approx(15.)


def test_turning_wheels_corrected_for_different_paths():
    n, radius, width = 30, 2., .3
    yaw = np.arange(n)*.05
    gt = np.column_stack((radius*np.sin(yaw), radius*(1-np.cos(yaw)), yaw,
                          np.full(n, 2.), np.zeros(n), np.ones(n)))
    angles = np.column_stack(((radius-width/2)*yaw, (radius+width/2)*yaw))/.0585
    report, _ = intervals(np.arange(n, dtype=np.int64)*50_000_000, gt, angles,
                          rear_track_width=width)
    for key in ('median_excess_pct', 'left_median_excess_pct', 'right_median_excess_pct'):
        assert report['groups']['all'][key] == pytest.approx(0., abs=1e-10)


def test_reset_and_stationary_intervals_excluded():
    stamps, gt, angles = straight()
    gt[15:, 0] -= 5
    angles[15:] -= 100
    report, _ = intervals(stamps, gt, angles)
    assert report['rejected_intervals'] == 1
    gt[:, 3] = 0.
    report, _ = intervals(stamps, gt, angles)
    assert report['groups']['all']['intervals'] == 0


def test_final_stop_remains_classified_after_command_stream_ends():
    stamps, gt, angles = straight()
    commands = [(0, (2.,)), (500_000_000, (0.,))]
    report, rows = intervals(stamps, gt, angles, commands)
    assert report['groups']['commanded_stop']['intervals'] == 19
    assert rows[-1]['speed_command_mps'] == 0.
