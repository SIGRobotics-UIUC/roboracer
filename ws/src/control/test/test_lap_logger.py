"""Tests for the lap logger: LapTracker (control.lap_stats) and lap_report unit tests, plus a functional test
that runs lap_logger_node, publishes the poses of a car driving laps, and checks the CSV it writes.

Needs a sourced ROS 2 environment with the control package built (the unit tests only need numpy):
    python3 -m pytest ws/src/control/test/test_lap_logger.py
"""
import csv
import json
import math
import os
import random
import signal
import subprocess
import time

import numpy as np
import pytest

from control.lap_stats import LapTracker, summary

R = 4.0                                 # circle raceline of radius 4 m, s = 0 at (R, 0), counter-clockwise
L = 2 * math.pi * R


def circle(n=400):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([R * np.cos(a), R * np.sin(a)])


def drive(tracker, laps, speed=2.0, rate=50.0, start=0.0, offset=0.0, t0=0.0, standstill=0.0):
    """Feed poses of a car on the circle (radius R + offset) from arc length `start`; returns (laps done, t)."""
    out, t = [], t0
    pos = start
    for _ in range(int(standstill * rate)):
        tracker.update(t, *point(pos, offset))
        t += 1 / rate
    while pos < start + laps * L:
        lap = tracker.update(t, *point(pos, offset))
        if lap is not None:
            out.append(lap)
        t += 1 / rate
        pos += speed / rate
    return out, t


def point(s, offset=0.0):
    a = s / R
    return (R + offset) * math.cos(a), (R + offset) * math.sin(a)


def test_project_gives_arc_length_and_offset():
    tr = LapTracker(circle())
    s, off = tr.project(*point(L / 4, 0.3))
    assert s == pytest.approx(L / 4, abs=0.02)
    assert off == pytest.approx(0.3, abs=0.01)


def test_standing_start_then_flying_laps():
    tr = LapTracker(circle())
    laps, _ = drive(tr, 3.2, speed=2.0, start=0.2, standstill=2.0)
    assert [lap.start for lap in laps] == ['standing', 'flying', 'flying']
    assert all(lap.status == 'ok' for lap in laps)
    # the standing lap is timed from when the car moved (not the 2 s wait) to the line; flying laps are full laps
    assert laps[0].time_s == pytest.approx((L - 0.2) / 2.0, abs=0.05)
    for lap in laps[1:]:
        assert lap.time_s == pytest.approx(L / 2.0, abs=0.03)       # interpolated at the line, not frame-rounded
        assert lap.distance_m == pytest.approx(L, rel=0.01)
        assert lap.mean_speed_mps == pytest.approx(2.0, rel=0.01)
        assert lap.max_speed_mps == pytest.approx(2.0, rel=0.02)


def test_max_speed_and_distance_ignore_uneven_pose_timing_and_jitter():
    # localizer poses arrive in bursts: a pose 1 ms after the last one, with 1 cm of noise, must not read as
    # 10 m/s; the speed is measured over SPEED_WINDOW
    tr = LapTracker(circle())
    rng = np.random.default_rng(0)
    t, pos, laps = 0.0, L / 2, []
    while pos < 2.2 * L:
        dt = 0.001 if rng.random() < 0.5 else 0.039
        t, pos = t + dt, pos + 2.0 * dt
        x, y = point(pos)
        lap = tr.update(t, x + rng.normal(0, 0.005), y + rng.normal(0, 0.005))
        laps += [lap] if lap else []
    assert len(laps) == 1
    assert laps[0].max_speed_mps == pytest.approx(2.0, abs=0.3)
    assert laps[0].distance_m == pytest.approx(L, rel=0.02)     # jitter does not add up


def test_start_just_before_the_line_is_a_standing_start():
    tr = LapTracker(circle())
    laps, _ = drive(tr, 2.1, start=L - 0.05, standstill=1.0)
    assert [lap.start for lap in laps] == ['standing', 'flying']


def test_start_mid_track_waits_for_the_line():
    tr = LapTracker(circle())
    laps, _ = drive(tr, 2.0, start=L / 2)
    assert len(laps) == 1 and laps[0].start == 'flying'
    assert laps[0].time_s == pytest.approx(L / 2.0, abs=0.03)


def test_offset_statistics():
    tr = LapTracker(circle())
    laps, _ = drive(tr, 2.1, start=L / 2, offset=0.25)
    assert laps[0].mean_offset_m == pytest.approx(0.25, abs=0.01)
    assert laps[0].max_offset_m == pytest.approx(0.25, abs=0.01)


def test_jitter_and_reversing_at_the_line_count_no_lap():
    tr = LapTracker(circle())
    t = 0.0
    for i in range(2000):           # wobbling back and forth over the line
        assert tr.update(t, *point(0.3 * math.sin(i * 0.1))) is None
        t += 0.02
    assert not [lap for lap in tr.laps if lap.status == 'ok']


def test_pose_jump_aborts_the_lap_and_starts_over():
    tr = LapTracker(circle())
    laps, t = drive(tr, 1.6, start=L / 2)               # one lap, then 2.5 m into the next
    assert len(laps) == 1
    tr.update(t, *point(2.1 * L + 0.01))
    aborted = tr.update(t + 0.02, *point(L / 2))         # carried 8 m across the track
    assert aborted.status == 'aborted' and 'jumped' in aborted.note
    laps, _ = drive(tr, 1.05, t0=t + 0.04, start=0.02, standstill=1.0)
    assert len(laps) == 1 and laps[0].start == 'standing' and laps[0].number == 3
    assert summary(tr.laps) == f'2 laps, best {min(lap.time_s for lap in tr.laps if lap.status == "ok"):.2f} s, ' \
                               f'flying mean {tr.laps[0].time_s:.2f} s, 1 aborted'


def test_relocalization_jump_is_counted_not_aborted():
    tr = LapTracker(circle())
    laps, t = drive(tr, 0.8, start=L / 2)                 # timing from the line, 0.3 laps in
    tr.update(t, *point(1.3 * L + 1.0))                  # the localizer moves the pose 1 m ahead
    laps, _ = drive(tr, 0.7, t0=t + 0.02, start=1.3 * L + 1.0)
    assert len(laps) == 1 and laps[0].status == 'ok'
    assert laps[0].pose_jumps == 1 and laps[0].max_jump_m == pytest.approx(1.0, abs=0.05)
    assert laps[0].distance_m == pytest.approx(L - 1.0, rel=0.01)     # the jump is not driven distance
    assert laps[0].max_speed_mps == pytest.approx(2.0, rel=0.05)


def test_gap_in_the_poses_is_not_a_jump():
    tr = LapTracker(circle())
    laps, t = drive(tr, 0.8, start=L / 2)
    laps, _ = drive(tr, 0.75, t0=t + 1.0, start=1.3 * L + 2.0)    # no poses for 1 s while driving 2 m
    assert len(laps) == 1 and laps[0].pose_jumps == 0
    assert laps[0].distance_m == pytest.approx(L, rel=0.01)


def test_abort_without_a_lap_returns_none():
    tr = LapTracker(circle())
    assert tr.abort('reset') is None
    assert summary(tr.laps) == 'no finished laps, 0 aborted'


def write_run(path, header, rows):
    with open(path, 'w', newline='') as f:
        f.write('# run: ' + json.dumps(header) + '\n')
        w = csv.writer(f)
        w.writerow(['lap', 'status', 'start', 'time_s', 'distance_m', 'mean_speed_mps', 'max_speed_mps',
                    'mean_offset_m', 'max_offset_m', 'collisions', 'pose_jumps', 'max_jump_m', 'note'])
        w.writerows(rows)


def test_lap_report_compares_runs(tmp_path, monkeypatch, capsys):
    from control import lap_report
    runs = tmp_path / 'porto' / 'runs'
    runs.mkdir(parents=True)
    settings = {'controller': 'pure_pursuit', 'speed_scale': 0.5, 'localizer': 'mcl', 'avoid': False}
    write_run(runs / '20261105-100000.csv', settings, [
        [1, 'ok', 'standing', '9.000', '30', '3.3', '4', '0.100', '0.2', 0, 0, '0.00', ''],
        [2, 'ok', 'flying', '8.000', '30', '3.7', '4', '0.060', '0.2', 0, 1, '0.80', ''],
        [3, 'aborted', 'flying', '2.000', '5', '2.5', '3', '0.1', '0.3', 1, 0, '0.00',
         'sim reset: collision']])
    write_run(runs / '20261105-110000.csv', {'speed_scale': 1.0}, [])
    monkeypatch.setenv('TRACKS_DIR', str(tmp_path))
    assert lap_report.main(['porto']) == 0
    lines = capsys.readouterr().out.splitlines()
    assert lines[1].split() == ['20261105-100000', 'pure_pursuit', '0.5', 'mcl', 'false', '2', '1', '8.00', '8.00',
                                '0.08']
    assert lines[2].split() == ['20261105-110000', '-', '1.0', '-', '-', '0', '0', '-', '-', '-']
    assert lap_report.main(['porto', '--last', '1']) == 0
    assert len(capsys.readouterr().out.splitlines()) == 2
    assert lap_report.main(['nowhere']) == 1


# --- functional test of the node --------------------------------------------------------------------------

DOMAIN = random.randint(1, 101)  # not 0: the live stack (bridge, sim car) runs there


@pytest.fixture(scope='module')
def ros():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init(domain_id=DOMAIN)
    node = rclpy.create_node('lap_logger_test')
    yield rclpy, node
    node.destroy_node()
    rclpy.shutdown()


def test_node_writes_laps_and_aborts_on_sim_reset(ros, tmp_path):
    rclpy, node = ros
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Int32, String

    raceline = tmp_path / 'raceline.csv'
    xy = circle()
    with open(raceline, 'w') as f:
        f.write('# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n')
        for i, (x, y) in enumerate(xy):
            f.write(f'{i * L / len(xy):.4f}; {x:.5f}; {y:.5f}; 0; 0; 3; 0\n')
    out = tmp_path / 'runs'
    proc = subprocess.Popen(
        ['ros2', 'run', 'control', 'lap_logger_node', '--ros-args', '-p', f'raceline_path:={raceline}',
         '-p', f'out_dir:={out}', '-p', 'speed_scale:=1', '-p', 'localizer:=mcl'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True,
        env=dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN)))
    pose_pub = node.create_publisher(PoseStamped, '/localized_pose', 50)
    hits_pub = node.create_publisher(Int32, '/autodrive/roboracer_1/collision_count', 10)
    reset_pub = node.create_publisher(String, '/autodrive/roboracer_1/sim_reset', 10)
    try:
        end = time.time() + 20
        while pose_pub.get_subscription_count() == 0 or reset_pub.get_subscription_count() == 0:
            assert proc.poll() is None, 'node exited:\n' + proc.stdout.read()
            assert time.time() < end, 'lap_logger_node did not subscribe'
            rclpy.spin_once(node, timeout_sec=0.1)

        def pose(t, s):
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
            msg.pose.position.x, msg.pose.position.y = point(s)
            msg.pose.orientation.w = 1.0
            pose_pub.publish(msg)
            time.sleep(0.002)   # sim time in the stamps: the run takes far less than its laps

        hits_pub.publish(Int32(data=3))
        time.sleep(0.2)
        t, s = 100.0, L / 2
        while s < L / 2 + 3.5 * L:       # timing starts at the line, two flying laps, then half of a third
            pose(t, s)
            t, s = t + 0.02, s + 0.08     # 4 m/s
        hits_pub.publish(Int32(data=4))
        time.sleep(0.2)
        reset_pub.publish(String(data='collision count 3 -> 4'))
        end = time.time() + 5
        while time.time() < end and len(list(out.glob('*.csv'))) == 1 \
                and 'aborted' not in next(out.glob('*.csv')).read_text():
            time.sleep(0.1)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        log = proc.stdout.read()

    files = list(out.glob('*.csv'))
    assert len(files) == 1, log
    from control.lap_report import read_run
    header, rows = read_run(files[0])
    assert header['speed_scale'] == 1 and header['localizer'] == 'mcl'
    assert header['lap_length_m'] == pytest.approx(L, rel=0.001)
    assert [r['status'] for r in rows] == ['ok', 'ok', 'aborted'], log
    assert float(rows[1]['time_s']) == pytest.approx(L / 4.0, abs=0.03)
    assert rows[2]['collisions'] == '1' and 'collision count' in rows[2]['note']
    assert '2 laps, best' in log and '1 aborted' in log
