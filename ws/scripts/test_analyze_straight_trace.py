"""Check candidate-clock analysis against synthetic constant-speed motion."""
import pytest
from analyze_straight_trace import compute


def test_simulation_clock_explains_motion_when_wall_clock_differs():
    frames=[]
    for i in range(8):
        frames.append(dict(ros_stamp_ns=i*100_000_000, arrival_monotonic_ns=i*100_000_000,
                           bridge_work_ns=2_000_000, payload={
                               'V1 Orientation Quaternion':'0 0 0 1',
                               'V1 Position':f'{i*.1} 0 0', 'V1 Lap Time':str(i*.05),
                               'V1 Linear Velocity':'2 0 0',
                               'V1 Encoder Angles':f'{i*.1/.0585} {i*.1/.0585}'}))
    phases={r['ros_stamp_ns']:'accelerate_cruise' for r in frames}
    report, rows=compute(frames, phases)
    assert report['cruise']['intervals'] == 7
    assert report['cruise']['position_speed_over_reported_wall']['median'] == pytest.approx(.5)
    assert report['cruise']['position_speed_over_reported_lap']['median'] == pytest.approx(1.)
    assert report['cruise']['wheel_excess_pct']['median'] == pytest.approx(0.,abs=1e-10)
    phases[frames[-1]['ros_stamp_ns']]='braking'
    report, rows=compute(frames, phases)
    assert report['all']['intervals'] == 6  # do not mix samples across phase boundaries
