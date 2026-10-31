import math

from control.input_health import InputHealth


def arm(guard, yaw=0.):
    for i in range(5):
        t=i*.1
        assert guard.advance(10+t,t,0.,True) is None
        result=guard.accept(yaw,(10+t,10+t),t)
    assert result is None


def test_clock_step_and_fresh_but_wrong_heading_cannot_rearm():
    guard=InputHealth()
    arm(guard, yaw=-1.5)
    assert guard.advance(12.96,.468,-.88,True) == 'clock discontinuity'
    for i in range(10):
        t=.5+i*.05
        guard.advance(12.992+t-.5,t,0.,True)
        assert guard.accept(-1.5+math.radians(119),(t,t),t) is not None
    # A stable wrong pose did not become the new reference. Correct poses can recover.
    for i in range(7):
        t=1.1+i*.05
        guard.advance(13.592+t-1.1,t,0.,True)
        result=guard.accept(-1.56,(t,t),t)
    assert result is None


def test_repeated_cached_samples_do_not_complete_recovery():
    guard=InputHealth()
    for i in range(10):
        t=i*.1
        guard.advance(10+t,t)
        assert guard.accept(0.,(10.,10.),t) is not None


def test_legitimate_turn_is_followed_by_measured_yaw_rate():
    guard=InputHealth()
    arm(guard)
    for i in range(1,20):
        t=.4+i*.05
        assert guard.advance(10+t,t,2.,True) is None
        assert guard.accept(i*.1,(10+t,10+t),t) is None


def test_recovery_needs_fresh_pose_and_fresh_odometry_not_one_stream_only():
    for stale_stream in (0,1):
        guard=InputHealth()
        for i in range(10):
            t=i*.1
            guard.advance(10+t,t)
            pair=[10+t,10+t]
            pair[stale_stream]=10.
            assert guard.accept(0.,tuple(pair),t) is not None


def test_initial_recovery_does_not_accept_a_sudden_heading_change():
    guard=InputHealth()
    guard.advance(10.,0.)
    assert guard.accept(0.,(10.,10.),0.) is not None
    for i in range(1,10):
        t=i*.1
        guard.advance(10+t,t)
        assert guard.accept(2.,(10+t,10+t),t) is not None


def test_recorded_hairpin_clock_and_heading_error_stays_stopped():
    # Repeat 4: 2.493 s in old ROS stamps versus 0.0683 s monotonic,
    # physical heading changes -3.31 deg, localization error becomes -118.77 deg.
    guard=InputHealth()
    arm(guard)
    dt=.0682722
    physical_turn=math.radians(-3.31)
    assert guard.advance(10.4+2.49335,.4+dt,physical_turn/dt,True) == 'clock discontinuity'
    bad_yaw=physical_turn+math.radians(-118.77)
    assert guard.accept(bad_yaw,(12.89335,12.89335),.4+dt) is not None
    assert guard.recovering
