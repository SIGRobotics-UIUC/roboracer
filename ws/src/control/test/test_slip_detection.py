"""Unit tests for the ROS-independent slip and friction math."""
import math

import pytest

from control.slip_detection import (
    CommandMotion, FrictionEstimator, PoseMotionEstimator, SlipDetector)


def test_pose_motion_estimates_body_speed_and_wrapped_yaw_rate():
    estimator = PoseMotionEstimator(smoothing=1.0)
    assert estimator.update(1.0, 0.0, 0.0, math.radians(179)) is None
    motion = estimator.update(1.1, -0.1, 0.0, math.radians(-179))
    assert motion.speed == pytest.approx(1.0, abs=0.02)
    assert motion.yaw_rate == pytest.approx(math.radians(20))


def test_bad_pose_interval_restarts_estimation():
    estimator = PoseMotionEstimator(smoothing=1.0, max_dt=0.2)
    assert estimator.update(1.0, 0.0, 0.0, 0.0) is None
    assert estimator.update(1.5, 0.5, 0.0, 0.0) is None
    motion = estimator.update(1.6, 0.6, 0.0, 0.0)
    assert motion.speed == pytest.approx(1.0)


def test_command_speed_is_rate_limited_from_measured_motion():
    command = CommandMotion(max_acceleration=2.5, max_deceleration=5.0)
    assert command.update(4.0, 1.0, 0.1) == pytest.approx(1.25)
    assert command.update(4.0, 1.25, 0.1) == pytest.approx(1.5)
    assert command.update(0.0, 1.5, 0.1) == pytest.approx(1.0)


def test_detector_combines_linear_and_angular_residuals_with_hysteresis():
    detector = SlipDetector(0.33, 0.5, 0.4, residual_smoothing=1.0,
                            trigger_samples=2, clear_samples=2)
    nominal = detector.update(2.0, 0.1, 2.0, 2.0 * math.tan(0.1) / 0.33)
    assert not nominal.slipping

    first = detector.update(2.0, 0.1, 1.0, nominal.expected_yaw_rate)
    assert first.linear_slip and not first.angular_slip and not first.slipping
    second = detector.update(2.0, 0.1, 1.0, nominal.expected_yaw_rate)
    assert second.slipping

    held = detector.update(2.0, 0.1, 2.0, nominal.expected_yaw_rate)
    assert held.slipping
    cleared = detector.update(2.0, 0.1, 2.0, nominal.expected_yaw_rate)
    assert not cleared.slipping

    angular = detector.update(2.0, 0.2, 2.0, 0.0)
    assert angular.angular_slip


def test_friction_is_maximum_planar_acceleration_only_while_not_slipping():
    friction = FrictionEstimator(gravity=10.0)
    assert friction.update(3.0, 4.0) == pytest.approx(0.5)
    assert friction.update(9.0, 0.0, slipping=True) == pytest.approx(0.5)
    assert friction.update(8.0, 0.0, valid=False) == pytest.approx(0.5)
    assert friction.update(6.0, 8.0) == pytest.approx(1.0)
    friction.reset()
    assert friction.estimate is None


@pytest.mark.parametrize('factory', [
    lambda: PoseMotionEstimator(smoothing=0.0),
    lambda: CommandMotion(max_acceleration=0.0),
    lambda: SlipDetector(0.0, 1.0, 1.0),
    lambda: FrictionEstimator(gravity=float('nan')),
])
def test_invalid_configuration_is_rejected(factory):
    with pytest.raises(ValueError):
        factory()
