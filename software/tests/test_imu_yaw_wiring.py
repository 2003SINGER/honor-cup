"""Pure-Python checks for IMU yaw wiring in the ground-loop diagnostic."""

import math

import pytest

from m3pro_nav.ground_loop_trial import (control_imu_log_fields,
                                         control_yaw_log_fields, parser,
                                         require_fresh_imu_yaw,
                                         yaw_feedback_state)
from m3pro_nav.odometry_adapter import OdometryState
from m3pro_nav.pose import Pose2D
from m3pro_nav.position_controller import PositionController
from m3pro_nav.relative_yaw import ImuTimingError, RelativeYawEstimator
from m3pro_nav.trajectory_reference import ReferenceState


def make_estimator():
    estimator = RelativeYawEstimator(stationary_sample_count=2,
        minimum_bias_duration_s=0.04, maximum_sample_gap_s=0.16)
    estimator.add_stationary_sample(1.00, 0.01)
    estimator.add_stationary_sample(1.04, 0.01)
    estimator.start(0.10, 1.04)
    return estimator


def test_imu_relative_yaw_changes_the_yaw_p_controller_feedback():
    estimator = make_estimator()
    estimator.update(1.08, 0.21)
    estimator.update(1.12, 0.21)
    odometry = OdometryState(Pose2D(1.2, -0.3, 0.10), 1.0, 0.0,
        -0.4, 1.10, 'odom', 'base_footprint')

    measured = yaw_feedback_state(odometry, 'imu', estimator)
    reference = ReferenceState(1.2, -0.3, 0.4, 0.0, 0.0,
                               1.0, 0.0, 0.0, 0.0)
    command = PositionController(kp_pos=0.0, kd_vel=0.0,
                                  kp_yaw=2.0, kd_yaw=0.2).update(
        reference, measured.pose,
        measured_velocity_world=(measured.vx_world, measured.vy_world),
        yaw_rate=measured.wz)

    assert measured.pose.yaw == pytest.approx(0.112)
    assert measured.wz == pytest.approx(0.20)
    assert command.wz == pytest.approx(2.0 * (0.4 - 0.112) - 0.2 * 0.2)
    # Odom position is kept; its world velocity is rotated by the yaw delta.
    assert (measured.pose.x, measured.pose.y) == (1.2, -0.3)
    assert measured.vx_world == pytest.approx(math.cos(0.012))
    assert measured.vy_world == pytest.approx(math.sin(0.012))


def test_odom_yaw_source_preserves_the_existing_feedback_state():
    odometry = OdometryState(Pose2D(0.2, 0.4, -0.8), 0.3, 0.1,
                             0.2, 2.0, 'odom', 'base_footprint')
    assert yaw_feedback_state(odometry, 'odom') is odometry


def test_invalid_or_stale_imu_fails_closed_instead_of_falling_back():
    odometry = OdometryState(Pose2D(0.0, 0.0, 1.0), 0.0, 0.0,
                             0.0, 2.0, 'odom', 'base_footprint')
    estimator = RelativeYawEstimator(stationary_sample_count=2,
                                     minimum_bias_duration_s=0.04)

    with pytest.raises(RuntimeError, match='refusing odometry fallback'):
        yaw_feedback_state(odometry, 'imu', estimator)
    estimator = make_estimator()
    with pytest.raises(RuntimeError, match='missing or stale'):
        require_fresh_imu_yaw(estimator, None, 2.0)
    with pytest.raises(RuntimeError, match='missing or stale'):
        require_fresh_imu_yaw(estimator, 1.0, 1.17)

    with pytest.raises(RuntimeError, match='missing or stale'):
        require_fresh_imu_yaw(estimator, 1.0, 2.0, maximum_age_s=0.5)


def test_source_gap_invalidates_imu_yaw_and_blocks_control_state():
    estimator = make_estimator()
    with pytest.raises(ImuTimingError, match='invalidated relative yaw'):
        estimator.update(1.21, 0.2)

    odometry = OdometryState(Pose2D(0.0, 0.0, 0.0), 0.0, 0.0,
                             0.0, 1.21, 'odom', 'base_footprint')
    with pytest.raises(RuntimeError, match='refusing odometry fallback'):
        yaw_feedback_state(odometry, 'imu', estimator)


def test_imu_yaw_mode_is_explicit_and_odom_remains_default():
    base = ['--run', '--expected-odom-frame', 'odom',
            '--expected-base-frame', 'base_footprint']
    assert parser().parse_args(base).yaw_source == 'odom'
    assert parser().parse_args(base + ['--yaw-source', 'imu']).yaw_source == 'imu'


def test_control_sample_diagnostic_fields_can_be_written_together():
    """The real control sample expands both mappings as keyword arguments."""
    estimator = make_estimator()
    feedback = OdometryState(Pose2D(0.0, 0.0, 0.1), 0.0, 0.0,
        0.0, 1.04, 'odom', 'base_footprint')
    yaw = control_yaw_log_fields('imu', feedback, estimator)
    imu = control_imu_log_fields((1.04, 'imu_frame', 9.8, 0.01, 2.0),
                                 estimator, 2.02)

    assert set(yaw).isdisjoint(imu)
    assert dict(**yaw, **imu)['imu_bias_radps'] == estimator.bias_radps
