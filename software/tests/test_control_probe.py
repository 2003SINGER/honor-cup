"""Hardware-facing control entry contracts without requiring ROS or a robot."""

import math

import pytest

from m3pro_nav.control_probe import (
    MAX_DISTANCE_M, ProbeOptions, StraightTrial, command_gate, limit_command,
    validate_options,
)
from m3pro_nav.pose import Pose2D


def test_control_probe_is_read_only_by_default_and_motion_requires_verified_frames():
    assert not validate_options(ProbeOptions()).execute
    with pytest.raises(ValueError):
        validate_options(ProbeOptions(axis='x', distance=0.01))
    with pytest.raises(ValueError, match='frame'):
        validate_options(ProbeOptions(execute=True, axis='x', distance=0.01))
    with pytest.raises(ValueError, match='distance'):
        validate_options(ProbeOptions(execute=True, expected_odom_frame='odom',
            expected_base_frame='base_footprint', axis='x', distance=MAX_DISTANCE_M + 0.001))


@pytest.mark.parametrize('axis,expected', [('x', 'vx'), ('y', 'vy')])
def test_short_trial_keeps_body_yaw_fixed_with_non_cardinal_odom_heading(axis, expected):
    start = Pose2D(1.0, 2.0, math.pi / 4)
    trial = StraightTrial(start, axis=axis, distance=0.02)
    cmd = trial.sample(trial.duration / 3, start, (0.0, 0.0), 0.0)
    assert math.isfinite(cmd.vx) and math.isfinite(cmd.vy) and math.isfinite(cmd.wz)
    assert getattr(cmd, expected) > 0
    assert getattr(cmd, 'vy' if expected == 'vx' else 'vx') == pytest.approx(0.0, abs=1e-12)
    assert cmd.wz == pytest.approx(0.0, abs=1e-12)


def test_trial_terminal_reference_remains_available_for_closed_loop_settling():
    start = Pose2D(1.0, 2.0, math.pi / 4)
    trial = StraightTrial(start, axis='x', distance=0.02)
    cmd = trial.sample(trial.duration + 0.5, start, (0.0, 0.0), 0.0)
    assert cmd.vx > 0  # still corrects position after feedforward has stopped
    assert cmd.vy == pytest.approx(0.0, abs=1e-12)
    assert math.hypot(trial.target_pose.x - start.x,
                      trial.target_pose.y - start.y) == pytest.approx(0.02)


def test_command_limit_caps_translational_norm_and_yaw_rate():
    vx, vy, wz = limit_command(0.08, 0.08, 0.3)
    assert math.hypot(vx, vy) == pytest.approx(0.05)
    assert wz == pytest.approx(0.15)
    with pytest.raises(ValueError):
        limit_command(math.nan, 0.0, 0.0)
    with pytest.raises(ValueError):
        limit_command(1.1, 0.0, 0.0)


@pytest.mark.parametrize('odom,imu,foreign,allowed', [
    (10.0, 10.0, 0, True),
    (None, 10.0, 0, False),
    (10.0, None, 0, False),
    (9.0, 10.0, 0, False),
    (10.0, 9.0, 0, False),
    (10.0, 10.0, 1, False),
])
def test_control_gate_requires_fresh_feedback_and_exclusive_command_ownership(
        odom, imu, foreign, allowed):
    result, _ = command_gate(10.1, odom, imu, foreign)
    assert result is allowed
