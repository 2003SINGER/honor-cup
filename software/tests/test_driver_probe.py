import math

import pytest

from m3pro_nav.driver_probe import (
    MAX_LINEAR_STEP,
    MAX_STEP_DURATION,
    MAX_YAW_STEP,
    ProbeOptions,
    execution_gate,
    twist_values,
    validate_options,
)


def test_default_mode_is_read_only_and_does_not_require_motion_values():
    options = validate_options(ProbeOptions())
    assert options.execute is False


@pytest.mark.parametrize('options', [
    ProbeOptions(axis='x', magnitude=0.01, duration=0.1),
    ProbeOptions(execute=True, magnitude=0.01, duration=0.1),
    ProbeOptions(execute=True, axis='x', duration=0.1),
    ProbeOptions(execute=True, axis='x', magnitude=0.01),
    ProbeOptions(execute=True, axis='x', magnitude=MAX_LINEAR_STEP + 1e-9,
                 duration=0.1),
    ProbeOptions(execute=True, axis='x', magnitude=-MAX_LINEAR_STEP - 1e-9,
                 duration=0.1),
    ProbeOptions(execute=True, axis='yaw', magnitude=MAX_YAW_STEP + 1e-9,
                 duration=0.1),
    ProbeOptions(execute=True, axis='x', magnitude=0.01,
                 duration=MAX_STEP_DURATION + 1e-9),
    ProbeOptions(execute=True, axis='x', magnitude=math.nan, duration=0.1),
    ProbeOptions(execute=True, axis='x', magnitude=0.0, duration=0.1),
])
def test_unsafe_or_incomplete_motion_arguments_are_rejected(options):
    with pytest.raises(ValueError):
        validate_options(options)


@pytest.mark.parametrize(('axis', 'expected'), [
    ('x', (0.02, 0.0, 0.0)),
    ('y', (0.0, 0.02, 0.0)),
    ('yaw', (0.0, 0.0, 0.02)),
])
def test_axis_step_sets_only_one_twist_component(axis, expected):
    assert twist_values(axis, 0.02) == expected


def test_negative_magnitude_requests_the_opposite_direction():
    assert twist_values('x', -0.02) == (-0.02, 0.0, 0.0)


def test_exact_safety_caps_are_allowed():
    assert validate_options(ProbeOptions(
        execute=True, axis='x', magnitude=MAX_LINEAR_STEP,
        duration=MAX_STEP_DURATION)).execute
    assert validate_options(ProbeOptions(
        execute=True, axis='yaw', magnitude=-MAX_YAW_STEP,
        duration=MAX_STEP_DURATION)).execute


@pytest.mark.parametrize(('odom', 'imu', 'foreign', 'allowed'), [
    (10.0, 10.0, 0, True),
    (None, 10.0, 0, False),
    (10.0, None, 0, False),
    (9.0, 10.0, 0, False),
    (10.0, 9.0, 0, False),
    (10.0, 10.0, 1, False),
])
def test_execution_gate_requires_fresh_odom_imu_and_no_other_cmd_publishers(
        odom, imu, foreign, allowed):
    result, _reason = execution_gate(10.1, odom, imu, foreign)
    assert result is allowed
