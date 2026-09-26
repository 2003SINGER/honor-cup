import math

from m3pro_nav.plant import (
    IdealPlant, LowerLoopPlant, LOWER_PERIOD_S, MAX_MOTOR_PULSE,
    MAX_WHEEL_MM_S, MOTOR_IGNORE_PULSE, _body_twist, _wheel_speeds_mm_s,
)
from m3pro_nav.pose import Pose2D, Twist2D


def test_ideal_plant_integrates_constant_body_twist():
    plant = IdealPlant(Pose2D(0.0, 0.0, 0.0))
    result = plant.step(Twist2D(1.0, 0.0, math.pi / 2), 1.0)
    assert math.isclose(result.pose.x, 2.0 / math.pi)
    assert math.isclose(result.pose.y, 2.0 / math.pi)
    assert math.isclose(result.pose.yaw, math.pi / 2)
    assert result.twist == Twist2D(1.0, 0.0, math.pi / 2)


def test_lower_plant_clamps_four_wheel_targets_and_pwm():
    plant = LowerLoopPlant()
    result = plant.step(Twist2D(0.7, 0.7, 4.2), LOWER_PERIOD_S)
    assert all(abs(v) <= MAX_WHEEL_MM_S for v in result.wheel_target_mm_s)
    assert all(abs(v) <= MAX_MOTOR_PULSE for v in result.pwm)
    assert len(result.encoder_delta) == 4


def test_lower_plant_accumulates_partial_period_without_extra_tick():
    plant = LowerLoopPlant()
    first = plant.step(Twist2D(0.2, 0.0, 0.0), LOWER_PERIOD_S / 2)
    assert first.pose == Pose2D(0.0, 0.0, 0.0)
    second = plant.step(Twist2D(0.2, 0.0, 0.0), LOWER_PERIOD_S / 2)
    assert second.pose.x > 0.0
    assert math.isclose(second.pose.x, plant.twist.vx * LOWER_PERIOD_S)


def test_lower_plant_is_deterministic():
    command = Twist2D(0.3, -0.1, 0.4)
    left, right = LowerLoopPlant(), LowerLoopPlant()
    for _ in range(20):
        a = left.step(command, LOWER_PERIOD_S)
        b = right.step(command, LOWER_PERIOD_S)
    assert a == b


def test_firmware_wheel_signs_and_forward_inverse_roundtrip():
    lateral = _wheel_speeds_mm_s(Twist2D(0.0, 0.2, 0.0))
    assert lateral == (-200.0, 200.0, 200.0, -200.0)
    recovered = _body_twist(lateral)
    assert recovered == Twist2D(0.0, 0.2, 0.0)
    spin = _wheel_speeds_mm_s(Twist2D(0.0, 0.0, 1.0))
    assert spin == (-167.0, -167.0, 167.0, 167.0)
    assert _body_twist(spin) == Twist2D(0.0, 0.0, 1.0)


def test_lower_applies_pwm_deadzone_compensation_and_zero_command_reset():
    plant = LowerLoopPlant()
    moving = plant.step(Twist2D(0.2, 0.0, 0.0), LOWER_PERIOD_S)
    assert all(MOTOR_IGNORE_PULSE < pulse <= MAX_MOTOR_PULSE
               for pulse in moving.pwm)
    stopped = plant.step(Twist2D(), LOWER_PERIOD_S)
    assert stopped.wheel_target_mm_s == (0, 0, 0, 0)
    assert stopped.pwm == (0.0, 0.0, 0.0, 0.0)
