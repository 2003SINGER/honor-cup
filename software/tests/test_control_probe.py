"""Hardware-facing control entry contracts without requiring ROS or a robot."""

import math
import threading

import pytest

from m3pro_nav.control_probe import (
    MAX_COMMAND_DISTANCE_M, MAX_DISTANCE_M, CommandDistanceBudget,
    ProbeOptions, StraightTrial, command_gate, install_stop_signal_handlers,
    limit_command, next_control_deadline, pose_envelope_error,
    restore_signal_handlers,
    send_zero_window, shutdown_ros_context, validate_options,
    trial_settled,
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
    with pytest.raises(ValueError, match='minimum observable'):
        validate_options(ProbeOptions(execute=True, expected_odom_frame='odom',
            expected_base_frame='base_link', axis='x', distance=0.004))


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


def test_pose_envelope_wraps_yaw_error_across_angle_boundary():
    start = Pose2D(0.0, 0.0, math.pi - 0.01)
    measured = Pose2D(0.03, 0.04, -math.pi + 0.01)
    displacement, yaw_error = pose_envelope_error(start, measured)
    assert displacement == pytest.approx(0.05)
    assert yaw_error == pytest.approx(0.02)


def test_trial_success_requires_quantization_tolerant_aligned_progress_and_settled_speed():
    start = Pose2D(1.0, 2.0, math.pi / 4)
    trial = StraightTrial(start, axis='x', distance=0.01)
    target = trial.target_pose
    unchanged_error = math.hypot(target.x - start.x, target.y - start.y)
    # The old <=1cm goal check accepted this unchanged 1cm pose.
    assert unchanged_error == pytest.approx(0.01)
    assert not trial_settled(
        start, start, target, distance=0.01, elapsed=trial.duration + 0.1,
        duration=trial.duration, measured_speed=0.0,
        position_error=unchanged_error, yaw_error=0.0)

    # 8mm progress is several encoder/pose increments and within a 2.5mm
    # distance-scaled terminal tolerance for this 1cm commissioning move.
    measured = Pose2D(start.x + 0.008 * math.cos(start.yaw),
                      start.y + 0.008 * math.sin(start.yaw), start.yaw)
    error = math.hypot(target.x - measured.x, target.y - measured.y)
    assert trial_settled(
        start, measured, target, distance=0.01, elapsed=trial.duration + 0.1,
        duration=trial.duration, measured_speed=0.0,
        position_error=error, yaw_error=0.0)

    off_axis = Pose2D(start.x + 0.008 * math.cos(start.yaw + math.pi / 2),
                      start.y + 0.008 * math.sin(start.yaw + math.pi / 2),
                      start.yaw)
    assert not trial_settled(
        start, off_axis, target, distance=0.01, elapsed=trial.duration + 0.1,
        duration=trial.duration, measured_speed=0.0,
        position_error=math.hypot(target.x - off_axis.x,
                                  target.y - off_axis.y), yaw_error=0.0)
    assert not trial_settled(
        start, measured, target, distance=0.01, elapsed=trial.duration + 0.1,
        duration=trial.duration, measured_speed=0.03,
        position_error=error, yaw_error=0.0)


def test_control_deadline_caps_command_rate_and_skips_missed_slots():
    assert next_control_deadline(0.0, 0.001, 0.02) == pytest.approx(0.02)
    assert next_control_deadline(0.02, 0.025, 0.02) == pytest.approx(0.04)
    assert next_control_deadline(0.04, 0.1, 0.02) == pytest.approx(0.12)


def test_zero_window_retries_for_full_wall_clock_interval_without_context_gate():
    class FakeClock:
        now = 0.0

        def monotonic(self):
            return self.now

        def sleep(self, duration):
            self.now += duration

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    clock = FakeClock()
    publisher = Publisher()
    count = send_zero_window(
        publisher, lambda: (0.0, 0.0, 0.0), duration=0.1, period=0.02,
        clock=clock.monotonic, sleep=clock.sleep)
    assert count == 5
    assert len(publisher.messages) == 5
    assert set(publisher.messages) == {(0.0, 0.0, 0.0)}
    assert clock.now == pytest.approx(0.1)


def test_zero_window_continues_attempts_when_ros_publisher_throws():
    class FakeClock:
        now = 0.0

        def monotonic(self):
            return self.now

        def sleep(self, duration):
            self.now += duration

    class BrokenPublisher:
        def __init__(self):
            self.attempts = 0

        def publish(self, _message):
            self.attempts += 1
            raise RuntimeError('context already invalid')

    clock = FakeClock()
    publisher = BrokenPublisher()
    with pytest.raises(RuntimeError, match='all emergency zero-command attempts failed'):
        send_zero_window(publisher, lambda: object(), duration=0.1, period=0.02,
                         clock=clock.monotonic, sleep=clock.sleep)
    assert publisher.attempts == 5
    assert clock.now == pytest.approx(0.1)


def test_stop_signal_handlers_flag_exit_without_shutting_down_ros_context():
    class FakeSignals:
        SIGINT = 2
        SIGTERM = 15

        def __init__(self):
            self.handlers = {self.SIGINT: 'old-int', self.SIGTERM: 'old-term'}

        def signal(self, signum, handler):
            previous = self.handlers[signum]
            self.handlers[signum] = handler
            return previous

    signals = FakeSignals()
    event = threading.Event()
    previous, received = install_stop_signal_handlers(event, signal_module=signals)
    signals.handlers[signals.SIGTERM](signals.SIGTERM, None)
    assert event.is_set()
    assert received['signal'] == signals.SIGTERM
    restore_signal_handlers(previous, signal_module=signals)
    assert signals.handlers == {signals.SIGINT: 'old-int',
                                signals.SIGTERM: 'old-term'}


def test_ros_context_uses_idempotent_shutdown_once_after_successful_init():
    class FakeRclpy:
        def __init__(self):
            self.calls = 0

        def try_shutdown(self):
            self.calls += 1

    ros = FakeRclpy()
    shutdown_ros_context(ros, initialized=False)
    assert ros.calls == 0
    shutdown_ros_context(ros, initialized=True)
    assert ros.calls == 1


def test_integrated_command_distance_stays_within_three_centimeters_at_20ms_ticks():
    budget = CommandDistanceBudget()
    sent = []
    for _ in range(100):
        vx, vy = budget.limit_next(0.05, 0.0)
        sent.append(math.hypot(vx, vy) * 0.02)
        budget.account(vx, vy, 0.02)
        if budget.used >= MAX_COMMAND_DISTANCE_M - 1e-12:
            break
    assert sum(sent) == pytest.approx(MAX_COMMAND_DISTANCE_M)
    assert budget.used <= MAX_COMMAND_DISTANCE_M + 1e-12
    with pytest.raises(RuntimeError, match='exhausted'):
        budget.limit_next(0.05, 0.0)


def test_command_distance_overrun_faults_instead_of_issuing_more_motion():
    budget = CommandDistanceBudget()
    with pytest.raises(RuntimeError, match='exceeded'):
        budget.account(0.05, 0.0, 0.7)


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
