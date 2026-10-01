"""Hardware-facing control entry contracts without requiring ROS or a robot."""

import math
import threading

import pytest

from m3pro_nav.control_probe import (
    MAX_COMMAND_DISTANCE_M, MAX_DISTANCE_M, CommandDistanceBudget,
    GROUND_ODOM_MAX_DISTANCE_M, GROUND_ODOM_MAX_REFERENCE_SPEED_MPS,
    GROUND_ODOM_MAX_COMMAND_SPEED_MPS,
    ProbeOptions, StraightTrial, WheelSpinMonitor, WHEELS_UP_TARGET_COMMAND_M,
    WHEELS_UP_MAX_COMMAND_PATH_M, WHEELS_UP_MAX_SPEED_MPS, WHEELS_UP_MAX_WALL_S,
    WHEELS_UP_ODOM_TARGET_M, WHEELS_UP_ODOM_MAX_COMMAND_M,
    WHEELS_UP_ODOM_MAX_WALL_S, WHEELS_UP_ODOM_ACCEL_MPS2,
    WHEELS_UP_ODOM_DEFAULT_REFERENCE_SPEED_MPS,
    command_gate, install_stop_signal_handlers,
    ground_trial_loop_exit,
    limit_command, next_control_deadline, pose_envelope_error,
    restore_signal_handlers,
    send_zero_window, shutdown_ros_context, source_stamp_gate, validate_options,
    ground_odom_gate, source_stamp_age,
    trial_settled, update_settle_dwell, integrate_command_distance, _parser,
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


def test_wheels_up_mode_is_explicit_and_excludes_ground_trial_options():
    parsed = _parser().parse_args(['--wheels-up', '--expected-odom-frame', 'odom',
                                   '--expected-base-frame', 'base_link'])
    assert parsed.wheels_up
    assert validate_options(ProbeOptions(expected_odom_frame=parsed.expected_odom_frame,
        expected_base_frame=parsed.expected_base_frame,
        wheels_up=parsed.wheels_up)).wheels_up
    opts = ProbeOptions(expected_odom_frame='odom', expected_base_frame='base_link',
                        wheels_up=True)
    assert validate_options(opts).wheels_up
    with pytest.raises(ValueError, match='requires explicit'):
        validate_options(ProbeOptions(wheels_up=True))
    with pytest.raises(ValueError, match='excludes'):
        validate_options(ProbeOptions(wheels_up=True, execute=True,
            expected_odom_frame='odom', expected_base_frame='base_link',
            axis='x', distance=0.01))
    with pytest.raises(ValueError, match='excludes'):
        validate_options(ProbeOptions(wheels_up=True,
            expected_odom_frame='odom', expected_base_frame='base_link',
            distance=0.01))


def test_wheels_up_odom_mode_is_separate_explicit_and_bounded():
    parsed = _parser().parse_args(['--wheels-up-odom',
        '--expected-odom-frame', 'odom', '--expected-base-frame', 'base_link'])
    assert parsed.wheels_up_odom
    assert validate_options(ProbeOptions(expected_odom_frame='odom',
        expected_base_frame='base_link', wheels_up_odom=True)).wheels_up_odom
    assert WHEELS_UP_ODOM_TARGET_M == 2.0
    assert WHEELS_UP_ODOM_MAX_COMMAND_M == 3.0
    assert WHEELS_UP_ODOM_MAX_WALL_S == 12.0
    with pytest.raises(ValueError, match='requires explicit'):
        validate_options(ProbeOptions(wheels_up_odom=True))
    with pytest.raises(ValueError, match='excludes'):
        validate_options(ProbeOptions(wheels_up_odom=True, wheels_up=True,
            expected_odom_frame='odom', expected_base_frame='base_link'))
    with pytest.raises(ValueError, match='excludes'):
        validate_options(ProbeOptions(wheels_up_odom=True, execute=True,
            expected_odom_frame='odom', expected_base_frame='base_link'))
    parsed_speed = _parser().parse_args(['--wheels-up-odom',
        '--wheels-up-odom-reference-speed', '0.5',
        '--expected-odom-frame', 'odom', '--expected-base-frame', 'base_link'])
    assert parsed_speed.wheels_up_odom_reference_speed == pytest.approx(0.5)
    assert validate_options(ProbeOptions(expected_odom_frame='odom',
        expected_base_frame='base_link', wheels_up_odom=True,
        wheels_up_odom_reference_speed=0.5)).wheels_up_odom_reference_speed == pytest.approx(0.5)
    assert validate_options(ProbeOptions(expected_odom_frame='odom',
        expected_base_frame='base_link', wheels_up_odom=True,
        wheels_up_odom_kd_vel=0.5)).wheels_up_odom_kd_vel == pytest.approx(0.5)
    assert WHEELS_UP_ODOM_DEFAULT_REFERENCE_SPEED_MPS == pytest.approx(0.7)
    with pytest.raises(ValueError, match='at most'):
        validate_options(ProbeOptions(expected_odom_frame='odom',
            expected_base_frame='base_link', wheels_up_odom=True,
            wheels_up_odom_reference_speed=0.71))
    with pytest.raises(ValueError, match='requires --wheels-up-odom'):
        validate_options(ProbeOptions(wheels_up_odom_reference_speed=0.5))
    with pytest.raises(ValueError, match='requires --wheels-up-odom'):
        validate_options(ProbeOptions(wheels_up_odom_kd_vel=0.5))
    with pytest.raises(ValueError, match=r'in \[0, 1\]'):
        validate_options(ProbeOptions(expected_odom_frame='odom',
            expected_base_frame='base_link', wheels_up_odom=True,
            wheels_up_odom_kd_vel=1.1))


def test_ground_odom_trial_requires_explicit_opt_in_and_stays_within_0p4m_caps():
    parsed = _parser().parse_args(['--ground-odom-straight', '--axis', 'x',
        '--distance', '0.4', '--expected-odom-frame', 'odom',
        '--expected-base-frame', 'base_link'])
    opts = ProbeOptions(expected_odom_frame=parsed.expected_odom_frame,
        expected_base_frame=parsed.expected_base_frame, axis=parsed.axis,
        distance=parsed.distance, ground_odom_straight=parsed.ground_odom_straight)
    assert validate_options(opts).ground_odom_straight
    assert GROUND_ODOM_MAX_DISTANCE_M == pytest.approx(0.4)
    assert GROUND_ODOM_MAX_REFERENCE_SPEED_MPS == pytest.approx(0.2)
    assert GROUND_ODOM_MAX_COMMAND_SPEED_MPS == pytest.approx(0.2)
    for bad in (
        ProbeOptions(expected_odom_frame='odom', expected_base_frame='base_link',
            axis='x', distance=0.4),
        ProbeOptions(expected_odom_frame='odom', expected_base_frame='base_link',
            axis='x', distance=0.401, ground_odom_straight=True),
        ProbeOptions(expected_odom_frame='odom', expected_base_frame='base_link',
            axis='x', distance=0.1, ground_odom_straight=True,
            execute=True),
        ProbeOptions(expected_odom_frame='odom', expected_base_frame='base_link',
            axis='x', distance=0.1, ground_odom_straight=True,
            ground_reference_speed=0.201),
        ProbeOptions(expected_odom_frame='odom', expected_base_frame='base_link',
            axis='x', distance=0.1, ground_odom_straight=True,
            ground_a_dec=0.01),
    ):
        with pytest.raises(ValueError):
            validate_options(bad)


def test_ground_command_integral_is_unbounded_telemetry_and_linear_command_stays_capped():
    # A 0.4m trial may need more than 0.45m of integrated commands to settle.
    total = integrate_command_distance(0.0, 0.2, 0.0, 2.5)
    total = integrate_command_distance(total, 0.1, 0.0, 1.0)
    assert total == pytest.approx(0.6)
    capped = limit_command(0.3, 0.0, 0.0,
                           max_linear=GROUND_ODOM_MAX_COMMAND_SPEED_MPS)
    assert math.hypot(capped[0], capped[1]) == pytest.approx(0.2)
    with pytest.raises(ValueError, match='finite and nonnegative'):
        integrate_command_distance(0.0, 0.1, 0.0, -0.1)


def test_ground_odom_straight_reuses_profile_and_controller_with_trial_overrides():
    start = Pose2D(1.0, -2.0, math.pi / 6)
    trial = StraightTrial(start, axis='x', distance=0.4,
        max_speed=0.15, max_distance=GROUND_ODOM_MAX_DISTANCE_M,
        a_acc=0.2, a_dec=0.25, kp_pos=1.2, kd_vel=0.3)
    assert trial.controller.kp_pos == pytest.approx(1.2)
    assert trial.controller.kd_vel == pytest.approx(0.3)
    assert trial.profile.sample(trial.duration / 2).speed == pytest.approx(0.15)
    faster_brake = StraightTrial(start, axis='x', distance=0.4,
        max_speed=0.15, max_distance=GROUND_ODOM_MAX_DISTANCE_M,
        a_acc=0.2, a_dec=0.5)
    assert trial.duration > faster_brake.duration
    ref = trial.reference_at(trial.duration / 2)
    command = trial.sample(trial.duration / 2,
        Pose2D(ref.x, ref.y, ref.yaw_ref), (ref.vx_world, ref.vy_world), 0.0)
    assert math.isfinite(command.vx) and math.isfinite(command.vy)


def test_ground_trial_nonsettled_shutdown_exit_is_incomplete():
    assert ground_trial_loop_exit(True, True) == 'operator_interrupt'
    assert ground_trial_loop_exit(False, False) == 'ros_shutdown'
    assert ground_trial_loop_exit(True, False) == 'unexpected_loop_exit'


def test_wheels_up_odom_uses_existing_profile_reference_and_feedback_controller():
    start = Pose2D(2.0, -1.0, math.pi / 3)
    trial = StraightTrial(start, axis='x', distance=WHEELS_UP_ODOM_TARGET_M,
        max_speed=WHEELS_UP_MAX_SPEED_MPS,
        max_distance=WHEELS_UP_ODOM_TARGET_M,
        a_acc=WHEELS_UP_ODOM_ACCEL_MPS2, a_dec=WHEELS_UP_ODOM_ACCEL_MPS2)
    assert trial.profile.sample(trial.duration / 2).speed == pytest.approx(0.7)
    assert trial.duration < WHEELS_UP_ODOM_MAX_WALL_S
    target = trial.target_pose
    halfway = trial.profile.sample(trial.duration / 2)
    ref = trial.frame_transform.transform_reference(trial.reference.sample(
        halfway.progress_s, halfway.speed, halfway.acceleration))
    measured_at_reference = Pose2D(ref.x, ref.y, ref.yaw_ref)
    command = trial.sample(trial.duration / 2, measured_at_reference,
                           (ref.vx_world, ref.vy_world), 0.0)
    # Position correction can push raw controller output above profile speed;
    # the probe's existing publication limiter enforces the explicit cap.
    capped = limit_command(command.vx, command.vy, command.wz,
                           max_linear=WHEELS_UP_MAX_SPEED_MPS)
    assert math.hypot(*capped[:2]) <= WHEELS_UP_MAX_SPEED_MPS + 1e-12
    assert abs(command.wz) <= 0.15

    assert not trial_settled(start, start, target,
        distance=WHEELS_UP_ODOM_TARGET_M, elapsed=trial.duration + 0.1,
        duration=trial.duration, measured_speed=0.0,
        position_error=math.hypot(target.x-start.x, target.y-start.y),
        yaw_error=0.0, tolerance_override=0.02,
        minimum_progress_fraction=0.90,
        max_speed=0.02, max_yaw_error=0.15)
    assert trial_settled(start, target, target,
        distance=WHEELS_UP_ODOM_TARGET_M, elapsed=trial.duration + 0.1,
        duration=trial.duration, measured_speed=0.0,
        position_error=0.0, yaw_error=0.0, tolerance_override=0.02,
        minimum_progress_fraction=0.90,
        max_speed=0.02, max_yaw_error=0.15)
    assert not trial_settled(start, target, target,
        distance=WHEELS_UP_ODOM_TARGET_M, elapsed=trial.duration + 0.1,
        duration=trial.duration, measured_speed=0.03,
        position_error=0.0, yaw_error=0.0, tolerance_override=0.02,
        minimum_progress_fraction=0.90,
        max_speed=0.02, max_yaw_error=0.15)


def test_lower_reference_cruise_creates_position_correction_headroom():
    start = Pose2D(0.0, 0.0, 0.0)
    def raw_command(cruise):
        trial = StraightTrial(start, axis='x', distance=2.0, max_speed=cruise,
            max_distance=2.0, a_acc=1.0, a_dec=1.0)
        sample = trial.profile.sample(trial.duration / 2)
        ref = trial.frame_transform.transform_reference(trial.reference.sample(
            sample.progress_s, sample.speed, sample.acceleration))
        measured = Pose2D(ref.x - 0.19, ref.y, ref.yaw_ref)
        cmd = trial.sample(trial.duration / 2, measured, (0.5, 0.0), 0.0)
        return math.hypot(cmd.vx, cmd.vy)

    # With kp=1 and kd=0, .5 reference + .19m error requests .69m/s;
    # .7 reference + the same lag requests .89m/s and saturates at the cap.
    assert raw_command(0.5) == pytest.approx(0.69)
    assert raw_command(0.7) == pytest.approx(0.89)
    assert limit_command(raw_command(0.5), 0.0, 0.0,
                         max_linear=WHEELS_UP_MAX_SPEED_MPS)[0] == pytest.approx(0.69)
    assert limit_command(raw_command(0.7), 0.0, 0.0,
                         max_linear=WHEELS_UP_MAX_SPEED_MPS)[0] == pytest.approx(0.7)


def test_wheels_up_kd_vel_is_per_trial_and_changes_velocity_feedback_term():
    start = Pose2D(0.0, 0.0, 0.0)
    trials = [StraightTrial(start, axis='x', distance=2.0, max_speed=0.5,
        max_distance=2.0, a_acc=1.0, a_dec=1.0, kd_vel=gain)
        for gain in (0.0, 0.5)]
    t = trials[0].duration / 2
    sample = trials[0].profile.sample(t)
    reference = trials[0].frame_transform.transform_reference(
        trials[0].reference.sample(sample.progress_s, sample.speed,
                                   sample.acceleration))
    measured = Pose2D(reference.x, reference.y, reference.yaw_ref)
    commands = [trial.sample(t, measured, (0.36, 0.0), 0.0)
                for trial in trials]
    assert trials[0].controller.kd_vel == 0.0
    assert trials[1].controller.kd_vel == 0.5
    assert commands[0].vx == pytest.approx(0.5)
    assert commands[1].vx == pytest.approx(0.57)




def test_wheels_up_odom_settle_dwell_requires_distinct_advancing_samples():
    state = (None, None, 0)
    state, complete = update_settle_dwell(True, 10.0, 1.0, state)
    assert not complete
    # A frozen pose/stamp cannot satisfy dwell even after the interval elapses.
    state, complete = update_settle_dwell(True, 10.0, 1.1, state)
    assert not complete
    assert state[2] == 1
    state, complete = update_settle_dwell(True, 10.1, 1.15, state)
    assert not complete
    state, complete = update_settle_dwell(True, 10.2, 1.3, state)
    assert complete
    reset, complete = update_settle_dwell(False, 10.2, 1.4, state)
    assert not complete
    assert reset == (None, None, 0)


def test_wheels_up_odometry_anomalies_are_warnings_not_command_stops():
    monitor = WheelSpinMonitor(Pose2D(0.0, 0.0, 0.0), 1.0)
    assert monitor.update(Pose2D(0.1, 0.0, 0.0), 3.0, 0.1) == pytest.approx(0.1)
    assert monitor.update(Pose2D(0.0, 0.0, 0.0), 5.0, 0.1) == pytest.approx(0.1)
    assert 'reverse_progress_observed' in monitor.warnings
    assert WHEELS_UP_TARGET_COMMAND_M == 10.0
    assert WHEELS_UP_MAX_SPEED_MPS == pytest.approx(0.70)
    assert WHEELS_UP_MAX_COMMAND_PATH_M >= WHEELS_UP_TARGET_COMMAND_M
    assert WHEELS_UP_MAX_COMMAND_PATH_M == 10.05
    assert WHEELS_UP_MAX_WALL_S == 20.0
    assert monitor.check_progress_timeout(9.0)


def test_wheels_up_lateral_and_implausible_odometry_are_warning_only():
    monitor = WheelSpinMonitor(Pose2D(0.0, 0.0, math.pi / 4), 1.0)
    c = math.cos(math.pi / 4)
    s = math.sin(math.pi / 4)
    lateral_pose = Pose2D(-0.06 * s, 0.06 * c, math.pi / 4)
    monitor.update(lateral_pose, 2.0, 0.06)
    assert 'lateral_drift' in monitor.warnings

    monitor = WheelSpinMonitor(Pose2D(0.0, 0.0, 0.0), 1.0)
    # Repeated small forward jumps cannot accumulate beyond command integral
    # plus the one fixed 20mm timing/quantization allowance.
    for index in range(1, 21):
        assert monitor.update(Pose2D(index * 0.001, 0.0, 0.0),
                              1.0 + index * 0.1, 0.0) == pytest.approx(index * 0.001)
    monitor.update(Pose2D(0.021, 0.0, 0.0), 3.1, 0.0)
    assert 'cumulative_odom_exceeds_command_integral' in monitor.warnings


def test_wheels_up_accepts_observed_receipt_jitter_step_and_yaw_drift_ratio():
    monitor = WheelSpinMonitor(Pose2D(0.0, 0.0, 0.0), 1.0)
    assert monitor.update(Pose2D(0.057554, 0.0, 0.036), 1.09, 0.056) == pytest.approx(0.057554)
    assert monitor.update(Pose2D(0.26, 0.006, 0.036), 1.35, 0.20) == pytest.approx(0.26)
    for index in range(1, 10):
        x = 0.26 + 0.1 * index
        y = 0.006 + 0.0036 * index
        monitor.update(Pose2D(x, y, 0.036), 1.35 + 0.1 * index, 0.1)
    assert monitor.path == pytest.approx(1.16)


def test_wheels_up_source_stamp_gate_rejects_stale_or_future_queued_samples():
    assert source_stamp_gate(100.0, 99.8, 99.9)[0]
    assert not source_stamp_gate(100.0, 99.0, 99.9)[0]
    assert not source_stamp_gate(100.0, 100.2, 99.9)[0]


def test_ground_odom_gate_uses_only_exclusive_fresh_odometry():
    # Source-stamp monotonicity is enforced by OdometryMonitor; this gate
    # uses monotonic callback receipt age, not an assumed ROS clock alignment.
    assert ground_odom_gate(10.1, 10.0, 0) == (True, 'ready')
    assert ground_odom_gate(10.1, None, 0)[0] is False
    assert ground_odom_gate(10.1, 10.0, 1)[0] is False
    assert ground_odom_gate(10.7, 10.0, 0)[0] is False


def test_ground_odom_gate_has_no_imu_dependency_and_stamp_age_is_diagnostic():
    # Ground-trial gate has no IMU parameter by design; IMU age stays telemetry.
    assert source_stamp_age(100.2, 100.0) == pytest.approx(0.2)
    assert source_stamp_age(100.2, None) is None


def test_wheels_up_command_integral_cap_and_emergency_zero_cleanup():
    budget = CommandDistanceBudget(WHEELS_UP_MAX_COMMAND_PATH_M)
    elapsed = 0.0
    for _ in range(1000):
        vx, vy = budget.limit_next(WHEELS_UP_MAX_SPEED_MPS, 0.0)
        vx = min(vx, (WHEELS_UP_TARGET_COMMAND_M - budget.used) / 0.02)
        assert math.hypot(vx, vy) <= WHEELS_UP_MAX_SPEED_MPS + 1e-12
        budget.account(vx, vy, 0.02)
        elapsed += 0.02
        if budget.used >= WHEELS_UP_TARGET_COMMAND_M - 1e-12:
            break
    assert budget.used == pytest.approx(WHEELS_UP_TARGET_COMMAND_M)
    assert elapsed == pytest.approx(WHEELS_UP_TARGET_COMMAND_M / WHEELS_UP_MAX_SPEED_MPS,
                                    abs=0.02)
    assert elapsed < WHEELS_UP_MAX_WALL_S
    assert WHEELS_UP_MAX_COMMAND_PATH_M >= budget.used

    class FakeClock:
        now = 0.0
        def monotonic(self): return self.now
        def sleep(self, dt): self.now += dt
    class Publisher:
        def __init__(self): self.messages = []
        def publish(self, message): self.messages.append(message)
    clock, publisher = FakeClock(), Publisher()
    count = send_zero_window(publisher, lambda: (0.0, 0.0, 0.0),
        duration=0.1, period=0.02, clock=clock.monotonic, sleep=clock.sleep)
    assert count == 5
    assert publisher.messages == [(0.0, 0.0, 0.0)] * 5


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
    assert limit_command(0.70, 0.0, 0.0,
                         max_linear=WHEELS_UP_MAX_SPEED_MPS)[0] == pytest.approx(0.70)


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
