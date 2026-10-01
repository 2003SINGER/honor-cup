"""Read-only ROS sensor recorder and explicitly gated short position trial.

The default invocation subscribes only. During execution Odometry supplies the
validated child-frame yaw rate; raw IMU axes are logged and used only as a
freshness gate because their frame/axis mapping is unverified. Hardware gains,
frames, command ownership, and plant response remain uncalibrated/unverified.
"""

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import threading
import time

from .motion_primitive import MotionPrimitive
from .pose import Pose2D, norm_angle
from .frame_transform import RigidFrameTransform
from .position_controller import PositionController
from .speed_profile import SpeedProfile
from .trajectory_reference import TrajectoryReference
from .odometry_adapter import OdometryMonitor, odometry_from_msg
from .motion_planner import V_CRUISE

MAX_DISTANCE_M = 0.03
MIN_DISTANCE_M = 0.005
# Local firmware notes list 2464 pulses/rev and 0.2513m wheel circumference
# (~0.1mm/pulse). This 2mm floor asks for multiple encoder increments; it is
# not a claim about calibrated odometry accuracy.
MIN_OBSERVED_PROGRESS_M = 0.002
MIN_OBSERVED_PROGRESS_FRACTION = 0.75
MAX_LINEAR_MPS = 0.05
MAX_ANGULAR_RADPS = 0.15
MAX_COMMAND_DISTANCE_M = 0.03
MAX_SENSOR_AGE_S = 0.5
CONTROL_PERIOD_S = 0.02
POST_STOP_S = 0.5
DISCOVERY_S = 1.0
WAIT_SENSORS_S = 3.0
WHEELS_UP_TARGET_COMMAND_M = 10.0
WHEELS_UP_MAX_WALL_S = 20.0
WHEELS_UP_MAX_COMMAND_PATH_M = WHEELS_UP_TARGET_COMMAND_M + 0.05
# Match the canonical fixed-template straight cruise speed; normal probe cap is unchanged.
WHEELS_UP_MAX_SPEED_MPS = V_CRUISE
WHEELS_UP_PROGRESS_TIMEOUT_S = 3.0
WHEELS_UP_MIN_PROGRESS_M = 0.002
WHEELS_UP_MAX_ODOM_SLACK_M = 0.02
WHEELS_UP_MAX_LATERAL_DRIFT_M = 0.05
WHEELS_UP_MAX_LATERAL_FRACTION = 0.10
WHEELS_UP_MAX_REVERSE_STEP_M = 0.002
WHEELS_UP_MAX_CUMULATIVE_REVERSE_M = 0.05
WHEELS_UP_ODOM_TARGET_M = 2.0
WHEELS_UP_ODOM_MAX_COMMAND_M = 3.0
WHEELS_UP_ODOM_MAX_WALL_S = 12.0
WHEELS_UP_ODOM_SETTLE_TOLERANCE_M = 0.02
WHEELS_UP_ODOM_SETTLE_SPEED_MPS = 0.02
WHEELS_UP_ODOM_MAX_CROSS_TRACK_M = 0.10
WHEELS_UP_ODOM_MAX_YAW_ERROR_RAD = 0.15
WHEELS_UP_ODOM_ACCEL_MPS2 = 1.0  # nav_runtime.yaml provisional profile value
WHEELS_UP_ODOM_SETTLE_TIMEOUT_S = 5.0
WHEELS_UP_ODOM_SETTLE_DWELL_S = 0.25
WHEELS_UP_ODOM_SETTLE_MIN_SAMPLES = 3
WHEELS_UP_ODOM_DEFAULT_REFERENCE_SPEED_MPS = WHEELS_UP_MAX_SPEED_MPS
GROUND_ODOM_MAX_DISTANCE_M = 0.4
GROUND_ODOM_MAX_REFERENCE_SPEED_MPS = 0.20
GROUND_ODOM_MAX_COMMAND_SPEED_MPS = 0.20
GROUND_ODOM_DEFAULT_SPEED_MPS = 0.15
GROUND_ODOM_DEFAULT_ACCEL_MPS2 = 0.20
GROUND_ODOM_DEFAULT_DECEL_MPS2 = 0.20
GROUND_ODOM_DEFAULT_KD_VEL = 0.20
GROUND_ODOM_MAX_WALL_S = 12.0
GROUND_ODOM_MAX_OVERSHOOT_M = 0.08
GROUND_ODOM_MAX_CROSS_TRACK_M = 0.05
GROUND_ODOM_MAX_YAW_ERROR_RAD = 0.15


def resolve_ground_kd_vel(kd_vel: float | None) -> float:
    """Use the provisional ground-trial damping default unless overridden."""
    return GROUND_ODOM_DEFAULT_KD_VEL if kd_vel is None else kd_vel


def install_stop_signal_handlers(stop_requested, *, signal_module=signal):
    """Replace rclpy's shutdown signals with a flag checked by the run loop."""
    received = {'signal': None}

    def request_stop(signum, _frame):
        received['signal'] = signum
        stop_requested.set()

    previous = {
        signum: signal_module.signal(signum, request_stop)
        for signum in (signal_module.SIGINT, signal_module.SIGTERM)
    }
    return previous, received


def restore_signal_handlers(previous, *, signal_module=signal):
    for signum, handler in previous.items():
        signal_module.signal(signum, handler)


def shutdown_ros_context(rclpy, initialized):
    """Use rclpy's idempotent shutdown helper exactly once after init."""
    if initialized:
        rclpy.try_shutdown()


def send_zero_window(publisher, make_zero, *, duration=POST_STOP_S,
                     period=CONTROL_PERIOD_S, clock=time.monotonic,
                     sleep=time.sleep, on_publish=None, on_error=None):
    """Retry zero publishes for a fixed wall-clock window, regardless of rclpy.ok()."""
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('zero window duration must be positive and finite')
    if not math.isfinite(period) or period <= 0:
        raise ValueError('zero publish period must be positive and finite')
    end = clock() + duration
    successful = 0
    last_error = None
    while clock() < end:
        try:
            publisher.publish(make_zero())
            successful += 1
            if on_publish is not None:
                try:
                    on_publish()
                except Exception as exc:
                    last_error = exc
                    if on_error is not None:
                        try:
                            on_error(exc)
                        except Exception:
                            pass
        except Exception as exc:
            last_error = exc
            if on_error is not None:
                try:
                    on_error(exc)
                except Exception:
                    pass
        remaining = end - clock()
        if remaining > 0:
            sleep(min(period, remaining))
    if successful == 0:
        raise RuntimeError('all emergency zero-command attempts failed') from last_error
    return successful


class CommandDistanceBudget:
    """Bound integrated linear setpoints at the nominal control period."""
    def __init__(self, limit=MAX_COMMAND_DISTANCE_M):
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError('command distance limit must be positive and finite')
        self.limit = limit
        self.used = 0.0

    def account(self, vx, vy, elapsed):
        if not all(math.isfinite(value) for value in (vx, vy, elapsed)) or elapsed < 0:
            raise ValueError('command and elapsed time must be finite and valid')
        self.used += math.hypot(vx, vy) * elapsed
        if self.used > self.limit + 1e-12:
            raise RuntimeError('integrated command-distance budget exceeded')

    def limit_next(self, vx, vy, period=CONTROL_PERIOD_S):
        if not all(math.isfinite(value) for value in (vx, vy, period)) or period <= 0:
            raise ValueError('command and period must be finite and valid')
        speed = math.hypot(vx, vy)
        remaining = max(0.0, self.limit - self.used)
        if remaining <= 1e-12:
            raise RuntimeError('integrated command-distance budget exhausted')
        allowed = remaining / period
        if speed > allowed:
            if allowed <= 0.0:
                raise RuntimeError('integrated command-distance budget exhausted')
            scale = allowed / speed
            vx, vy = vx * scale, vy * scale
        return vx, vy


def integrate_command_distance(total, vx, vy, elapsed):
    """Accumulate commanded linear distance for telemetry without limiting it."""
    if not all(math.isfinite(value) for value in (total, vx, vy, elapsed)) \
            or total < 0 or elapsed < 0:
        raise ValueError('command integral inputs must be finite and nonnegative')
    return total + math.hypot(vx, vy) * elapsed


class WheelSpinMonitor:
    """Collect odometry telemetry and warning codes; never gates wheel commands."""
    def __init__(self, start_pose: Pose2D, start_time: float):
        if not all(math.isfinite(v) for v in
                   (start_pose.x, start_pose.y, start_time)):
            raise ValueError('invalid wheels-up odometry baseline')
        self.last_pose = start_pose.copy()
        self.initial_pose = start_pose.copy()
        self.start_yaw = start_pose.yaw
        self.last_time = start_time
        self.path = 0.0
        self.command_integral = 0.0
        self.reverse_drift = 0.0
        self.lateral_path = 0.0
        self.last_progress_time = start_time
        self.warnings = set()

    def update(self, pose: Pose2D, now: float, command_integral: float) -> float:
        if (not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw,
                                                now, command_integral))
                or now <= self.last_time or command_integral < 0):
            raise ValueError('invalid or non-monotonic wheels-up odometry')
        dx, dy = pose.x - self.last_pose.x, pose.y - self.last_pose.y
        step = math.hypot(dx, dy)
        if step > command_integral + WHEELS_UP_MAX_ODOM_SLACK_M + 1e-9:
            self.warnings.add('odom_step_exceeds_command_integral')
        # Count only positive travel along the initial body-x axis. Reverse or
        # lateral movement cannot add toward the requested wheel-spin distance.
        c, s = math.cos(self.start_yaw), math.sin(self.start_yaw)
        forward_signed = dx * c + dy * s
        if forward_signed < -WHEELS_UP_MAX_REVERSE_STEP_M:
            self.warnings.add('reverse_progress_observed')
        self.reverse_drift += max(0.0, -forward_signed)
        if self.reverse_drift > WHEELS_UP_MAX_CUMULATIVE_REVERSE_M:
            self.warnings.add('cumulative_reverse_drift')
        self.lateral_path += abs(-dx * s + dy * c)
        lateral_limit = (WHEELS_UP_MAX_LATERAL_DRIFT_M
                         + WHEELS_UP_MAX_LATERAL_FRACTION * self.path)
        if self.lateral_path > lateral_limit:
            self.warnings.add('lateral_drift')
        self.command_integral += command_integral
        forward_step = max(0.0, forward_signed)
        self.path += forward_step
        if self.path > self.command_integral + WHEELS_UP_MAX_ODOM_SLACK_M + 1e-9:
            self.warnings.add('cumulative_odom_exceeds_command_integral')
        if forward_step >= WHEELS_UP_MIN_PROGRESS_M:
            self.last_progress_time = now
        self.last_pose = pose.copy()
        self.last_time = now
        return self.path

    def check_progress_timeout(self, now: float):
        if now - self.last_progress_time > WHEELS_UP_PROGRESS_TIMEOUT_S:
            self.warnings.add('no_odom_progress')
        return 'no_odom_progress' in self.warnings


@dataclass(frozen=True)
class ProbeOptions:
    execute: bool = False
    expected_odom_frame: str | None = None
    expected_base_frame: str | None = None
    axis: str | None = None
    distance: float | None = None
    csv_path: str | None = None
    wheels_up: bool = False
    wheels_up_odom: bool = False
    wheels_up_odom_reference_speed: float | None = None
    wheels_up_odom_kd_vel: float | None = None
    ground_odom_straight: bool = False
    ground_reference_speed: float | None = None
    ground_a_dec: float | None = None
    ground_kp_pos: float | None = None
    ground_kd_vel: float | None = None


def validate_options(options: ProbeOptions) -> ProbeOptions:
    """Validate CLI constraints without importing ROS."""
    if options.ground_odom_straight:
        if options.execute or options.wheels_up or options.wheels_up_odom:
            raise ValueError('--ground-odom-straight excludes other motion modes')
        if (options.wheels_up_odom_reference_speed is not None
                or options.wheels_up_odom_kd_vel is not None):
            raise ValueError('--ground-odom-straight excludes wheels-up tuning parameters')
        if not options.expected_odom_frame or not options.expected_base_frame:
            raise ValueError('--ground-odom-straight requires explicit --expected-odom-frame and --expected-base-frame')
        if options.axis not in ('x', 'y'):
            raise ValueError('--ground-odom-straight requires --axis x|y')
        if (options.distance is None or not math.isfinite(options.distance)
                or options.distance < MIN_DISTANCE_M
                or options.distance > GROUND_ODOM_MAX_DISTANCE_M):
            raise ValueError('--ground-odom-straight distance must be in [0.005, 0.4] m')
        if (options.ground_reference_speed is not None
                and (not math.isfinite(options.ground_reference_speed)
                     or options.ground_reference_speed <= 0
                     or options.ground_reference_speed > GROUND_ODOM_MAX_REFERENCE_SPEED_MPS)):
            raise ValueError('--ground-reference-speed must be in (0, 0.20] m/s')
        if (options.ground_a_dec is not None
                and (not math.isfinite(options.ground_a_dec)
                     or not 0.05 <= options.ground_a_dec <= 1.0)):
            raise ValueError('--ground-a-dec must be in [0.05, 1.0] m/s^2')
        if (options.ground_kp_pos is not None
                and (not math.isfinite(options.ground_kp_pos)
                     or not 0.0 <= options.ground_kp_pos <= 2.0)):
            raise ValueError('--ground-kp-pos must be in [0, 2]')
        if (options.ground_kd_vel is not None
                and (not math.isfinite(options.ground_kd_vel)
                     or not 0.0 <= options.ground_kd_vel <= 1.0)):
            raise ValueError('--ground-kd-vel must be in [0, 1]')
        return options
    if any(v is not None for v in (options.ground_reference_speed,
                                    options.ground_a_dec,
                                    options.ground_kp_pos,
                                    options.ground_kd_vel)):
        raise ValueError('ground tuning parameters require --ground-odom-straight')
    if options.wheels_up_odom:
        if options.execute or options.wheels_up or options.axis is not None or options.distance is not None:
            raise ValueError('--wheels-up-odom excludes --execute, --wheels-up, --axis, and --distance')
        if not options.expected_odom_frame or not options.expected_base_frame:
            raise ValueError('--wheels-up-odom requires explicit --expected-odom-frame and --expected-base-frame')
        if (options.wheels_up_odom_reference_speed is not None
                and (not math.isfinite(options.wheels_up_odom_reference_speed)
                     or options.wheels_up_odom_reference_speed <= 0
                     or options.wheels_up_odom_reference_speed > WHEELS_UP_MAX_SPEED_MPS)):
            raise ValueError('--wheels-up-odom-reference-speed must be positive and at most the 0.70m/s command cap')
        if (options.wheels_up_odom_kd_vel is not None
                and (not math.isfinite(options.wheels_up_odom_kd_vel)
                     or not 0.0 <= options.wheels_up_odom_kd_vel <= 1.0)):
            raise ValueError('--wheels-up-odom-kd-vel must be finite and in [0, 1]')
        return options
    if options.wheels_up_odom_reference_speed is not None:
        raise ValueError('--wheels-up-odom-reference-speed requires --wheels-up-odom')
    if options.wheels_up_odom_kd_vel is not None:
        raise ValueError('--wheels-up-odom-kd-vel requires --wheels-up-odom')
    if options.wheels_up:
        if options.execute or options.axis is not None or options.distance is not None:
            raise ValueError('--wheels-up excludes --execute, --axis, and --distance')
        if not options.expected_odom_frame or not options.expected_base_frame:
            raise ValueError('--wheels-up requires explicit --expected-odom-frame and --expected-base-frame')
        return options
    if not options.execute:
        if any(v is not None for v in (options.axis, options.distance,
                                       options.expected_odom_frame,
                                       options.expected_base_frame)):
            raise ValueError('motion parameters require --execute')
        return options
    if not options.expected_odom_frame or not options.expected_base_frame:
        raise ValueError('--execute requires explicit --expected-odom-frame and --expected-base-frame')
    if options.axis not in ('x', 'y'):
        raise ValueError('--execute requires --axis x|y')
    if options.distance is None or not math.isfinite(options.distance) or options.distance <= 0:
        raise ValueError('--execute requires a positive finite --distance')
    if options.distance < MIN_DISTANCE_M:
        raise ValueError(f'distance is below minimum observable trial distance {MIN_DISTANCE_M} m')
    if options.distance > MAX_DISTANCE_M:
        raise ValueError(f'distance exceeds safety cap {MAX_DISTANCE_M} m')
    return options


def command_gate(now: float, odom_received: float | None,
                 imu_received: float | None, foreign_publishers: int) -> tuple[bool, str]:
    """Pure pre-publication gate for receipt age and command ownership."""
    if not math.isfinite(now):
        return False, 'invalid monotonic time'
    for label, receipt in (('odometry', odom_received), ('IMU', imu_received)):
        if receipt is None or not math.isfinite(receipt) or now < receipt:
            return False, f'{label} missing or invalid receipt time'
        if now - receipt > MAX_SENSOR_AGE_S:
            return False, f'{label} receipt is stale'
    if foreign_publishers != 0:
        return False, f'/cmd_vel has {foreign_publishers} other publisher(s)'
    return True, 'ready'


def source_stamp_gate(ros_now: float, odom_stamp: float | None,
                      imu_stamp: float | None, max_age=MAX_SENSOR_AGE_S):
    """Reject old/future source stamps using the active ROS clock domain."""
    if not math.isfinite(ros_now) or not math.isfinite(max_age) or max_age <= 0:
        return False, 'invalid ROS clock or source-stamp age limit'
    for label, stamp in (('odometry', odom_stamp), ('IMU', imu_stamp)):
        if stamp is None or not math.isfinite(stamp):
            return False, f'{label} source stamp missing or invalid'
        age = ros_now - stamp
        if age < -0.1:
            return False, f'{label} source stamp is in the future'
        if age > max_age:
            return False, f'{label} source stamp is stale'
    return True, 'source stamps fresh'


def ground_odom_gate(monotonic_now: float, odom_received: float | None,
                     foreign_publishers: int, max_age=MAX_SENSOR_AGE_S):
    """Gate on exclusive commands and recent odom callback receipt.

    OdometryMonitor validates each sample's frame and strictly advancing
    source stamp before it can become the node's current odometry. Comparing
    that source clock against this node's ROS clock is diagnostic only: those
    clocks can have a stable offset even while samples arrive in order.
    """
    if not math.isfinite(monotonic_now):
        return False, 'invalid monotonic time'
    if foreign_publishers != 0:
        return False, f'/cmd_vel has {foreign_publishers} other publisher(s)'
    if odom_received is None or not math.isfinite(odom_received) \
            or monotonic_now < odom_received:
        return False, 'odometry missing or invalid receipt time'
    if monotonic_now - odom_received > max_age:
        return False, 'odometry receipt is stale'
    if not math.isfinite(max_age) or max_age <= 0:
        return False, 'invalid odometry receipt age limit'
    return True, 'ready'


def source_stamp_age(ros_now: float, stamp: float | None):
    """Return source-stamp age for diagnostics, or None when unavailable."""
    if stamp is None or not math.isfinite(stamp) or not math.isfinite(ros_now):
        return None
    return ros_now - stamp


def limit_command(vx: float, vy: float, wz: float,
                  max_linear=MAX_LINEAR_MPS):
    """Reject invalid/extreme controller output and apply configured caps."""
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        raise ValueError('refusing nonfinite command')
    if abs(vx) > 1.0 or abs(vy) > 1.0 or abs(wz) > 1.0:
        raise ValueError('controller command is extreme')
    speed = math.hypot(vx, vy)
    if speed > max_linear:
        scale = max_linear / speed
        vx, vy = vx * scale, vy * scale
    wz = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, wz))
    return vx, vy, wz


def pose_envelope_error(start_pose: Pose2D, measured_pose: Pose2D):
    """Return displacement and wrapped heading error for trial stop gating."""
    displacement = math.hypot(measured_pose.x - start_pose.x,
                              measured_pose.y - start_pose.y)
    yaw_error = abs(norm_angle(measured_pose.yaw - start_pose.yaw))
    return displacement, yaw_error


def observed_trial_progress(start_pose: Pose2D, measured_pose: Pose2D,
                            target_pose: Pose2D):
    """Project measured displacement along/across the commanded trial axis."""
    tx, ty = target_pose.x - start_pose.x, target_pose.y - start_pose.y
    distance = math.hypot(tx, ty)
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError('trial target must be separated from its start')
    ux, uy = tx / distance, ty / distance
    dx, dy = measured_pose.x - start_pose.x, measured_pose.y - start_pose.y
    progress = dx * ux + dy * uy
    cross_track = abs(-dx * uy + dy * ux)
    return progress, cross_track


def trial_settled(start_pose: Pose2D, measured_pose: Pose2D,
                  target_pose: Pose2D, *, distance: float, elapsed: float,
                  duration: float, measured_speed: float,
                  position_error: float, yaw_error: float,
                  tolerance_override: float | None = None,
                  minimum_progress_fraction=MIN_OBSERVED_PROGRESS_FRACTION,
                  max_speed=0.02, max_yaw_error=0.05):
    """Require observable forward progress, target accuracy, and low speed."""
    values = (distance, elapsed, duration, measured_speed, position_error,
              yaw_error)
    if not all(math.isfinite(value) for value in values):
        return False
    tolerance = (tolerance_override if tolerance_override is not None else
                 min(0.01, max(0.001, distance * 0.25)))
    minimum_progress = max(MIN_OBSERVED_PROGRESS_M,
                           distance * minimum_progress_fraction)
    progress, cross_track = observed_trial_progress(
        start_pose, measured_pose, target_pose)
    return (elapsed >= duration and position_error <= tolerance
            and progress >= minimum_progress and cross_track <= tolerance
            and measured_speed <= max_speed and yaw_error <= max_yaw_error)


def update_settle_dwell(inside_tolerance: bool, source_stamp: float,
                        now: float, state: tuple, *, dwell_s=0.25,
                        min_samples=WHEELS_UP_ODOM_SETTLE_MIN_SAMPLES):
    """Require distinct advancing source stamps throughout measured settling."""
    if (not math.isfinite(source_stamp) or not math.isfinite(now)
            or not math.isfinite(dwell_s) or dwell_s < 0 or min_samples < 2):
        raise ValueError('invalid settling dwell input')
    since, last_stamp, count = state
    if not inside_tolerance:
        return (None, None, 0), False
    if since is None:
        return (now, source_stamp, 1), False
    if source_stamp > last_stamp:
        count += 1
        last_stamp = source_stamp
    next_state = (since, last_stamp, count)
    complete = now - since >= dwell_s and count >= min_samples
    return next_state, complete


def ground_trial_loop_exit(ros_ok: bool, stop_requested: bool) -> str:
    """Classify a non-settled exit; callers must treat all outcomes as incomplete."""
    if stop_requested:
        return 'operator_interrupt'
    if not ros_ok:
        return 'ros_shutdown'
    return 'unexpected_loop_exit'


def next_control_deadline(previous_deadline: float, now: float,
                          period: float = CONTROL_PERIOD_S):
    """Return the next paced tick, skipping missed slots instead of bursting."""
    if not all(math.isfinite(value) for value in
               (previous_deadline, now, period)) or period <= 0:
        raise ValueError('control deadline inputs must be finite and period positive')
    candidate = previous_deadline + period
    return candidate if candidate > now else now + period


class StraightTrial:
    """A short straight generated through the existing profile/reference/controller."""
    def __init__(self, start_pose: Pose2D, *, axis: str, distance: float,
                 max_speed=MAX_LINEAR_MPS, max_distance=MAX_DISTANCE_M,
                 a_acc=0.05, a_dec=0.05, kp_pos=1.0, kd_vel=0.0):
        if axis not in ('x', 'y') or not all(math.isfinite(v) for v in
                (start_pose.x, start_pose.y, start_pose.yaw, distance,
                 max_speed, max_distance, a_acc, a_dec, kp_pos)):
            raise ValueError('invalid straight-trial input')
        if (distance <= 0 or distance > max_distance or max_speed <= 0
                or max_speed > 1.0 or a_acc <= 0 or a_dec <= 0):
            raise ValueError('straight trial exceeds safety limits')
        if not math.isfinite(kd_vel) or kd_vel < 0 or kd_vel > 1.0:
            raise ValueError('straight trial kd_vel must be in [0, 1]')
        if kp_pos < 0 or kp_pos > 2.0:
            raise ValueError('straight trial kp_pos must be in [0, 2]')
        # Construct geometry in an axis-aligned local frame, then transform
        # references into odom. MotionPrimitive intentionally rejects diagonals.
        dx, dy = (distance, 0.0) if axis == 'x' else (0.0, distance)
        primitive = MotionPrimitive(kind='STRAIGHT', start_pose=Pose2D(0.0, 0.0, 0.0),
            p0=(0.0, 0.0), p1=(dx, dy), yaw0=0.0,
            length=distance, v_max=max_speed, v_end=0.0)
        # Conservative trial parameterization, still provisional pending calibration.
        self.profile = SpeedProfile((primitive,), start_speed=0.0,
                                    a_acc=a_acc, a_dec=a_dec)
        self.reference = TrajectoryReference((primitive,), yaw_ref=0.0)
        self.controller = PositionController(kp_pos=kp_pos, kd_vel=kd_vel)
        self.duration = self.profile.duration
        self.start_pose = start_pose.copy()
        self.frame_transform = RigidFrameTransform(
            Pose2D(0.0, 0.0, 0.0), self.start_pose)

    @property
    def target_pose(self):
        local = self.reference.sample(self.profile.length)
        return self.frame_transform.transform_pose(
            Pose2D(local.x, local.y, local.yaw_ref))

    def sample(self, elapsed: float, measured_pose: Pose2D,
               measured_velocity_world, yaw_rate: float):
        reference = self.reference_at(elapsed)
        return self.controller.update(reference, measured_pose,
            measured_velocity_world=measured_velocity_world, yaw_rate=yaw_rate)

    def reference_at(self, elapsed: float):
        speed = self.profile.sample(elapsed)
        return self.frame_transform.transform_reference(self.reference.sample(
            speed.progress_s, speed.speed, speed.acceleration))


CSV_FIELDS = ('received_monotonic_s', 'record_type', 'source_stamp_s', 'frame_id',
              'child_frame_id', 'reason', 'pose_x_m', 'pose_y_m', 'pose_yaw_rad',
              'world_vx_mps', 'world_vy_mps', 'odom_wz_radps', 'imu_frame_id',
              'imu_wz_radps', 'imu_ax_mps2', 'imu_ay_mps2', 'cmd_vx_mps',
              'cmd_vy_mps', 'cmd_wz_radps', 'command_integral_m',
              'odom_forward_path_m', 'odom_displacement_m', 'odom_lateral_path_m',
              'warning', 'ros_now_s', 'odom_stamp_age_s', 'imu_stamp_age_s',
              'ref_x_m', 'ref_y_m', 'ref_yaw_rad', 'pos_err_m',
              'yaw_err_rad', 'settled')


def _parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--wheels-up', action='store_true',
                   help='explicit 10m command-integral wheel-spin diagnostic; vehicle must be securely lifted')
    p.add_argument('--wheels-up-odom', action='store_true',
                   help='2m odometry-closed controller diagnostic; vehicle must be securely lifted')
    p.add_argument('--wheels-up-odom-reference-speed', type=float,
                   help='diagnostic reference cruise speed in m/s (default 0.70; command cap remains 0.70)')
    p.add_argument('--wheels-up-odom-kd-vel', type=float,
                   help='diagnostic-only velocity damping gain (default 0; allowed range 0..1)')
    p.add_argument('--ground-odom-straight', action='store_true',
                   help='explicit bounded ground straight trial; clear, supervised route required')
    p.add_argument('--ground-reference-speed', type=float,
                   help='ground reference speed in m/s (default 0.15; hard cap 0.20)')
    p.add_argument('--ground-a-dec', type=float,
                   help='ground reference deceleration in m/s^2 (default 0.20)')
    p.add_argument('--ground-kp-pos', type=float,
                   help='ground position gain (default 1.0; range 0..2)')
    p.add_argument('--ground-kd-vel', type=float,
                   help='ground velocity damping gain (default 0.2; range 0..1)')
    p.add_argument('--expected-odom-frame')
    p.add_argument('--expected-base-frame')
    p.add_argument('--axis', choices=('x', 'y'))
    p.add_argument('--distance', type=float)
    p.add_argument('--csv', dest='csv_path')
    return p


def main(args=None):
    parsed, ros_args = _parser().parse_known_args(args)
    options = validate_options(ProbeOptions(parsed.execute, parsed.expected_odom_frame,
        parsed.expected_base_frame, parsed.axis, parsed.distance, parsed.csv_path,
        parsed.wheels_up, parsed.wheels_up_odom,
        parsed.wheels_up_odom_reference_speed,
        parsed.wheels_up_odom_kd_vel, parsed.ground_odom_straight,
        parsed.ground_reference_speed, parsed.ground_a_dec,
        parsed.ground_kp_pos, parsed.ground_kd_vel))

    # Delayed ROS imports keep pure logic importable on macOS.
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class ControlProbeNode(Node):
        def __init__(self):
            super().__init__('m3pro_control_probe_' + str(int(time.time()*1000) % 1000000))
            csv_stamp = f'{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns() % 1000000000:09d}'
            if options.wheels_up or options.wheels_up_odom or options.ground_odom_straight:
                default_csv = ('/tmp/m3pro_ground_odom.csv'
                               if options.ground_odom_straight else
                               '/tmp/m3pro_wheels_up.csv')
                requested = Path(options.csv_path or default_csv)
                suffix = requested.suffix or '.csv'
                if options.ground_odom_straight:
                    self.csv_path = requested.with_name(
                        f'{requested.stem}_ground_odom_{csv_stamp}{suffix}')
                else:
                    mode = 'odom_closed' if options.wheels_up_odom else 'command'
                    self.csv_path = requested.with_name(
                        f'{requested.stem}_wheels_up_{mode}_{csv_stamp}{suffix}')
            else:
                self.csv_path = Path(options.csv_path or
                    f'/tmp/m3pro_control_probe_{csv_stamp}.csv')
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            self.file = self.csv_path.open('x' if (options.wheels_up or options.wheels_up_odom or options.ground_odom_straight) else 'w',
                                           newline='', encoding='utf-8')
            self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
            self.writer.writeheader()
            self.odom = self.imu = None
            self.odom_received = self.imu_received = None
            self.imu_stamp = None
            self.odom_monitor = OdometryMonitor(max_age=MAX_SENSOR_AGE_S)
            self.failure = None
            self._wheels_up_warnings = set()
            self.publisher = None
            self._cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
            self._odom_sub = self.create_subscription(Odometry, '/odom_raw', self.on_odom, 10)
            self._imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 10)

        def write(self, **values):
            row = {key: '' for key in CSV_FIELDS}
            row.update(received_monotonic_s=f'{time.monotonic():.6f}', **values)
            self.writer.writerow(row)
            self.file.flush()

        @staticmethod
        def stamp(msg):
            sec, nanosec = int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)
            if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
                raise ValueError('message stamp fields are out of range')
            return float(sec) + float(nanosec) / 1e9

        def on_cmd(self, msg):
            self.write(record_type='cmd_vel_observed', cmd_vx_mps=msg.linear.x,
                cmd_vy_mps=msg.linear.y, cmd_wz_radps=msg.angular.z)

        def on_odom(self, msg):
            try:
                value = odometry_from_msg(msg, expected_odom_frame=(options.expected_odom_frame or msg.header.frame_id),
                    expected_base_frame=(options.expected_base_frame or msg.child_frame_id))
                received = time.monotonic()
                self.odom_monitor.accept(value, now=received, received=received,
                    expected_odom_frame=(options.expected_odom_frame or msg.header.frame_id),
                    expected_base_frame=(options.expected_base_frame or msg.child_frame_id))
                self.odom, self.odom_received = value, received
                self.write(record_type='odom_raw', source_stamp_s=value.stamp,
                    frame_id=value.frame_id, child_frame_id=value.child_frame_id,
                    pose_x_m=value.pose.x, pose_y_m=value.pose.y, pose_yaw_rad=value.pose.yaw,
                    world_vx_mps=value.vx_world, world_vy_mps=value.vy_world,
                    odom_wz_radps=value.wz)
            except Exception as exc:
                self.write(record_type='invalid_odom', frame_id=msg.header.frame_id,
                           child_frame_id=msg.child_frame_id, reason=str(exc)[:160])
                if options.execute or options.ground_odom_straight:
                    self.failure = f'invalid odometry: {exc}'
                elif options.wheels_up_odom:
                    self.failure = f'invalid odometry: {exc}'
                elif options.wheels_up or options.wheels_up_odom:
                    self._wheels_up_warnings.add('invalid_odom_sample')

        def on_imu(self, msg):
            try:
                stamp = self.stamp(msg)
                if (options.execute or options.wheels_up or options.wheels_up_odom
                        or options.ground_odom_straight) and not msg.header.frame_id:
                    raise ValueError('IMU header.frame_id must be nonempty')
                vals = (stamp, msg.angular_velocity.z, msg.linear_acceleration.x,
                        msg.linear_acceleration.y)
                if not all(math.isfinite(float(x)) for x in vals):
                    raise ValueError('IMU contains nonfinite values')
                if self.imu_stamp is not None and stamp <= self.imu_stamp:
                    raise ValueError('IMU stamp did not advance')
                self.imu, self.imu_stamp = msg, stamp
                self.imu_received = time.monotonic()
                self.write(record_type='imu_data_raw', source_stamp_s=stamp,
                    imu_frame_id=msg.header.frame_id, imu_wz_radps=msg.angular_velocity.z,
                    imu_ax_mps2=msg.linear_acceleration.x,
                    imu_ay_mps2=msg.linear_acceleration.y)
            except Exception as exc:
                self.write(record_type='invalid_imu', source_stamp_s='',
                           imu_frame_id=msg.header.frame_id, reason=str(exc)[:160])
                if options.execute:
                    self.failure = f'invalid IMU: {exc}'
                elif options.wheels_up_odom:
                    self.failure = f'invalid IMU: {exc}'
                elif options.wheels_up or options.wheels_up_odom \
                        or options.ground_odom_straight:
                    self._wheels_up_warnings.add('invalid_imu_sample')

        def foreign_publishers(self):
            return sum(1 for ep in self.get_publishers_info_by_topic('/cmd_vel')
                if not (ep.node_name == self.get_name() and ep.node_namespace == self.get_namespace()))

        def ground_gate_diagnostics(self):
            ros_now = self.get_clock().now().nanoseconds / 1e9
            imu_age = source_stamp_age(ros_now, self.imu_stamp)
            warnings = set(self._wheels_up_warnings)
            if self.imu is None:
                warnings.add('imu_missing')
            if self.imu_stamp is None:
                warnings.add('imu_source_stamp_unavailable')
            elif imu_age is not None:
                if imu_age > MAX_SENSOR_AGE_S:
                    warnings.add('imu_source_stamp_stale')
                elif imu_age < -0.1:
                    warnings.add('imu_source_stamp_future')
            if self.imu_received is None:
                warnings.add('imu_receipt_unavailable')
            elif time.monotonic() - self.imu_received > MAX_SENSOR_AGE_S:
                warnings.add('imu_receipt_stale')
            return {
                'ros_now_s': ros_now,
                'odom_stamp_age_s': source_stamp_age(
                    ros_now, self.odom.stamp if self.odom else None),
                'imu_stamp_age_s': imu_age,
                'warning': ';'.join(sorted(warnings)),
            }

        def write_ground_gate_diagnostics(self, record_type, reason):
            diagnostics = self.ground_gate_diagnostics()
            self.write(record_type=record_type,
                source_stamp_s=(self.odom.stamp if self.odom else ''),
                frame_id=(self.odom.frame_id if self.odom else ''),
                child_frame_id=(self.odom.child_frame_id if self.odom else ''),
                reason=reason, **diagnostics)

        def gate(self):
            if options.wheels_up:
                foreign = self.foreign_publishers()
                if foreign:
                    return False, f'/cmd_vel has {foreign} other publisher(s)'
                return True, 'exclusive /cmd_vel ownership'
            if options.ground_odom_straight:
                return ground_odom_gate(time.monotonic(), self.odom_received,
                    self.foreign_publishers())
            if options.wheels_up_odom:
                allowed, reason = command_gate(time.monotonic(), self.odom_received,
                    self.imu_received, self.foreign_publishers())
                if not allowed:
                    return allowed, reason
                return source_stamp_gate(
                    self.get_clock().now().nanoseconds / 1e9,
                    self.odom.stamp if self.odom else None, self.imu_stamp)
            allowed, reason = command_gate(time.monotonic(), self.odom_received,
                                           self.imu_received, self.foreign_publishers())
            return allowed, reason

        def publish(self, vx, vy, wz, check=True):
            try:
                vx, vy, wz = limit_command(vx, vy, wz,
                    max_linear=(WHEELS_UP_MAX_SPEED_MPS if
                                (options.wheels_up or options.wheels_up_odom)
                                else GROUND_ODOM_MAX_COMMAND_SPEED_MPS if options.ground_odom_straight
                                else MAX_LINEAR_MPS))
            except ValueError as exc:
                raise RuntimeError(str(exc)) from exc
            if check:
                if self.failure:
                    raise RuntimeError(self.failure)
                allowed, reason = self.gate()
                if not allowed:
                    raise RuntimeError(reason)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
            self.publisher.publish(msg)
            self.write(record_type='cmd_vel_sent', cmd_vx_mps=vx,
                       cmd_vy_mps=vy, cmd_wz_radps=wz,
                       command_integral_m=(
                           getattr(self, '_ground_command_integral_m', '')
                           if options.ground_odom_straight else
                           getattr(self, '_wheels_budget', None).used
                           if getattr(self, '_wheels_budget', None) is not None else ''))

        def stop_window(self):
            errors = []

            def record_zero():
                try:
                    self.write(record_type='cmd_vel_sent', cmd_vx_mps=0.0,
                               cmd_vy_mps=0.0, cmd_wz_radps=0.0)
                except Exception as exc:
                    errors.append(exc)

            try:
                send_zero_window(self.publisher, Twist, on_publish=record_zero,
                                 on_error=errors.append)
            except Exception as exc:
                errors.append(exc)
            for exc in errors:
                self.get_logger().error(f'zero-command retry window error: {exc}')

        def run(self, stop_requested):
            deadline = time.monotonic() + WAIT_SENSORS_S
            needs_imu = not options.ground_odom_straight
            while (rclpy.ok() and not stop_requested.is_set()
                   and time.monotonic() < deadline
                   and (self.odom is None or (needs_imu and self.imu is None))):
                rclpy.spin_once(self, timeout_sec=0.05)
            if not (options.execute or options.wheels_up or options.wheels_up_odom
                    or options.ground_odom_straight):
                self.get_logger().info(f'read-only CSV recording: {self.csv_path}')
                while rclpy.ok() and not stop_requested.is_set():
                    rclpy.spin_once(self, timeout_sec=0.1)
                return
            if stop_requested.is_set():
                return
            if self.odom is None or (needs_imu and self.imu is None):
                raise RuntimeError('odometry and IMU required before execution'
                    if needs_imu else 'odometry required before ground trial')
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            discovery_end = time.monotonic() + DISCOVERY_S
            while (rclpy.ok() and not stop_requested.is_set()
                   and time.monotonic() < discovery_end):
                rclpy.spin_once(self, timeout_sec=0.05)
            if stop_requested.is_set():
                return
            allowed, reason = self.gate()
            if not allowed:
                if options.ground_odom_straight:
                    self.write_ground_gate_diagnostics(
                        'ground_gate_failure', reason)
                raise RuntimeError(reason)
            if options.wheels_up_odom:
                self.run_wheels_up_odom(stop_requested)
                return
            if options.ground_odom_straight:
                self.run_ground_odom_straight(stop_requested)
                return
            if options.wheels_up:
                self.run_wheels_up(stop_requested)
                return
            trial = StraightTrial(self.odom.pose, axis=options.axis,
                                  distance=options.distance)
            start = time.monotonic()
            next_tick = start
            last_command_time = start
            last_linear_command = (0.0, 0.0)
            command_budget = CommandDistanceBudget()
            completed = False
            while rclpy.ok() and not stop_requested.is_set():
                now = time.monotonic()
                command_budget.account(
                    *last_linear_command, now - last_command_time)
                last_command_time = now
                elapsed = now - start
                if elapsed > trial.duration + 2.0:
                    raise RuntimeError('straight trial timed out')
                if self.failure:
                    raise RuntimeError(self.failure)
                target = trial.target_pose
                error = math.hypot(self.odom.pose.x-target.x,
                                   self.odom.pose.y-target.y)
                speed = math.hypot(self.odom.vx_world, self.odom.vy_world)
                displacement, yaw_error = pose_envelope_error(
                    trial.start_pose, self.odom.pose)
                if displacement > 0.08 or yaw_error > 0.2:
                    raise RuntimeError('trial pose left the short-motion envelope')
                if trial_settled(
                        trial.start_pose, self.odom.pose, target,
                        distance=options.distance, elapsed=elapsed,
                        duration=trial.duration, measured_speed=speed,
                        position_error=error, yaw_error=yaw_error):
                    completed = True
                    break
                cmd = trial.sample(min(elapsed, trial.duration), self.odom.pose,
                                   (self.odom.vx_world, self.odom.vy_world),
                                   self.odom.wz)
                vx, vy = command_budget.limit_next(cmd.vx, cmd.vy)
                self.publish(vx, vy, cmd.wz)
                last_linear_command = (vx, vy)
                rclpy.spin_once(self, timeout_sec=0.0)
                next_tick = next_control_deadline(
                    next_tick, time.monotonic())
                remaining = next_tick - time.monotonic()
                if remaining > 0 and stop_requested.wait(remaining):
                    break
            if stop_requested.is_set():
                return
            if not completed:
                raise RuntimeError('trial ended without settling')

        def run_ground_odom_straight(self, stop_requested):
            """Explicitly opted-in, bounded ground segment using odometry feedback."""
            speed = options.ground_reference_speed or GROUND_ODOM_DEFAULT_SPEED_MPS
            decel = options.ground_a_dec or GROUND_ODOM_DEFAULT_DECEL_MPS2
            trial = StraightTrial(
                self.odom.pose, axis=options.axis, distance=options.distance,
                max_speed=speed, max_distance=GROUND_ODOM_MAX_DISTANCE_M,
                a_acc=GROUND_ODOM_DEFAULT_ACCEL_MPS2, a_dec=decel,
                kp_pos=(1.0 if options.ground_kp_pos is None
                        else options.ground_kp_pos),
                kd_vel=resolve_ground_kd_vel(options.ground_kd_vel))
            self.write(record_type='ground_trial_start',
                source_stamp_s=self.odom.stamp, frame_id=self.odom.frame_id,
                child_frame_id=self.odom.child_frame_id,
                pose_x_m=trial.start_pose.x, pose_y_m=trial.start_pose.y,
                pose_yaw_rad=trial.start_pose.yaw,
                ref_x_m=trial.target_pose.x, ref_y_m=trial.target_pose.y,
                ref_yaw_rad=trial.target_pose.yaw,
                reason=(f'axis={options.axis};distance={options.distance:.4f};'
                        f'speed={speed:.3f};a_dec={decel:.3f};'
                        f'kp_pos={trial.controller.kp_pos:.3f};'
                        f'kd_vel={trial.controller.kd_vel:.3f}'))
            start = time.monotonic()
            next_tick = start
            last_command_time = start
            last_command = (0.0, 0.0)
            command_integral_m = 0.0
            self._ground_command_integral_m = command_integral_m
            settle_state = (None, None, 0)
            stop_reason = 'unknown'
            try:
                while rclpy.ok() and not stop_requested.is_set():
                    now = time.monotonic()
                    command_integral_m = integrate_command_distance(
                        command_integral_m, *last_command,
                        now - last_command_time)
                    self._ground_command_integral_m = command_integral_m
                    last_command_time = now
                    elapsed = now - start
                    if elapsed > GROUND_ODOM_MAX_WALL_S:
                        stop_reason = 'wall_clock_limit'
                        raise RuntimeError('ground trial wall-clock limit exceeded')
                    if self.failure:
                        stop_reason = 'sensor_or_frame_failure'
                        raise RuntimeError(self.failure)
                    allowed, gate_reason = self.gate()
                    if not allowed:
                        stop_reason = f'sensor_or_publisher_gate:{gate_reason}'
                        self.write_ground_gate_diagnostics(
                            'ground_gate_failure', gate_reason)
                        raise RuntimeError(gate_reason)
                    target = trial.target_pose
                    measured = self.odom.pose
                    error = math.hypot(measured.x - target.x,
                                       measured.y - target.y)
                    speed_measured = math.hypot(self.odom.vx_world,
                                                self.odom.vy_world)
                    progress, cross_track = observed_trial_progress(
                        trial.start_pose, measured, target)
                    yaw_error = abs(norm_angle(measured.yaw -
                                               trial.start_pose.yaw))
                    if progress > options.distance + GROUND_ODOM_MAX_OVERSHOOT_M:
                        stop_reason = 'odom_overshoot_limit'
                        raise RuntimeError('ground trial odometry overshoot limit exceeded')
                    if cross_track > GROUND_ODOM_MAX_CROSS_TRACK_M:
                        stop_reason = 'cross_track_limit'
                        raise RuntimeError('ground trial cross-track limit exceeded')
                    if yaw_error > GROUND_ODOM_MAX_YAW_ERROR_RAD:
                        stop_reason = 'yaw_error_limit'
                        raise RuntimeError('ground trial yaw-error limit exceeded')
                    reference = trial.reference_at(min(elapsed, trial.duration))
                    settled = trial_settled(
                        trial.start_pose, measured, target,
                        distance=options.distance, elapsed=elapsed,
                        duration=trial.duration, measured_speed=speed_measured,
                        position_error=error, yaw_error=yaw_error,
                        tolerance_override=0.01,
                        minimum_progress_fraction=0.90,
                        max_speed=0.02, max_yaw_error=0.05)
                    settle_state, dwell_complete = update_settle_dwell(
                        settled, self.odom.stamp, now, settle_state,
                        dwell_s=0.25, min_samples=3)
                    self.write(record_type='ground_trial_sample',
                        source_stamp_s=self.odom.stamp,
                        frame_id=self.odom.frame_id,
                        child_frame_id=self.odom.child_frame_id,
                        pose_x_m=measured.x, pose_y_m=measured.y,
                        pose_yaw_rad=measured.yaw,
                        world_vx_mps=self.odom.vx_world,
                        world_vy_mps=self.odom.vy_world,
                        odom_wz_radps=self.odom.wz,
                        odom_forward_path_m=progress,
                        odom_lateral_path_m=cross_track,
                        command_integral_m=command_integral_m,
                        **self.ground_gate_diagnostics(),
                        ref_x_m=reference.x, ref_y_m=reference.y,
                        ref_yaw_rad=reference.yaw_ref,
                        pos_err_m=error, yaw_err_rad=yaw_error,
                        settled=dwell_complete,
                        reason=(f'progress={progress:.4f};cross_track={cross_track:.4f};'
                                f'one_shot=true'))
                    if dwell_complete:
                        stop_reason = 'settled'
                        return
                    command = trial.sample(min(elapsed, trial.duration), measured,
                        (self.odom.vx_world, self.odom.vy_world), self.odom.wz)
                    vx, vy, _ = limit_command(
                        command.vx, command.vy, command.wz,
                        max_linear=GROUND_ODOM_MAX_COMMAND_SPEED_MPS)
                    self.publish(vx, vy, command.wz)
                    last_command = (vx, vy)
                    rclpy.spin_once(self, timeout_sec=0.0)
                    next_tick = next_control_deadline(next_tick, time.monotonic())
                    remaining = next_tick - time.monotonic()
                    if remaining > 0 and stop_requested.wait(remaining):
                        stop_reason = 'operator_interrupt'
                        break
                stop_reason = ground_trial_loop_exit(
                    rclpy.ok(), stop_requested.is_set())
                if stop_reason == 'operator_interrupt':
                    return
                raise RuntimeError(
                    f'ground trial ended before settling: {stop_reason}')
            except Exception as exc:
                if stop_reason == 'unknown':
                    stop_reason = f'error:{type(exc).__name__}:{exc}'
                raise
            finally:
                command_integral_m = integrate_command_distance(
                    command_integral_m, *last_command,
                    max(0.0, time.monotonic() - last_command_time))
                self._ground_command_integral_m = command_integral_m
                self.write(record_type='ground_trial_stop',
                    source_stamp_s=(self.odom.stamp if self.odom else ''),
                    reason=stop_reason,
                    pose_x_m=(self.odom.pose.x if self.odom else ''),
                    pose_y_m=(self.odom.pose.y if self.odom else ''),
                    pose_yaw_rad=(self.odom.pose.yaw if self.odom else ''),
                    command_integral_m=command_integral_m,
                    **self.ground_gate_diagnostics())

        def run_wheels_up(self, stop_requested):
            """Command 10m integrated forward setpoint at the planner cruise speed."""
            start = time.monotonic()
            monitor = WheelSpinMonitor(self.odom.pose, self.odom_received)
            monitor.warnings.update(self._wheels_up_warnings)
            budget = CommandDistanceBudget(WHEELS_UP_MAX_COMMAND_PATH_M)
            self._wheels_budget = budget
            last_command_time = start
            last_command = (0.0, 0.0)
            last_odom_receipt = self.odom_received
            last_monitor_command_integral = 0.0
            next_tick = start
            last_progress_print = start
            self.get_logger().info(
                f'wheels-up diagnostic: 10m command-integrated forward setpoint '
                f'at {WHEELS_UP_MAX_SPEED_MPS:.2f}m/s (not a claim of 10m odom travel); '
                f'CSV {self.csv_path}')
            self.write(record_type='wheels_up_start',
                       reason='target_command_integral_m=10.0')
            while rclpy.ok() and not stop_requested.is_set():
                now = time.monotonic()
                budget.account(*last_command, now - last_command_time)
                last_command_time = now
                elapsed = now - start
                monitor.warnings.update(self._wheels_up_warnings)
                if elapsed >= WHEELS_UP_MAX_WALL_S:
                    raise RuntimeError('wheels-up diagnostic exceeded 20s wall-time cap')

                if self.odom_received != last_odom_receipt:
                    delta_command = budget.used - last_monitor_command_integral
                    try:
                        monitor.update(self.odom.pose, self.odom_received, delta_command)
                    except Exception as exc:
                        monitor.warnings.add('invalid_odom_telemetry')
                        self.write(record_type='wheels_up_warning', reason=str(exc)[:160],
                                   warning='invalid_odom_telemetry',
                                   command_integral_m=budget.used)
                    last_monitor_command_integral = budget.used
                    last_odom_receipt = self.odom_received
                    source_ok, source_reason = source_stamp_gate(
                        self.get_clock().now().nanoseconds / 1e9,
                        self.odom.stamp, self.imu_stamp)
                    if not source_ok:
                        monitor.warnings.add('source_stamp_warning')
                        self.write(record_type='wheels_up_warning', reason=source_reason,
                                   warning='source_stamp_warning',
                                   command_integral_m=budget.used)
                    displacement = math.hypot(self.odom.pose.x-monitor.initial_pose.x,
                                              self.odom.pose.y-monitor.initial_pose.y)
                    self.write(record_type='wheels_up_odom_progress',
                        source_stamp_s=self.odom.stamp, pose_x_m=self.odom.pose.x,
                        pose_y_m=self.odom.pose.y, pose_yaw_rad=self.odom.pose.yaw,
                        world_vx_mps=self.odom.vx_world, world_vy_mps=self.odom.vy_world,
                        command_integral_m=budget.used,
                        odom_forward_path_m=monitor.path,
                        odom_displacement_m=displacement,
                        odom_lateral_path_m=monitor.lateral_path,
                        warning=';'.join(sorted(monitor.warnings)))

                monitor.check_progress_timeout(now)
                if monitor.warnings:
                    warning_text = ';'.join(sorted(monitor.warnings))
                else:
                    warning_text = ''
                if now - last_progress_print >= 1.0:
                    if self.odom_received is None or now - self.odom_received > MAX_SENSOR_AGE_S:
                        monitor.warnings.add('odom_receipt_stale')
                    if self.imu_received is None or now - self.imu_received > MAX_SENSOR_AGE_S:
                        monitor.warnings.add('imu_receipt_stale')
                    source_ok, source_reason = source_stamp_gate(
                        self.get_clock().now().nanoseconds / 1e9,
                        self.odom.stamp, self.imu_stamp)
                    if not source_ok:
                        monitor.warnings.add('source_stamp_warning')
                    displacement = math.hypot(self.odom.pose.x-monitor.initial_pose.x,
                                              self.odom.pose.y-monitor.initial_pose.y)
                    self.get_logger().info(
                        f'wheels-up progress: command={budget.used:.3f}/'
                        f'{WHEELS_UP_TARGET_COMMAND_M:.3f}m, '
                        f'odom_forward={monitor.path:.3f}m, '
                        f'displacement={displacement:.3f}m, '
                        f'warnings={warning_text or "none"}')
                    self.write(record_type='wheels_up_progress',
                        reason=f'elapsed_s={elapsed:.3f}',
                        command_integral_m=budget.used,
                        odom_forward_path_m=monitor.path,
                        odom_displacement_m=displacement,
                        odom_lateral_path_m=monitor.lateral_path,
                        warning=warning_text)
                    last_progress_print = now

                if budget.used >= WHEELS_UP_TARGET_COMMAND_M:
                    self.publish(0.0, 0.0, 0.0)
                    self.write(record_type='wheels_up_complete',
                               reason='command_integral_target_reached',
                               command_integral_m=budget.used,
                               odom_forward_path_m=monitor.path,
                               warning=warning_text)
                    return

                allowed, reason = self.gate()
                if not allowed:
                    raise RuntimeError(reason)
                vx, vy = budget.limit_next(WHEELS_UP_MAX_SPEED_MPS, 0.0)
                remaining_time = WHEELS_UP_MAX_WALL_S - elapsed
                remaining_distance = WHEELS_UP_TARGET_COMMAND_M - budget.used
                vx = min(vx, remaining_distance / CONTROL_PERIOD_S,
                         WHEELS_UP_MAX_SPEED_MPS * remaining_time / CONTROL_PERIOD_S)
                if vx <= 0:
                    raise RuntimeError('wheels-up command budget exhausted before target')
                self.publish(vx, vy, 0.0)
                last_command = (vx, vy)
                rclpy.spin_once(self, timeout_sec=0.0)
                next_tick = next_control_deadline(next_tick, time.monotonic())
                remaining = next_tick - time.monotonic()
                if remaining > 0 and stop_requested.wait(remaining):
                    break
            if stop_requested.is_set():
                return
            raise RuntimeError('wheels-up diagnostic ended before command-integral target')

        def run_wheels_up_odom(self, stop_requested):
            """Exercise StraightTrial.sample against measured /odom_raw pose."""
            reference_speed = (options.wheels_up_odom_reference_speed
                if options.wheels_up_odom_reference_speed is not None
                else WHEELS_UP_ODOM_DEFAULT_REFERENCE_SPEED_MPS)
            trial = StraightTrial(
                self.odom.pose, axis='x', distance=WHEELS_UP_ODOM_TARGET_M,
                max_speed=reference_speed,
                max_distance=WHEELS_UP_ODOM_TARGET_M,
                a_acc=WHEELS_UP_ODOM_ACCEL_MPS2,
                a_dec=WHEELS_UP_ODOM_ACCEL_MPS2,
                kd_vel=(options.wheels_up_odom_kd_vel
                        if options.wheels_up_odom_kd_vel is not None else 0.0))
            start = time.monotonic()
            next_tick = start
            last_command_time = start
            last_command = (0.0, 0.0)
            last_progress_time = start
            last_progress = 0.0
            settle_window = (None, None, 0)
            budget = CommandDistanceBudget(WHEELS_UP_ODOM_MAX_COMMAND_M)
            self._wheels_budget = budget
            target = trial.target_pose
            self.get_logger().info(
                f'wheels-up odom-closed diagnostic: {WHEELS_UP_ODOM_TARGET_M:.2f}m, '
                f'reference {reference_speed:.2f}m/s cruise, '
                f'command cap {WHEELS_UP_MAX_SPEED_MPS:.2f}m/s, '
                f'kd_vel={trial.controller.kd_vel:.2f}, '
                f'{trial.duration:.2f}s reference profile; CSV {self.csv_path}')
            self.write(record_type='wheels_up_odom_start',
                pose_x_m=self.odom.pose.x, pose_y_m=self.odom.pose.y,
                pose_yaw_rad=self.odom.pose.yaw,
                ref_x_m=target.x, ref_y_m=target.y, ref_yaw_rad=target.yaw,
                reason=f'reference_speed_mps={reference_speed:.3f},'
                       f'profile_duration_s={trial.duration:.3f}')

            while rclpy.ok() and not stop_requested.is_set():
                now = time.monotonic()
                budget.account(*last_command, now - last_command_time)
                last_command_time = now
                elapsed = now - start
                if elapsed >= WHEELS_UP_ODOM_MAX_WALL_S:
                    raise RuntimeError('wheels-up odometry trial exceeded wall-time cap')
                if self.failure:
                    raise RuntimeError(self.failure)
                allowed, reason = self.gate()
                if not allowed:
                    raise RuntimeError(reason)

                pose = self.odom.pose
                error = math.hypot(target.x - pose.x, target.y - pose.y)
                yaw_error = abs(norm_angle(target.yaw - pose.yaw))
                speed = math.hypot(self.odom.vx_world, self.odom.vy_world)
                progress, cross_track = observed_trial_progress(
                    trial.start_pose, pose, target)
                # Keep the short wheels-up diagnostic in a broad envelope. The
                # measured pose remains the controller feedback; these bounds
                # only catch gross divergence and are not calibration claims.
                if cross_track > 0.25 or yaw_error > 0.30:
                    raise RuntimeError('wheels-up trial left the position/yaw envelope')
                if progress > WHEELS_UP_ODOM_TARGET_M + 0.25:
                    raise RuntimeError('wheels-up trial overshot target envelope')

                settled = trial_settled(
                    trial.start_pose, pose, target,
                    distance=WHEELS_UP_ODOM_TARGET_M, elapsed=elapsed,
                    duration=trial.duration, measured_speed=speed,
                    position_error=error, yaw_error=yaw_error,
                    tolerance_override=WHEELS_UP_ODOM_SETTLE_TOLERANCE_M,
                    minimum_progress_fraction=0.90,
                    max_speed=WHEELS_UP_ODOM_SETTLE_SPEED_MPS,
                    max_yaw_error=WHEELS_UP_ODOM_MAX_YAW_ERROR_RAD)
                if settled:
                    settle_window, settled = update_settle_dwell(
                        True, self.odom.stamp, now, settle_window,
                        dwell_s=WHEELS_UP_ODOM_SETTLE_DWELL_S)
                else:
                    settle_window, settled = update_settle_dwell(
                        False, self.odom.stamp, now, settle_window,
                        dwell_s=WHEELS_UP_ODOM_SETTLE_DWELL_S)
                speed_sample = trial.profile.sample(min(elapsed, trial.duration))
                ref_local = trial.reference.sample(
                    speed_sample.progress_s, speed_sample.speed,
                    speed_sample.acceleration)
                reference = trial.frame_transform.transform_reference(ref_local)
                next_command = None
                if not settled:
                    raw_command = trial.sample(min(elapsed, trial.duration), pose,
                        (self.odom.vx_world, self.odom.vy_world), self.odom.wz)
                    vx, vy = budget.limit_next(raw_command.vx, raw_command.vy)
                    next_command = (vx, vy, raw_command.wz)
                self.write(record_type='wheels_up_odom_control',
                    source_stamp_s=self.odom.stamp, frame_id=self.odom.frame_id,
                    child_frame_id=self.odom.child_frame_id,
                    pose_x_m=pose.x, pose_y_m=pose.y, pose_yaw_rad=pose.yaw,
                    world_vx_mps=self.odom.vx_world,
                    world_vy_mps=self.odom.vy_world, odom_wz_radps=self.odom.wz,
                    ref_x_m=reference.x, ref_y_m=reference.y,
                    ref_yaw_rad=reference.yaw_ref, pos_err_m=error,
                    yaw_err_rad=yaw_error, odom_forward_path_m=progress,
                    odom_lateral_path_m=cross_track,
                    command_integral_m=budget.used,
                    cmd_vx_mps=next_command[0] if next_command else 0.0,
                    cmd_vy_mps=next_command[1] if next_command else 0.0,
                    cmd_wz_radps=next_command[2] if next_command else 0.0,
                    settled=str(settled).lower())

                if settled:
                    # Recheck immediately before declaring completion; dwell
                    # must not outlive either receipt or source-stamp freshness.
                    allowed, reason = self.gate()
                    if not allowed:
                        raise RuntimeError(reason)
                    if time.monotonic() - self.odom_received > MAX_SENSOR_AGE_S:
                        raise RuntimeError('odometry became stale before settling completion')
                    self.publish(0.0, 0.0, 0.0)
                    self.write(record_type='wheels_up_odom_settled',
                        pose_x_m=pose.x, pose_y_m=pose.y,
                        ref_x_m=target.x, ref_y_m=target.y,
                        pos_err_m=error, yaw_err_rad=yaw_error,
                        odom_forward_path_m=progress,
                        world_vx_mps=self.odom.vx_world,
                        world_vy_mps=self.odom.vy_world,
                        command_integral_m=budget.used, settled='true')
                    post_stop_end = time.monotonic() + POST_STOP_S
                    while rclpy.ok() and not stop_requested.is_set() \
                            and time.monotonic() < post_stop_end:
                        rclpy.spin_once(self, timeout_sec=0.05)
                        if self.odom is not None:
                            self.write(record_type='wheels_up_odom_post_stop',
                                source_stamp_s=self.odom.stamp,
                                pose_x_m=self.odom.pose.x,
                                pose_y_m=self.odom.pose.y,
                                pose_yaw_rad=self.odom.pose.yaw,
                                world_vx_mps=self.odom.vx_world,
                                world_vy_mps=self.odom.vy_world,
                                pos_err_m=math.hypot(target.x-self.odom.pose.x,
                                                     target.y-self.odom.pose.y))
                    return
                if elapsed > trial.duration + WHEELS_UP_ODOM_SETTLE_TIMEOUT_S:
                    raise RuntimeError('wheels-up odometry trial failed measured settling')
                if progress >= last_progress + WHEELS_UP_MIN_PROGRESS_M:
                    last_progress_time = now
                    last_progress = progress
                if now - last_progress_time > WHEELS_UP_PROGRESS_TIMEOUT_S:
                    raise RuntimeError('wheels-up odometry trial has no measured progress')
                vx, vy, wz = next_command
                self.publish(vx, vy, wz)
                last_command = (vx, vy)
                rclpy.spin_once(self, timeout_sec=0.0)
                next_tick = next_control_deadline(next_tick, time.monotonic())
                remaining = next_tick - time.monotonic()
                if remaining > 0 and stop_requested.wait(remaining):
                    break
            if stop_requested.is_set():
                return
            raise RuntimeError('wheels-up odometry trial stopped before settling')

        def close(self):
            self.file.close()

    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions

    stop_requested = threading.Event()
    previous_handlers = None
    received_signal = {'signal': None}
    node = None
    initialized = False
    unexpected_shutdown = False
    try:
        rclpy.init(args=ros_args,
                   signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        previous_handlers, received_signal = install_stop_signal_handlers(
            stop_requested)
        node = ControlProbeNode()
        try:
            node.run(stop_requested)
        except ExternalShutdownException:
            # Unexpected context shutdown still enters the single best-effort
            # emergency-zero path in finally below.
            stop_requested.set()
            unexpected_shutdown = True
    finally:
        if node is not None:
            if (options.execute or options.wheels_up or options.wheels_up_odom
                    or options.ground_odom_straight) and node.publisher is not None:
                try:
                    node.stop_window()
                except Exception as exc:
                    node.get_logger().error(f'emergency zero window failed: {exc}')
            try:
                node.close()
            except Exception as exc:
                node.get_logger().error(f'CSV close failed: {exc}')
            try:
                node.destroy_node()
            except Exception as exc:
                node.get_logger().error(f'node destroy failed: {exc}')
        try:
            shutdown_ros_context(rclpy, initialized)
        finally:
            if previous_handlers is not None:
                restore_signal_handlers(previous_handlers)
    if unexpected_shutdown and received_signal['signal'] is None:
        return 1
    if received_signal['signal'] is not None:
        return 128 + int(received_signal['signal'])


if __name__ == '__main__':
    main()
