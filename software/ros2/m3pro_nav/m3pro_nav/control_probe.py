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
        allowed = remaining / period
        if speed > allowed:
            if allowed <= 0.0:
                raise RuntimeError('integrated command-distance budget exhausted')
            scale = allowed / speed
            vx, vy = vx * scale, vy * scale
        return vx, vy


@dataclass(frozen=True)
class ProbeOptions:
    execute: bool = False
    expected_odom_frame: str | None = None
    expected_base_frame: str | None = None
    axis: str | None = None
    distance: float | None = None
    csv_path: str | None = None


def validate_options(options: ProbeOptions) -> ProbeOptions:
    """Validate CLI constraints without importing ROS."""
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


def limit_command(vx: float, vy: float, wz: float):
    """Reject invalid/extreme controller output and apply configured caps."""
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        raise ValueError('refusing nonfinite command')
    if abs(vx) > 1.0 or abs(vy) > 1.0 or abs(wz) > 1.0:
        raise ValueError('controller command is extreme')
    speed = math.hypot(vx, vy)
    if speed > MAX_LINEAR_MPS:
        scale = MAX_LINEAR_MPS / speed
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
                  position_error: float, yaw_error: float):
    """Require observable forward progress, target accuracy, and low speed."""
    values = (distance, elapsed, duration, measured_speed, position_error,
              yaw_error)
    if not all(math.isfinite(value) for value in values):
        return False
    tolerance = min(0.01, max(0.001, distance * 0.25))
    minimum_progress = max(MIN_OBSERVED_PROGRESS_M,
                           distance * MIN_OBSERVED_PROGRESS_FRACTION)
    progress, cross_track = observed_trial_progress(
        start_pose, measured_pose, target_pose)
    return (elapsed >= duration and position_error <= tolerance
            and progress >= minimum_progress and cross_track <= tolerance
            and measured_speed <= 0.02 and yaw_error <= 0.05)


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
                 max_speed=MAX_LINEAR_MPS):
        if axis not in ('x', 'y') or not all(math.isfinite(v) for v in
                (start_pose.x, start_pose.y, start_pose.yaw, distance, max_speed)):
            raise ValueError('invalid straight-trial input')
        if distance <= 0 or distance > MAX_DISTANCE_M or max_speed <= 0 or max_speed > MAX_LINEAR_MPS:
            raise ValueError('straight trial exceeds safety limits')
        # Construct geometry in an axis-aligned local frame, then transform
        # references into odom. MotionPrimitive intentionally rejects diagonals.
        dx, dy = (distance, 0.0) if axis == 'x' else (0.0, distance)
        primitive = MotionPrimitive(kind='STRAIGHT', start_pose=Pose2D(0.0, 0.0, 0.0),
            p0=(0.0, 0.0), p1=(dx, dy), yaw0=0.0,
            length=distance, v_max=max_speed, v_end=0.0)
        # Conservative trial parameterization, still provisional pending calibration.
        self.profile = SpeedProfile((primitive,), start_speed=0.0, a_acc=0.05, a_dec=0.05)
        self.reference = TrajectoryReference((primitive,), yaw_ref=0.0)
        self.controller = PositionController()
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
        speed = self.profile.sample(elapsed)
        reference = self.frame_transform.transform_reference(self.reference.sample(
            speed.progress_s, speed.speed, speed.acceleration))
        return self.controller.update(reference, measured_pose,
            measured_velocity_world=measured_velocity_world, yaw_rate=yaw_rate)


CSV_FIELDS = ('received_monotonic_s', 'record_type', 'source_stamp_s', 'frame_id',
              'child_frame_id', 'reason', 'pose_x_m', 'pose_y_m', 'pose_yaw_rad',
              'world_vx_mps', 'world_vy_mps', 'odom_wz_radps', 'imu_frame_id',
              'imu_wz_radps', 'imu_ax_mps2', 'imu_ay_mps2', 'cmd_vx_mps',
              'cmd_vy_mps', 'cmd_wz_radps')


def _parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--expected-odom-frame')
    p.add_argument('--expected-base-frame')
    p.add_argument('--axis', choices=('x', 'y'))
    p.add_argument('--distance', type=float)
    p.add_argument('--csv', dest='csv_path')
    return p


def main(args=None):
    parsed, ros_args = _parser().parse_known_args(args)
    options = validate_options(ProbeOptions(parsed.execute, parsed.expected_odom_frame,
        parsed.expected_base_frame, parsed.axis, parsed.distance, parsed.csv_path))

    # Delayed ROS imports keep pure logic importable on macOS.
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class ControlProbeNode(Node):
        def __init__(self):
            super().__init__('m3pro_control_probe_' + str(int(time.time()*1000) % 1000000))
            self.csv_path = Path(options.csv_path or
                f'/tmp/m3pro_control_probe_{time.strftime("%Y%m%d_%H%M%S")}.csv')
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            self.file = self.csv_path.open('w', newline='', encoding='utf-8')
            self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
            self.writer.writeheader()
            self.odom = self.imu = None
            self.odom_received = self.imu_received = None
            self.imu_stamp = None
            self.odom_monitor = OdometryMonitor(max_age=MAX_SENSOR_AGE_S)
            self.failure = None
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
                if options.execute:
                    self.failure = f'invalid odometry: {exc}'

        def on_imu(self, msg):
            try:
                stamp = self.stamp(msg)
                if options.execute and not msg.header.frame_id:
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

        def foreign_publishers(self):
            return sum(1 for ep in self.get_publishers_info_by_topic('/cmd_vel')
                if not (ep.node_name == self.get_name() and ep.node_namespace == self.get_namespace()))

        def gate(self):
            return command_gate(time.monotonic(), self.odom_received,
                                self.imu_received, self.foreign_publishers())

        def publish(self, vx, vy, wz, check=True):
            try:
                vx, vy, wz = limit_command(vx, vy, wz)
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
                       cmd_vy_mps=vy, cmd_wz_radps=wz)

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
            while (rclpy.ok() and not stop_requested.is_set()
                   and time.monotonic() < deadline
                   and (self.odom is None or self.imu is None)):
                rclpy.spin_once(self, timeout_sec=0.05)
            if not options.execute:
                self.get_logger().info(f'read-only CSV recording: {self.csv_path}')
                while rclpy.ok() and not stop_requested.is_set():
                    rclpy.spin_once(self, timeout_sec=0.1)
                return
            if stop_requested.is_set():
                return
            if self.odom is None or self.imu is None:
                raise RuntimeError('odometry and IMU required before execution')
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            discovery_end = time.monotonic() + DISCOVERY_S
            while (rclpy.ok() and not stop_requested.is_set()
                   and time.monotonic() < discovery_end):
                rclpy.spin_once(self, timeout_sec=0.05)
            if stop_requested.is_set():
                return
            allowed, reason = self.gate()
            if not allowed:
                raise RuntimeError(reason)
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
            if options.execute and node.publisher is not None:
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
