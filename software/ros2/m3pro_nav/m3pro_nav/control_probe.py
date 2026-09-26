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
import time

from .motion_primitive import MotionPrimitive
from .pose import Pose2D
from .frame_transform import RigidFrameTransform
from .position_controller import PositionController
from .speed_profile import SpeedProfile
from .trajectory_reference import TrajectoryReference
from .odometry_adapter import OdometryMonitor, odometry_from_msg

MAX_DISTANCE_M = 0.03
MAX_LINEAR_MPS = 0.05
MAX_ANGULAR_RADPS = 0.15
MAX_SENSOR_AGE_S = 0.5
CONTROL_PERIOD_S = 0.02
POST_STOP_S = 0.5
DISCOVERY_S = 1.0
WAIT_SENSORS_S = 3.0


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
            end = time.monotonic() + POST_STOP_S
            last_error = None
            successful = 0
            while rclpy.ok() and time.monotonic() < end:
                try:
                    self.publish(0.0, 0.0, 0.0, check=False)
                    successful += 1
                except Exception as exc:
                    # A CSV flush or transient publisher error must not suppress
                    # the remaining emergency zero attempts.
                    last_error = exc
                try:
                    rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD_S)
                except Exception as exc:
                    last_error = exc
            if last_error is not None:
                self.get_logger().error(f'zero-command retry window saw error: {last_error}')
            if successful == 0:
                raise RuntimeError('all emergency zero-command attempts failed') from last_error

        def run(self):
            deadline = time.monotonic() + WAIT_SENSORS_S
            while rclpy.ok() and time.monotonic() < deadline and (self.odom is None or self.imu is None):
                rclpy.spin_once(self, timeout_sec=0.05)
            if not options.execute:
                self.get_logger().info(f'read-only CSV recording: {self.csv_path}')
                rclpy.spin(self)
                return
            if self.odom is None or self.imu is None:
                raise RuntimeError('odometry and IMU required before execution')
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            discovery_end = time.monotonic() + DISCOVERY_S
            while rclpy.ok() and time.monotonic() < discovery_end:
                rclpy.spin_once(self, timeout_sec=0.05)
            allowed, reason = self.gate()
            if not allowed:
                raise RuntimeError(reason)
            trial = StraightTrial(self.odom.pose, axis=options.axis,
                                  distance=options.distance)
            start = time.monotonic()
            completed = False
            failure = None
            try:
                while rclpy.ok():
                    now = time.monotonic()
                    elapsed = now - start
                    if elapsed > trial.duration + 2.0:
                        raise RuntimeError('straight trial timed out')
                    if self.failure:
                        raise RuntimeError(self.failure)
                    target = trial.target_pose
                    error = math.hypot(self.odom.pose.x-target.x,
                                       self.odom.pose.y-target.y)
                    speed = math.hypot(self.odom.vx_world, self.odom.vy_world)
                    displacement = math.hypot(self.odom.pose.x-trial.start_pose.x,
                                              self.odom.pose.y-trial.start_pose.y)
                    yaw_error = abs(norm_angle(self.odom.pose.yaw-trial.start_pose.yaw))
                    if displacement > 0.08 or yaw_error > 0.2:
                        raise RuntimeError('trial pose left the short-motion envelope')
                    if (elapsed >= trial.duration and error <= 0.01 and
                            speed <= 0.02 and yaw_error <= 0.05):
                        completed = True
                        break
                    cmd = trial.sample(min(elapsed, trial.duration), self.odom.pose,
                                       (self.odom.vx_world, self.odom.vy_world),
                                       self.odom.wz)
                    self.publish(cmd.vx, cmd.vy, cmd.wz)
                    rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD_S)
            except Exception as exc:
                failure = exc
            finally:
                # Emergency stop does not depend on sensor/ownership checks.
                self.stop_window()
            if failure is not None:
                raise failure
            if not completed:
                raise RuntimeError('trial ended without settling')

        def close(self):
            self.file.close()

    rclpy.init(args=ros_args)
    node = ControlProbeNode()
    try:
        node.run()
    finally:
        try:
            node.close()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
