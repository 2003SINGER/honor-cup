"""Safe read-only ROS driver probe with an explicitly gated motion step.

Default mode subscribes and records sensor/command topics. Motion requires
``--execute`` plus a bounded axis, magnitude, and duration. Hardware execution
has not been validated from this development host.
"""

import argparse
from dataclasses import dataclass
import csv
import math
from pathlib import Path
import time


MAX_LINEAR_STEP = 0.05  # m/s, deliberately small and still uncalibrated
MAX_YAW_STEP = 0.15  # rad/s
MAX_STEP_DURATION = 0.5  # seconds
MAX_SENSOR_AGE = 0.5  # seconds
SENSOR_WAIT_TIMEOUT = 3.0  # seconds
COMMAND_PERIOD = 0.05  # seconds
POST_STEP_SETTLE = 2.0  # seconds
PUBLISHER_DISCOVERY_WAIT = 1.0  # seconds


@dataclass(frozen=True)
class ProbeOptions:
    execute: bool = False
    axis: str | None = None
    magnitude: float | None = None
    duration: float | None = None
    csv_path: str | None = None


def validate_options(options: ProbeOptions) -> ProbeOptions:
    """Validate pure CLI safety constraints without importing ROS libraries."""
    if not options.execute:
        if options.axis is not None or options.magnitude is not None or options.duration is not None:
            raise ValueError('motion parameters require --execute')
        return options
    if options.axis not in ('x', 'y', 'yaw'):
        raise ValueError('--execute requires --axis x|y|yaw')
    if options.magnitude is None or not math.isfinite(options.magnitude) or options.magnitude == 0:
        raise ValueError('--execute requires a finite nonzero --magnitude')
    limit = MAX_YAW_STEP if options.axis == 'yaw' else MAX_LINEAR_STEP
    if abs(options.magnitude) > limit:
        raise ValueError(f'{options.axis} magnitude exceeds safety cap {limit}')
    if options.duration is None or not math.isfinite(options.duration) or options.duration <= 0:
        raise ValueError('--execute requires a finite positive --duration')
    if options.duration > MAX_STEP_DURATION:
        raise ValueError(f'duration exceeds safety cap {MAX_STEP_DURATION}s')
    return options


def execution_gate(now: float, odom_received: float | None,
                   imu_received: float | None, foreign_publishers: int) -> tuple[bool, str]:
    """Return whether fresh feedback and exclusive /cmd_vel ownership permit a step."""
    if odom_received is None or now - odom_received > MAX_SENSOR_AGE:
        return False, 'odometry missing or stale'
    if imu_received is None or now - imu_received > MAX_SENSOR_AGE:
        return False, 'IMU missing or stale'
    if foreign_publishers != 0:
        return False, f'/cmd_vel has {foreign_publishers} other publisher(s)'
    return True, 'ready'


def twist_values(axis: str, magnitude: float) -> tuple[float, float, float]:
    """Return the requested x/y/yaw velocity components."""
    if axis == 'x':
        return magnitude, 0.0, 0.0
    if axis == 'y':
        return 0.0, magnitude, 0.0
    if axis == 'yaw':
        return 0.0, 0.0, magnitude
    raise ValueError('axis must be x, y, or yaw')


CSV_FIELDS = (
    'received_monotonic_s', 'record_type', 'source_stamp_s',
    'pose_x_m', 'pose_y_m', 'pose_yaw_rad',
    'odom_vx_mps', 'odom_vy_mps', 'odom_wz_radps',
    'imu_wz_radps', 'imu_ax_mps2', 'imu_ay_mps2',
    'cmd_vx_mps', 'cmd_vy_mps', 'cmd_wz_radps',
)


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='enable one bounded step')
    parser.add_argument('--axis', choices=('x', 'y', 'yaw'))
    parser.add_argument('--magnitude', type=float)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--csv', dest='csv_path')
    return parser


def main(args=None):
    parsed, ros_args = _parser().parse_known_args(args)
    options = validate_options(ProbeOptions(
        execute=parsed.execute, axis=parsed.axis, magnitude=parsed.magnitude,
        duration=parsed.duration, csv_path=parsed.csv_path))

    # Keep imports lazy so argument and safety-gate logic can be checked on hosts
    # that do not have ROS 2 installed.
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class DriverProbeNode(Node):
        def __init__(self):
            super().__init__('driver_probe_' + str(int(time.time() * 1000) % 1000000))
            self.csv_path = Path(options.csv_path or
                                 f'/tmp/m3pro_driver_probe_{time.strftime("%Y%m%d_%H%M%S")}.csv')
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
            self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
            self._writer.writeheader()
            self._latest = {'odom': None, 'imu': None}
            self._received = {'odom': None, 'imu': None}
            self._cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
            self._odom_sub = self.create_subscription(Odometry, '/odom_raw', self._on_odom, 10)
            self._imu_sub = self.create_subscription(Imu, '/imu/data_raw', self._on_imu, 10)
            self._publisher = None
            self._motion_started = False

        def _write(self, **values):
            row = {field: '' for field in CSV_FIELDS}
            row.update(received_monotonic_s=f'{time.monotonic():.6f}', **values)
            self._writer.writerow(row)
            self._csv_file.flush()

        @staticmethod
        def _stamp(msg):
            return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        def _on_cmd(self, msg):
            self._write(record_type='cmd_vel_observed',
                        cmd_vx_mps=msg.linear.x, cmd_vy_mps=msg.linear.y,
                        cmd_wz_radps=msg.angular.z)

        def _on_odom(self, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            p, v = msg.pose.pose.position, msg.twist.twist
            self._latest['odom'] = msg
            self._received['odom'] = time.monotonic()
            self._write(record_type='odom_raw', source_stamp_s=self._stamp(msg),
                        pose_x_m=p.x, pose_y_m=p.y, pose_yaw_rad=yaw,
                        odom_vx_mps=v.linear.x, odom_vy_mps=v.linear.y,
                        odom_wz_radps=v.angular.z)

        def _on_imu(self, msg):
            self._latest['imu'] = msg
            self._received['imu'] = time.monotonic()
            self._write(record_type='imu_data_raw', source_stamp_s=self._stamp(msg),
                        imu_wz_radps=msg.angular_velocity.z,
                        imu_ax_mps2=msg.linear_acceleration.x,
                        imu_ay_mps2=msg.linear_acceleration.y)

        def _foreign_cmd_publishers(self):
            return sum(1 for endpoint in self.get_publishers_info_by_topic('/cmd_vel')
                       if not (endpoint.node_name == self.get_name() and
                               endpoint.node_namespace == self.get_namespace()))

        def _gate(self):
            return execution_gate(time.monotonic(), self._received['odom'],
                                  self._received['imu'], self._foreign_cmd_publishers())

        def _publish(self, vx, vy, wz, *, safety_check=True):
            if safety_check:
                allowed, reason = self._gate()
                if not allowed:
                    raise RuntimeError(f'motion safety gate: {reason}')
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
            self._publisher.publish(msg)
            self._write(record_type='cmd_vel_sent', cmd_vx_mps=vx,
                        cmd_vy_mps=vy, cmd_wz_radps=wz)

        def run_probe(self):
            deadline = time.monotonic() + SENSOR_WAIT_TIMEOUT
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._received['odom'] is not None and self._received['imu'] is not None:
                    break
            if not options.execute:
                self.get_logger().info(f'read-only probe recording to {self.csv_path}')
                rclpy.spin(self)
                return
            if self._received['odom'] is None or self._received['imu'] is None:
                raise RuntimeError('motion safety gate: odometry and IMU data required')
            self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            # Let DDS discovery observe the newly created endpoint before checking
            # that there are no competing publishers.
            discovery_end = time.monotonic() + PUBLISHER_DISCOVERY_WAIT
            while rclpy.ok() and time.monotonic() < discovery_end:
                rclpy.spin_once(self, timeout_sec=0.05)
            allowed, reason = self._gate()
            if not allowed:
                raise RuntimeError(f'motion safety gate: {reason}')
            vx, vy, wz = twist_values(options.axis, options.magnitude)
            self._motion_started = True
            end = time.monotonic() + options.duration
            try:
                while rclpy.ok() and time.monotonic() < end:
                    self._publish(vx, vy, wz)
                    rclpy.spin_once(self, timeout_sec=COMMAND_PERIOD)
            finally:
                try:
                    self._publish(0.0, 0.0, 0.0, safety_check=False)
                finally:
                    self._motion_started = False
            settle_end = time.monotonic() + POST_STEP_SETTLE
            while rclpy.ok() and time.monotonic() < settle_end:
                rclpy.spin_once(self, timeout_sec=0.05)

        def close(self):
            try:
                if self._motion_started and self._publisher is not None:
                    self._publish(0.0, 0.0, 0.0, safety_check=False)
                    self._motion_started = False
            finally:
                self._csv_file.close()

    rclpy.init(args=ros_args)
    node = DriverProbeNode()
    try:
        node.run_probe()
    finally:
        try:
            node.close()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
