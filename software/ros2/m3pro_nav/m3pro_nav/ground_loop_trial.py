"""Logged odometry-closed loop through one fixed-template left arc and back.

This is an explicitly enabled, one-shot field diagnostic. It does not use the
map or lidar for control; /odom_raw closes the position loop and the concurrent
rosbag is responsible for preserving raw scan data.
"""

import argparse
import csv
import math
from pathlib import Path
import signal
import time

from .feedback_trajectory_follower import FeedbackTrajectoryFollower
from .motion_planner import MotionPlanner, validate_geometry
from .motion_primitive import MotionPrimitive
from .odometry_adapter import OdometryMonitor, odometry_from_msg
from .pose import Pose2D
from .position_controller import PositionController

from .control_probe import (CONTROL_PERIOD_S, MAX_SENSOR_AGE_S,
    ground_odom_gate, limit_command, next_control_deadline, send_zero_window)

CELL_M = 0.4
SPEED_MPS = 0.50
MAX_REFERENCE_SPEED_MPS = 0.50
ACCEL_MPS2 = 1.00
POSITION_GAIN = 1.0
VELOCITY_DAMPING = 0.20
MAX_DISPLACEMENT_M = 0.65
MAX_REFERENCE_ERROR_M = 0.12
MAX_YAW_ERROR_RAD = 0.25
MAX_COMMAND_MPS = 0.70
MAX_COMMAND_CAP_MPS = 0.70
MAX_WALL_S = 18.0
SETTLE_TIMEOUT_S = 5.0

CSV_FIELDS = (
    'monotonic_s', 'record_type', 'segment', 'primitive_index',
    'source_stamp_s', 'frame_id', 'child_frame_id',
    'pose_x_m', 'pose_y_m', 'pose_yaw_rad', 'world_vx_mps',
    'world_vy_mps', 'odom_wz_radps', 'ref_x_m', 'ref_y_m',
    'ref_yaw_rad', 'ref_vx_mps', 'ref_vy_mps', 'position_error_m',
    'yaw_error_rad', 'cmd_vx_mps', 'cmd_vy_mps', 'cmd_wz_radps',
    'displacement_from_start_m', 'reason')


def compile_loop_route(start_cell=(6, 2), speed=SPEED_MPS):
    """Return fixed-template primitives that leave and return to start center.

    Chassis yaw remains fixed by FeedbackTrajectoryFollower. Grid heading N is
    +Y in planner coordinates. Partial half-cell straights join the template
    edge anchors to cell centers; turns themselves come from MotionPlanner.
    """
    if start_cell != (6, 2):
        raise ValueError('diagnostic route is defined only for start cell (6, 2)')
    if not math.isfinite(speed) or not 0.05 <= speed <= MAX_REFERENCE_SPEED_MPS:
        raise ValueError(f'speed must be in [0.05, {MAX_REFERENCE_SPEED_MPS:.2f}] m/s')
    planner = MotionPlanner()
    cell = start_cell
    center = ((cell[0] + 0.5) * CELL_M, (cell[1] + 0.5) * CELL_M)
    yaw_north = math.pi / 2
    pose = Pose2D(*center, yaw_north)
    primitives, point, _ = planner.template(None, cell, (6, 3), pose)
    pose = Pose2D(point[0], point[1], yaw_north)

    arc, point, _ = planner.template((6, 2), (6, 3), (5, 3), pose)
    arc[0].meta['field_trial_segment'] = 'left_arc_out'
    primitives.extend(arc)
    pose = Pose2D(point[0], point[1], yaw_north)

    west_center = ((5 + 0.5) * CELL_M, (3 + 0.5) * CELL_M)

    def straight(a, b, end_speed, label):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        prim = MotionPrimitive('STRAIGHT', Pose2D(a[0], a[1], yaw_north),
            p0=a, p1=b, yaw0=yaw_north, length=length,
            v_max=speed, v_end=end_speed, meta={'field_trial_segment': label})
        return prim

    primitives.append(straight((pose.x, pose.y), west_center, 0.0,
                               'west_to_cell_center'))
    pose = Pose2D(*west_center, yaw_north)
    arc_entry = (pose.x + CELL_M / 2, pose.y)
    primitives.append(straight((pose.x, pose.y), arc_entry, speed,
                               'east_back_to_arc'))
    pose = Pose2D(*arc_entry, yaw_north)

    reverse_arc, point, _ = planner.template((5, 3), (6, 3), (6, 2), pose)
    reverse_arc[0].meta['field_trial_segment'] = 'right_arc_back'
    primitives.extend(reverse_arc)
    pose = Pose2D(point[0], point[1], yaw_north)
    start_center = center
    primitives.append(straight((pose.x, pose.y), start_center, 0.0,
                               'south_back_to_start'))
    pose = Pose2D(*start_center, yaw_north)
    primitives.append(MotionPrimitive('STOP', pose, duration=0.0,
                                      meta={'field_trial_terminal': True}))

    # Straight speed is configurable; arcs retain the planner's V_ARC ceiling.
    # Keep the two direction reversals stopped at the intermediate cell center.
    for primitive in primitives:
        if primitive.kind != 'STOP':
            primitive.v_max = min(speed, planner.v_arc) if primitive.kind == 'ARC' else speed
            if primitive.meta.get('field_trial_segment') == 'west_to_cell_center':
                primitive.v_end = 0.0
            elif primitive.meta.get('field_trial_segment') == 'south_back_to_start':
                primitive.v_end = 0.0
            elif primitive.kind == 'ARC':
                primitive.v_end = primitive.v_max
            else:
                primitive.v_end = speed
    validate_geometry(primitives)
    return tuple(primitives)


def validate_trial_limits(speed, command_cap):
    """Validate the deliberately narrow gradual-speed comparison bounds."""
    if not math.isfinite(speed) or not 0.05 <= speed <= MAX_REFERENCE_SPEED_MPS:
        raise ValueError(f'speed must be in [0.05, {MAX_REFERENCE_SPEED_MPS:.2f}] m/s')
    if not math.isfinite(command_cap) or speed > command_cap \
            or command_cap > MAX_COMMAND_CAP_MPS or command_cap <= 0:
        raise ValueError(
            f'command cap must be >= speed and <= {MAX_COMMAND_CAP_MPS:.2f} m/s')
    return speed, command_cap


def nominal_route_end(primitives):
    """Return final point and total geometric path length (STOP excluded)."""
    x = y = length = 0.0
    for primitive in primitives:
        if primitive.kind == 'STRAIGHT':
            x, y = primitive.p1
            length += primitive.length
        elif primitive.kind == 'ARC':
            angle = primitive.yaw0 + primitive.yaw1
            radius = primitive.meta['r']
            x = primitive.p0[0] + radius * math.cos(angle)
            y = primitive.p0[1] + radius * math.sin(angle)
            length += primitive.length
    return (x, y), length


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', action='store_true', required=True,
                   help='explicitly execute the (6,2) out-and-back route')
    p.add_argument('--expected-odom-frame', required=True)
    p.add_argument('--expected-base-frame', required=True)
    p.add_argument('--csv', default='/tmp/ground_loop_trial.csv')
    p.add_argument('--speed', type=float, default=SPEED_MPS)
    p.add_argument('--command-cap', type=float, default=MAX_COMMAND_MPS,
                   help='linear command cap in m/s (must be >= speed and <= 0.70)')
    return p


def main(args=None):
    options = parser().parse_args(args)
    validate_trial_limits(options.speed, options.command_cap)
    primitives = compile_loop_route(speed=options.speed)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    stop = {'requested': False}
    old_handlers = {}
    def on_signal(signum, _frame):
        stop['requested'] = True
    class TrialNode(Node):
        def __init__(self):
            super().__init__('ground_loop_trial')
            path = Path(options.csv)
            path.parent.mkdir(parents=True, exist_ok=True)
            stamp = f'{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns()%1_000_000_000:09d}'
            self.csv_path = path.with_name(f'{path.stem}_{stamp}{path.suffix or ".csv"}')
            self.file = self.csv_path.open('x', newline='', encoding='utf-8')
            self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
            self.writer.writeheader()
            self.odom = self.odom_received = None
            self.monitor = OdometryMonitor(max_age=MAX_SENSOR_AGE_S)
            self.failure = None
            self.publisher = None
            self._zero_sent = False
            self.create_subscription(Odometry, '/odom_raw', self.on_odom, 10)

        def write(self, record_type, **values):
            row = {key: '' for key in CSV_FIELDS}
            row.update(monotonic_s=f'{time.monotonic():.6f}',
                       record_type=record_type, **values)
            self.writer.writerow(row)
            self.file.flush()

        def on_odom(self, msg):
            try:
                sample = odometry_from_msg(msg,
                    expected_odom_frame=options.expected_odom_frame,
                    expected_base_frame=options.expected_base_frame)
                received = time.monotonic()
                self.monitor.accept(sample, now=received, received=received,
                    expected_odom_frame=options.expected_odom_frame,
                    expected_base_frame=options.expected_base_frame)
                self.odom, self.odom_received = sample, received
            except Exception as exc:
                self.failure = str(exc)

        def foreign_publishers(self):
            return sum(1 for endpoint in self.get_publishers_info_by_topic('/cmd_vel')
                if not (endpoint.node_name == self.get_name()
                        and endpoint.node_namespace == self.get_namespace()))

        def zero(self):
            if self.publisher is None or self._zero_sent:
                return
            send_zero_window(self.publisher, Twist,
                on_publish=lambda: self.write('emergency_zero',
                    cmd_vx_mps=0.0, cmd_vy_mps=0.0, cmd_wz_radps=0.0))
            self._zero_sent = True

        def execute(self):
            wait_until = time.monotonic() + 4.0
            while rclpy.ok() and not stop['requested'] and self.odom is None and time.monotonic() < wait_until:
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is None:
                raise RuntimeError(f'no valid /odom_raw before timeout: {self.failure or "missing"}')
            if stop['requested']:
                return
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            discovery_until = time.monotonic() + 1.0
            while rclpy.ok() and not stop['requested'] and time.monotonic() < discovery_until:
                rclpy.spin_once(self, timeout_sec=0.05)
            allowed, reason = ground_odom_gate(time.monotonic(), self.odom_received,
                                               self.foreign_publishers())
            if not allowed:
                raise RuntimeError(f'preflight: {reason}')
            if stop['requested']:
                return

            start_odom = self.odom
            initial_center = (6.5 * CELL_M, 2.5 * CELL_M)
            follower = FeedbackTrajectoryFollower(
                primitives, planner_start=Pose2D(*initial_center, math.pi / 2),
                odom_start=start_odom.pose, a_acc=ACCEL_MPS2,
                a_dec=ACCEL_MPS2,
                controller=PositionController(kp_pos=POSITION_GAIN,
                                              kd_vel=VELOCITY_DAMPING),
                start_speed=0.0, position_tolerance=0.025,
                yaw_tolerance=0.06, velocity_tolerance=0.025,
                yaw_rate_tolerance=0.15, settle_time=0.30,
                expected_odom_frame=options.expected_odom_frame,
                expected_base_frame=options.expected_base_frame)
            profile_duration = follower.profile.duration
            self.get_logger().info(
                f'out-and-back: length={follower.profile.length:.3f}m, '
                f'profile={profile_duration:.2f}s, start=(6,2) N, '
                f'CSV={self.csv_path}')
            self.write('trial_start', source_stamp_s=start_odom.stamp,
                frame_id=start_odom.frame_id, child_frame_id=start_odom.child_frame_id,
                pose_x_m=start_odom.pose.x, pose_y_m=start_odom.pose.y,
                pose_yaw_rad=start_odom.pose.yaw,
                reason=(f'route_length_m={follower.profile.length:.6f};'
                        f'profile_s={profile_duration:.6f};speed={options.speed:.3f};'
                        f'command_cap={options.command_cap:.3f};'
                        f'kp_pos={POSITION_GAIN:.3f};kd_vel={VELOCITY_DAMPING:.3f}'))
            started = time.monotonic()
            deadline = started
            last_primitive = None
            reason = 'unknown'
            try:
                while rclpy.ok() and not stop['requested']:
                    now = time.monotonic()
                    elapsed = now - started
                    if elapsed > min(MAX_WALL_S, profile_duration + SETTLE_TIMEOUT_S):
                        reason = 'settle_timeout'
                        raise RuntimeError('trajectory did not settle before timeout')
                    if self.failure:
                        reason = 'invalid_odometry'
                        raise RuntimeError(self.failure)
                    allowed, gate_reason = ground_odom_gate(now, self.odom_received,
                                                            self.foreign_publishers())
                    if not allowed:
                        reason = f'feedback_gate:{gate_reason}'
                        raise RuntimeError(reason)
                    state = follower.update(elapsed, self.odom)
                    displacement = math.hypot(self.odom.pose.x - start_odom.pose.x,
                                              self.odom.pose.y - start_odom.pose.y)
                    if displacement > MAX_DISPLACEMENT_M:
                        reason = 'displacement_envelope'
                        raise RuntimeError(f'pose displacement {displacement:.3f}m exceeded {MAX_DISPLACEMENT_M:.2f}m')
                    if state.position_error > MAX_REFERENCE_ERROR_M:
                        reason = 'tracking_error_envelope'
                        raise RuntimeError(f'track error {state.position_error:.3f}m exceeded {MAX_REFERENCE_ERROR_M:.2f}m')
                    if abs(state.yaw_error) > MAX_YAW_ERROR_RAD:
                        reason = 'yaw_envelope'
                        raise RuntimeError(f'yaw error {state.yaw_error:.3f}rad exceeded {MAX_YAW_ERROR_RAD:.2f}rad')
                    sample = follower.profile.sample(min(elapsed, profile_duration))
                    primitive = primitives[sample.primitive_index]
                    segment = primitive.meta.get('field_trial_segment',
                        ('north_out', 'left_arc_out', 'west_to_cell_center',
                         'east_back_to_arc', 'right_arc_back',
                         'south_back_to_start', 'settle')[sample.primitive_index])
                    if sample.primitive_index != last_primitive:
                        self.write('segment_enter', segment=segment,
                            primitive_index=sample.primitive_index,
                            source_stamp_s=self.odom.stamp,
                            pose_x_m=self.odom.pose.x, pose_y_m=self.odom.pose.y,
                            pose_yaw_rad=self.odom.pose.yaw)
                        last_primitive = sample.primitive_index
                    reference = state.reference
                    command = state.command
                    vx, vy, wz = limit_command(command.vx, command.vy,
                                               command.wz, options.command_cap)
                    self.write('control_sample', segment=segment,
                        primitive_index=sample.primitive_index,
                        source_stamp_s=self.odom.stamp,
                        frame_id=self.odom.frame_id,
                        child_frame_id=self.odom.child_frame_id,
                        pose_x_m=self.odom.pose.x, pose_y_m=self.odom.pose.y,
                        pose_yaw_rad=self.odom.pose.yaw,
                        world_vx_mps=self.odom.vx_world,
                        world_vy_mps=self.odom.vy_world,
                        odom_wz_radps=self.odom.wz,
                        ref_x_m=reference.x, ref_y_m=reference.y,
                        ref_yaw_rad=reference.yaw_ref,
                        ref_vx_mps=reference.vx_world,
                        ref_vy_mps=reference.vy_world,
                        position_error_m=state.position_error,
                        yaw_error_rad=state.yaw_error,
                        cmd_vx_mps=vx, cmd_vy_mps=vy, cmd_wz_radps=wz,
                        displacement_from_start_m=displacement,
                        reason=f'progress={sample.progress_s:.4f};speed={sample.speed:.4f};settled={state.complete}')
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
                    self.publisher.publish(msg)
                    if state.complete:
                        reason = 'settled'
                        return
                    rclpy.spin_once(self, timeout_sec=0.0)
                    deadline = next_control_deadline(deadline, time.monotonic())
                    remaining = deadline - time.monotonic()
                    if remaining > 0 and stop_requested_wait(remaining):
                        break
                reason = 'operator_interrupt' if stop['requested'] else 'ros_shutdown'
            except Exception as exc:
                if reason == 'unknown':
                    reason = f'error:{type(exc).__name__}:{exc}'
                raise
            finally:
                self.zero()
                self.write('trial_stop', reason=reason,
                    source_stamp_s=self.odom.stamp if self.odom else '',
                    pose_x_m=self.odom.pose.x if self.odom else '',
                    pose_y_m=self.odom.pose.y if self.odom else '',
                    pose_yaw_rad=self.odom.pose.yaw if self.odom else '')
            if reason != 'operator_interrupt':
                raise RuntimeError(f'trial ended: {reason}')

    node = None
    try:
        rclpy.init()
        # Install after rclpy.init so ROS cannot replace our Ctrl-C handler.
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, on_signal)
        node = TrialNode()
        node.execute()
    finally:
        if node is not None:
            try:
                node.zero()
            except Exception:
                pass
            node.file.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


def stop_requested_wait(duration):
    """Wait in short slices so Ctrl-C promptly reaches the zero window."""
    end = time.monotonic() + duration
    while time.monotonic() < end:
        time.sleep(min(0.01, end - time.monotonic()))
    return False


if __name__ == '__main__':
    main()
