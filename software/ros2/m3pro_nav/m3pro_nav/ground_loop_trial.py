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
from .odometry_adapter import (OdometryMonitor, OdometryState,
                               odometry_from_msg)
from .pose import Pose2D, norm_angle
from .position_controller import PositionController
from .relative_yaw import RelativeYawEstimator
from .speed_profile import SpeedProfile
from .trajectory_reference import TrajectoryReference

from .control_probe import (CONTROL_PERIOD_S, MAX_SENSOR_AGE_S,
    ground_odom_gate, limit_command, next_control_deadline, send_zero_window)

CELL_M = 0.4
SPEED_MPS = 0.50
MAX_REFERENCE_SPEED_MPS = 0.50
# Trial hypothesis: reduce translational direction change rate on the 0.2m arc.
# The chassis heading remains fixed; this is not a yaw-rate limit.
ARC_SPEED_MPS = 0.25
ACCEL_MPS2 = 1.00
DECEL_MPS2 = 0.60
# The previous 0.60 command slew held a negative command for ~0.45 s after
# feedback requested positive braking on the 2026-10-01 dorm run. This is a
# command limit, not a claim about achieved chassis deceleration.
COMMAND_DECEL_MPS2 = 3.00
POSITION_GAIN = 1.4
VELOCITY_DAMPING = 0.20
YAW_POSITION_GAIN = 2.0
YAW_RATE_DAMPING = 0.0
MAX_DISPLACEMENT_M = 0.65
MAX_CORRIDOR_ERROR_M = 0.08
MAX_YAW_ERROR_RAD = 0.25
MAX_COMMAND_MPS = 0.50
MAX_COMMAND_CAP_MPS = 0.50
MAX_WALL_S = 18.0
SETTLE_TIMEOUT_S = 5.0

CSV_FIELDS = (
    'monotonic_s', 'record_type', 'segment', 'primitive_index',
    'phase', 'repeated_control_ticks',
    'source_stamp_s', 'receipt_monotonic_s', 'frame_id', 'child_frame_id',
    'odom_body_vx_mps', 'odom_body_vy_mps',
    'pose_x_m', 'pose_y_m', 'pose_yaw_rad', 'world_vx_mps',
    'world_vy_mps', 'odom_wz_radps', 'ref_x_m', 'ref_y_m',
    'measured_progress_m', 'reference_progress_m',
    'ref_yaw_rad', 'ref_vx_mps', 'ref_vy_mps', 'position_error_m',
    'corridor_error_m', 'yaw_error_rad', 'cmd_vx_mps', 'cmd_vy_mps', 'cmd_wz_radps',
    'displacement_from_start_m', 'imu_source_stamp_s', 'imu_frame_id',
    'imu_acc_z_mps2', 'imu_wz_radps', 'imu_bias_corrected_wz_radps',
    'imu_relative_yaw_rad', 'imu_bias_radps', 'imu_receipt_age_s',
    'yaw_source', 'control_yaw_rad', 'control_yaw_rate_radps', 'reason')


def yaw_feedback_state(odometry, yaw_source='odom', imu_estimator=None):
    """Keep odom x/y and world velocity while selecting the yaw feedback."""
    if not isinstance(odometry, OdometryState):
        raise ValueError('valid odometry is required for yaw feedback')
    if yaw_source == 'odom':
        return odometry
    if yaw_source != 'imu':
        raise ValueError("yaw_source must be 'odom' or 'imu'")
    if (imu_estimator is None or not imu_estimator.valid
            or imu_estimator.yaw_rad is None
            or imu_estimator.yaw_rate_radps is None):
        raise RuntimeError('IMU relative yaw is invalid; refusing odometry fallback')
    # odometry_adapter rotated child-frame twist into world using its yaw.
    # Rotate that vector by the odom-to-IMU yaw delta to preserve world velocity
    # when the controller deliberately uses the gyro-integrated heading.
    delta = math.atan2(math.sin(imu_estimator.yaw_rad - odometry.pose.yaw),
                       math.cos(imu_estimator.yaw_rad - odometry.pose.yaw))
    c, s = math.cos(delta), math.sin(delta)
    return OdometryState(
        pose=Pose2D(odometry.pose.x, odometry.pose.y, imu_estimator.yaw_rad),
        vx_world=c * odometry.vx_world - s * odometry.vy_world,
        vy_world=s * odometry.vx_world + c * odometry.vy_world,
        wz=imu_estimator.yaw_rate_radps, stamp=odometry.stamp,
        frame_id=odometry.frame_id, child_frame_id=odometry.child_frame_id)


def service_ros_callbacks(spin_once, node, *, callback_budget=16,
                          timeout_sec=0.0, should_continue=None,
                          callback_count):
    """Service callbacks serially, then drain a bounded ready batch.

    Keeping callback execution on the control thread avoids shared-state races.
    The initial spin may wait; subsequent spins are nonblocking so a ready IMU
    stream cannot hold odometry behind a long wait.
    """
    if callback_budget < 1:
        raise ValueError('callback_budget must be positive')
    if callback_count is None:
        raise ValueError('callback_count is required to stop draining when idle')
    serviced = 0
    while serviced < callback_budget:
        if should_continue is not None and not should_continue():
            break
        before = callback_count()
        spin_once(node, timeout_sec=(timeout_sec if serviced == 0 else 0.0))
        if callback_count() == before:
            break
        serviced += 1
    return serviced


def odometry_sample_log_fields(sample, receipt_monotonic_s, body_vx, body_vy):
    """Return a source-stamped, per-callback odometry CSV record."""
    return dict(source_stamp_s=sample.stamp,
        receipt_monotonic_s=receipt_monotonic_s,
        frame_id=sample.frame_id, child_frame_id=sample.child_frame_id,
        pose_x_m=sample.pose.x, pose_y_m=sample.pose.y,
        pose_yaw_rad=sample.pose.yaw,
        odom_body_vx_mps=float(body_vx),
        odom_body_vy_mps=float(body_vy),
        world_vx_mps=sample.vx_world, world_vy_mps=sample.vy_world,
        odom_wz_radps=sample.wz)


def source_stamp_is_newer(previous_stamp, source_stamp):
    """Return whether a finite odometry source stamp advances the control input."""
    if (not isinstance(source_stamp, (int, float)) or
            not math.isfinite(source_stamp)):
        return False
    return previous_stamp is None or source_stamp > previous_stamp


def select_control_command(candidate, previous, *, update_allowed,
                           stop_requested=False):
    """Prefer an immediate zero on stop, else update only on an allowed tick."""
    if stop_requested:
        return (0.0, 0.0, 0.0)
    return candidate if update_allowed else previous


def require_fresh_imu_yaw(imu_estimator, imu_received_s, now_s,
                          maximum_age_s=None):
    """Fail closed when the IMU stream or its relative-yaw state is stale."""
    if (imu_estimator is None or not imu_estimator.valid
            or imu_estimator.yaw_rad is None
            or imu_estimator.yaw_rate_radps is None):
        raise RuntimeError('IMU relative yaw is invalid')
    if maximum_age_s is None:
        maximum_age_s = min(MAX_SENSOR_AGE_S,
                            imu_estimator.maximum_sample_gap_s)
    if (imu_received_s is None or not math.isfinite(imu_received_s)
            or not math.isfinite(now_s) or now_s < imu_received_s
            or now_s - imu_received_s > maximum_age_s):
        raise RuntimeError('IMU feedback is missing or stale')
    return True


def control_yaw_log_fields(yaw_source, feedback, imu_estimator):
    return dict(yaw_source=yaw_source,
        control_yaw_rad=feedback.pose.yaw,
        control_yaw_rate_radps=feedback.wz,
        imu_bias_radps=(imu_estimator.bias_radps
            if imu_estimator is not None else ''))


def control_imu_log_fields(imu, imu_estimator, now):
    if imu is None:
        return {}
    stamp, frame, acc_z, wz, received = imu
    return dict(imu_source_stamp_s=stamp, imu_frame_id=frame,
        imu_acc_z_mps2=acc_z, imu_wz_radps=wz,
        imu_bias_corrected_wz_radps=(imu_estimator.yaw_rate_radps
            if imu_estimator is not None else ''),
        imu_relative_yaw_rad=(imu_estimator.yaw_rad
            if imu_estimator is not None else ''),
        imu_receipt_age_s=max(0.0, now - received))


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

    # Straight speed is configurable; arcs are clamped to ARC_SPEED_MPS
    # (麦轮平移走弧: 0.45 m/s@r=0.2 m 速度矢量方向 129deg/s 扫掠, 底盘跟不住
    #  切角出廊; 1001 实测滞后 55.7deg, 走廊偏差 7cm)。
    # Keep the two direction reversals stopped at the intermediate cell center.
    for primitive in primitives:
        if primitive.kind != 'STOP':
            primitive.v_max = min(speed, ARC_SPEED_MPS) if primitive.kind == 'ARC' else speed
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


def split_route_at_west_center(primitives):
    """Split the fixed loop at its planned zero-speed west-center seam."""
    segments = [p.meta.get('field_trial_segment') for p in primitives]
    try:
        stop_index = segments.index('west_to_cell_center')
        return_index = segments.index('east_back_to_arc')
    except ValueError as exc:
        raise ValueError('route is missing the west-center phase seam') from exc
    if return_index != stop_index + 1:
        raise ValueError('west-center stop and return segments must be adjacent')
    if primitives[stop_index].v_end != 0.0:
        raise ValueError('west-center seam must have a zero terminal speed')
    midpoint_pose = primitives[return_index].start_pose
    midpoint_stop = MotionPrimitive('STOP', midpoint_pose, duration=0.0,
        meta={'field_trial_terminal': True, 'field_trial_phase_stop': 'midpoint'})
    outbound = tuple(primitives[:return_index]) + (midpoint_stop,)
    returning = tuple(primitives[return_index:])
    validate_geometry(outbound)
    validate_geometry(returning)
    return outbound, returning, return_index, midpoint_pose


def validate_trial_limits(speed, command_cap):
    """Validate the deliberately narrow gradual-speed comparison bounds."""
    if not math.isfinite(speed) or not 0.05 <= speed <= MAX_REFERENCE_SPEED_MPS:
        raise ValueError(f'speed must be in [0.05, {MAX_REFERENCE_SPEED_MPS:.2f}] m/s')
    if not math.isfinite(command_cap) or speed > command_cap \
            or command_cap > MAX_COMMAND_CAP_MPS or command_cap <= 0:
        raise ValueError(
            f'command cap must be >= speed and <= {MAX_COMMAND_CAP_MPS:.2f} m/s')
    return speed, command_cap


def validate_controller_gains(kp_pos, kd_vel):
    """Validate bounded field-trial gains; defaults change only kp for A/B."""
    if not math.isfinite(kp_pos) or not 0.0 <= kp_pos <= 2.0:
        raise ValueError('kp_pos must be finite and in [0, 2]')
    if not math.isfinite(kd_vel) or not 0.0 <= kd_vel <= 1.0:
        raise ValueError('kd_vel must be finite and in [0, 1]')
    return kp_pos, kd_vel


def validate_yaw_gains(kp_yaw, kd_yaw):
    if not math.isfinite(kp_yaw) or not 0.0 <= kp_yaw <= 6.0:
        raise ValueError('kp_yaw must be finite and in [0, 6]')
    if not math.isfinite(kd_yaw) or not 0.0 <= kd_yaw <= 2.0:
        raise ValueError('kd_yaw must be finite and in [0, 2]')
    return kp_yaw, kd_yaw


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


def route_corridor_waypoints(primitives, transform, *, samples=240):
    """Sample the fixed geometric path and map it into the odometry frame."""
    if samples < 2:
        raise ValueError('at least two route corridor samples are required')
    reference = TrajectoryReference(primitives, yaw_ref=math.pi / 2)
    points = []
    for index in range(samples + 1):
        state = reference.sample(reference.length * index / samples)
        pose = transform.transform_pose(Pose2D(state.x, state.y, state.yaw_ref))
        points.append((pose.x, pose.y))
    return tuple(points)


def distance_to_polyline(point, waypoints):
    """Euclidean distance from a point to a sampled route polyline."""
    if len(waypoints) < 2:
        raise ValueError('route polyline requires at least two waypoints')
    px, py = point
    if not all(math.isfinite(value) for value in (px, py)):
        raise ValueError('query point must be finite')
    nearest_sq = math.inf
    for a, b in zip(waypoints, waypoints[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_sq = dx * dx + dy * dy
        fraction = 0.0 if length_sq <= 1e-18 else max(0.0, min(1.0,
            ((px - a[0]) * dx + (py - a[1]) * dy) / length_sq))
        ex, ey = px - (a[0] + fraction * dx), py - (a[1] + fraction * dy)
        nearest_sq = min(nearest_sq, ex * ex + ey * ey)
    return math.sqrt(nearest_sq)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', action='store_true', required=True,
                   help='explicitly execute the (6,2) out-and-back route')
    p.add_argument('--expected-odom-frame', required=True)
    p.add_argument('--expected-base-frame', required=True)
    p.add_argument('--csv', default='/tmp/ground_loop_trial.csv')
    p.add_argument('--odom-synchronous-control', action='store_true',
                   help='update control only on a newer /odom_raw source stamp')
    p.add_argument('--speed', type=float, default=SPEED_MPS)
    p.add_argument('--command-cap', type=float, default=MAX_COMMAND_MPS,
                   help='linear command cap in m/s (must be >= speed and <= 0.50)')
    p.add_argument('--yaw-source', choices=('odom', 'imu'), default='odom',
                   help='yaw feedback source; imu requires stationary bias calibration before motion')
    p.add_argument('--kp-yaw', type=float, default=YAW_POSITION_GAIN,
                   help='yaw-angle proportional gain (default 2.0; allowed range 0..6)')
    p.add_argument('--kd-yaw', type=float, default=YAW_RATE_DAMPING,
                   help='bias-corrected yaw-rate damping gain (default 0; allowed range 0..2)')
    p.add_argument('--kp-pos', type=float, default=POSITION_GAIN,
                   help='position gain (default 1.4; allowed range 0..2)')
    p.add_argument('--kd-vel', type=float, default=VELOCITY_DAMPING,
                   help='velocity damping gain (default 0.2; allowed range 0..1)')
    return p


def main(args=None):
    options = parser().parse_args(args)
    validate_trial_limits(options.speed, options.command_cap)
    validate_controller_gains(options.kp_pos, options.kd_vel)
    validate_yaw_gains(options.kp_yaw, options.kd_yaw)
    primitives = compile_loop_route(speed=options.speed)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
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
            self.imu = None
            self.imu_estimator = (RelativeYawEstimator()
                                 if options.yaw_source == 'imu' else None)
            self.collect_imu_bias = False
            self.imu_calibration_issue = None
            self.imu_failure = None
            self.monitor = OdometryMonitor(max_age=MAX_SENSOR_AGE_S)
            self.failure = None
            self._ros_callback_count = 0
            self.publisher = None
            self._zero_sent = False
            self._last_cmd = None
            self._last_control_source_stamp = None
            self._held_control_ticks = 0
            self._last_control_state = None
            if options.odom_synchronous_control:
                self._last_cmd = (0.0, 0.0, 0.0)
            self.create_subscription(Odometry, '/odom_raw', self.on_odom, 10)
            self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 10)

        def write(self, record_type, **values):
            row = {key: '' for key in CSV_FIELDS}
            row.update(monotonic_s=f'{time.monotonic():.6f}',
                       record_type=record_type, **values)
            self.writer.writerow(row)
            self.file.flush()

        def on_odom(self, msg):
            self._ros_callback_count += 1
            received = time.monotonic()
            try:
                sample = odometry_from_msg(msg,
                    expected_odom_frame=options.expected_odom_frame,
                    expected_base_frame=options.expected_base_frame)
                body_twist = msg.twist.twist
                self.write('odom_sample', **odometry_sample_log_fields(
                    sample, received, body_twist.linear.x,
                    body_twist.linear.y))
                self.monitor.accept(sample, now=received, received=received,
                    expected_odom_frame=options.expected_odom_frame,
                    expected_base_frame=options.expected_base_frame)
                self.odom, self.odom_received = sample, received
            except Exception as exc:
                self.failure = str(exc)
                self.write('invalid_odom_sample',
                    receipt_monotonic_s=received,
                    frame_id=getattr(getattr(msg, 'header', None),
                                     'frame_id', ''),
                    child_frame_id=getattr(msg, 'child_frame_id', ''),
                    reason=str(exc)[:240])

        def service_callbacks(self, timeout_sec=0.0):
            return service_ros_callbacks(
                rclpy.spin_once, self, timeout_sec=timeout_sec,
                should_continue=lambda: rclpy.ok() and not stop['requested'],
                callback_count=lambda: self._ros_callback_count)

        @staticmethod
        def stamp(msg):
            sec, nanosec = int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)
            if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
                raise ValueError('IMU stamp fields are out of range')
            return float(sec) + float(nanosec) / 1e9

        def on_imu(self, msg):
            """Record gyro data and update the optional diagnostic yaw estimate."""
            self._ros_callback_count += 1
            try:
                stamp = self.stamp(msg)
                acc_z = float(msg.linear_acceleration.z)
                wz = float(msg.angular_velocity.z)
                if not all(math.isfinite(value) for value in (stamp, acc_z, wz)):
                    raise ValueError('IMU contains nonfinite diagnostic values')
                if not msg.header.frame_id:
                    raise ValueError('IMU header.frame_id must be nonempty')
                received = time.monotonic()
                # Receipt freshness tracks the stream even when a calibration
                # sample is rejected and its stationary window is reset.
                self.imu = (stamp, msg.header.frame_id, acc_z, wz, received)
                if self.imu_estimator is not None:
                    if self.imu_estimator.valid:
                        self.imu_estimator.update(stamp, wz)
                    elif self.collect_imu_bias:
                        self.imu_estimator.add_stationary_sample(stamp, wz)
                yaw = (self.imu_estimator.yaw_rad
                       if self.imu_estimator is not None else None)
                corrected_rate = (self.imu_estimator.yaw_rate_radps
                                  if self.imu_estimator is not None else None)
                self.write('imu_sample', source_stamp_s=stamp,
                    imu_source_stamp_s=stamp, imu_frame_id=msg.header.frame_id,
                    imu_acc_z_mps2=acc_z, imu_wz_radps=wz,
                    imu_bias_corrected_wz_radps=(corrected_rate
                        if corrected_rate is not None else ''),
                    imu_relative_yaw_rad=yaw if yaw is not None else '',
                    imu_bias_radps=(self.imu_estimator.bias_radps
                        if self.imu_estimator is not None
                        and self.imu_estimator.bias_radps is not None else ''),
                    yaw_source=options.yaw_source, imu_receipt_age_s=0.0)
            except Exception as exc:
                if self.imu_estimator is not None:
                    if self.imu_estimator.valid:
                        self.imu_failure = str(exc)
                    elif self.collect_imu_bias:
                        self.imu_estimator.reset_bias_calibration()
                        self.imu_calibration_issue = str(exc)
                self.write('invalid_imu', imu_frame_id=msg.header.frame_id,
                           yaw_source=options.yaw_source,
                           reason=str(exc)[:160])

        def foreign_publishers(self):
            return sum(1 for endpoint in self.get_publishers_info_by_topic('/cmd_vel')
                if not (endpoint.node_name == self.get_name()
                        and endpoint.node_namespace == self.get_namespace()))

        def zero(self):
            if self.publisher is None or self._zero_sent:
                return
            # In the bundled firmware sample an all-zero Twist enters
            # Motion_Stop(STOP_BRAKE). Do not keep commanding motion after a
            # fault while waiting for a nonzero "braking ramp" to finish.
            def record_zero():
                self.write('emergency_zero', cmd_vx_mps=0.0,
                           cmd_vy_mps=0.0, cmd_wz_radps=0.0)
                # Keep receiving odometry during the zero-command window so
                # stopping distance is visible in this trial's CSV.
                if rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.0)

            send_zero_window(self.publisher, Twist,
                             on_publish=record_zero)
            self._zero_sent = True

        def execute(self):
            wait_until = time.monotonic() + 4.0
            while rclpy.ok() and not stop['requested'] and self.odom is None and time.monotonic() < wait_until:
                self.service_callbacks(timeout_sec=0.05)
            if self.odom is None:
                raise RuntimeError(f'no valid /odom_raw before timeout: {self.failure or "missing"}')
            if stop['requested']:
                return
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            discovery_until = time.monotonic() + 1.0
            while rclpy.ok() and not stop['requested'] and time.monotonic() < discovery_until:
                self.service_callbacks(timeout_sec=0.05)
            allowed, reason = ground_odom_gate(time.monotonic(), self.odom_received,
                                               self.foreign_publishers())
            if not allowed:
                raise RuntimeError(f'preflight: {reason}')
            if stop['requested']:
                return

            if self.imu_estimator is not None:
                self.collect_imu_bias = True
                calibration_deadline = time.monotonic() + 10.0
                try:
                    while (rclpy.ok() and not stop['requested']
                           and not self.imu_estimator.bias_ready
                           and time.monotonic() < calibration_deadline):
                        now = time.monotonic()
                        allowed, reason = ground_odom_gate(
                            now, self.odom_received, self.foreign_publishers())
                        if not allowed:
                            raise RuntimeError(
                                f'preflight during IMU calibration: {reason}')
                        if (self.imu is not None
                                and now - self.imu[4] >
                                self.imu_estimator.maximum_sample_gap_s):
                            raise RuntimeError('IMU stream became stale during calibration')
                        self.service_callbacks(timeout_sec=0.05)
                    if stop['requested']:
                        self.zero()
                        return
                    if not rclpy.ok():
                        raise RuntimeError('ROS shut down during IMU calibration')
                    if not self.imu_estimator.bias_ready:
                        detail = (f'; last calibration issue: {self.imu_calibration_issue}'
                                  if self.imu_calibration_issue else '')
                        raise RuntimeError(
                            'IMU stationary bias calibration timed out before '
                            'sample-count and 4.5-second source-duration checks passed'
                            + detail)
                    if (self.imu is None or time.monotonic() - self.imu[4] >
                            self.imu_estimator.maximum_sample_gap_s):
                        raise RuntimeError('IMU stream is missing or stale after calibration')
                    if (self.odom is None or self.odom_received is None
                            or time.monotonic() - self.odom_received > MAX_SENSOR_AGE_S):
                        raise RuntimeError('odometry is missing or stale after calibration')
                    start_odom = self.odom
                    imu_stamp, imu_frame, _acc_z, _raw_wz, _received = self.imu
                    self.imu_estimator.start(start_odom.pose.yaw, imu_stamp)
                    self.collect_imu_bias = False
                    self.write('imu_yaw_anchor',
                        source_stamp_s=imu_stamp, imu_source_stamp_s=imu_stamp,
                        imu_frame_id=imu_frame, yaw_source='imu',
                        pose_x_m=start_odom.pose.x, pose_y_m=start_odom.pose.y,
                        pose_yaw_rad=start_odom.pose.yaw,
                        control_yaw_rad=self.imu_estimator.yaw_rad,
                        control_yaw_rate_radps=self.imu_estimator.yaw_rate_radps,
                        imu_bias_radps=self.imu_estimator.bias_radps,
                        reason=(f'bias_std_radps={self.imu_estimator.bias_std_radps:.6f};'
                                f'calibration_duration_s='
                                f'{self.imu_estimator.calibration_duration_s:.3f}'))
                except Exception as exc:
                    self.collect_imu_bias = False
                    self.write('imu_yaw_fault', yaw_source='imu',
                               reason=str(exc)[:240])
                    self.zero()
                    raise
            else:
                start_odom = self.odom
            initial_center = (6.5 * CELL_M, 2.5 * CELL_M)
            outbound, returning, return_index, midpoint_pose = \
                split_route_at_west_center(primitives)
            controller = PositionController(kp_pos=options.kp_pos,
                kd_vel=options.kd_vel, kp_yaw=options.kp_yaw,
                kd_yaw=options.kd_yaw)

            def build_follower(route, planner_start, *, odom_start=None,
                               transform=None):
                return FeedbackTrajectoryFollower(
                    route, planner_start=planner_start, odom_start=odom_start,
                    transform=transform, a_acc=ACCEL_MPS2, a_dec=DECEL_MPS2,
                    controller=controller, start_speed=0.0,
                    position_tolerance=0.025, yaw_tolerance=0.06,
                    velocity_tolerance=0.025, yaw_rate_tolerance=0.15,
                    settle_time=0.30,
                    max_reference_lead_m=0.08,
                    max_linear_speed_mps=MAX_COMMAND_MPS,
                    max_command_accel_mps2=ACCEL_MPS2,
                    max_command_decel_mps2=COMMAND_DECEL_MPS2,
                    expected_odom_frame=options.expected_odom_frame,
                    expected_base_frame=options.expected_base_frame)

            follower = build_follower(
                outbound, Pose2D(*initial_center, math.pi / 2),
                odom_start=start_odom.pose)
            corridor = route_corridor_waypoints(
                primitives, follower.transform, samples=240)
            outbound_duration = follower.profile.duration
            returning_start = Pose2D(midpoint_pose.x, midpoint_pose.y,
                                     math.pi / 2)
            returning_profile = SpeedProfile(
                returning, start_speed=0.0, a_acc=ACCEL_MPS2,
                a_dec=DECEL_MPS2)
            profile_duration = outbound_duration + returning_profile.duration
            self.get_logger().info(
                f'out-and-back: length={nominal_route_end(primitives)[1]:.3f}m, '
                f'profile={profile_duration:.2f}s, midpoint stop enabled, '
                f'start=(6,2) N, yaw_source={options.yaw_source}, '
                f'CSV={self.csv_path}')
            self.write('trial_start', source_stamp_s=start_odom.stamp,
                frame_id=start_odom.frame_id, child_frame_id=start_odom.child_frame_id,
                pose_x_m=start_odom.pose.x, pose_y_m=start_odom.pose.y,
                pose_yaw_rad=start_odom.pose.yaw,
                yaw_source=options.yaw_source,
                control_yaw_rad=(self.imu_estimator.yaw_rad
                    if self.imu_estimator is not None else start_odom.pose.yaw),
                control_yaw_rate_radps=(self.imu_estimator.yaw_rate_radps
                    if self.imu_estimator is not None else start_odom.wz),
                imu_bias_radps=(self.imu_estimator.bias_radps
                    if self.imu_estimator is not None else ''),
                reason=(f'route_length_m={nominal_route_end(primitives)[1]:.6f};'
                        f'profile_s={profile_duration:.6f};speed={options.speed:.3f};'
                        f'a_acc={ACCEL_MPS2:.3f};a_dec={DECEL_MPS2:.3f};'
                        f'command_decel={COMMAND_DECEL_MPS2:.3f};'
                        f'midpoint_stop=true;'
                        f'yaw_source={options.yaw_source};'
                        f'command_cap={options.command_cap:.3f};'
                        f'kp_pos={options.kp_pos:.3f};kd_vel={options.kd_vel:.3f};'
                        f'kp_yaw={options.kp_yaw:.3f};kd_yaw={options.kd_yaw:.3f}'))
            started = time.monotonic()
            phase_started = started
            phase_index = 0
            phase_name = 'outbound'
            primitive_offset = 0
            phase_primitives = outbound
            phase_hold_logged = False
            deadline = started
            last_primitive = None
            last_progress_s = 0.0
            progress_stall_since = started
            reason = 'unknown'
            state = None
            try:
                while rclpy.ok() and not stop['requested']:
                    now = time.monotonic()
                    elapsed = now - phase_started
                    total_elapsed = now - started
                    if total_elapsed > MAX_WALL_S:
                        reason = 'wall_timeout'
                        raise RuntimeError('trajectory exceeded the wall-time limit')
                    if self.failure:
                        reason = 'invalid_odometry'
                        raise RuntimeError(self.failure)
                    if self.imu_failure:
                        reason = 'invalid_imu_yaw'
                        raise RuntimeError(self.imu_failure)
                    allowed, gate_reason = ground_odom_gate(now, self.odom_received,
                                                            self.foreign_publishers())
                    if not allowed:
                        reason = f'feedback_gate:{gate_reason}'
                        raise RuntimeError(reason)
                    if self.imu_estimator is not None:
                        try:
                            require_fresh_imu_yaw(
                                self.imu_estimator,
                                self.imu[4] if self.imu is not None else None,
                                now)
                            control_feedback = yaw_feedback_state(
                                self.odom, 'imu', self.imu_estimator)
                        except Exception as exc:
                            reason = 'invalid_imu_yaw'
                            raise RuntimeError(str(exc)) from exc
                    else:
                        control_feedback = yaw_feedback_state(self.odom, 'odom')
                    update_allowed = (
                        not options.odom_synchronous_control or
                        source_stamp_is_newer(self._last_control_source_stamp,
                                              control_feedback.stamp))
                    if update_allowed:
                        if options.odom_synchronous_control:
                            self._last_control_source_stamp = control_feedback.stamp
                        state = follower.update(elapsed, control_feedback)
                        if options.odom_synchronous_control:
                            self._last_control_state = state
                    else:
                        state = self._last_control_state
                        if state is None:
                            raise RuntimeError(
                                'no initial control state for repeated odometry stamp')
                    yaw_log_values = control_yaw_log_fields(
                        options.yaw_source, control_feedback,
                        self.imu_estimator)
                    if (update_allowed and
                            state.measured_progress_s >= last_progress_s + 0.01):
                        last_progress_s = state.measured_progress_s
                        progress_stall_since = now
                    elif now - progress_stall_since > SETTLE_TIMEOUT_S:
                        reason = ('midpoint_progress_stall' if phase_index == 0
                                  else 'return_progress_stall')
                        raise RuntimeError(
                            f'{phase_name} measured route progress stalled for '
                            f'{SETTLE_TIMEOUT_S:.1f}s')
                    displacement = math.hypot(self.odom.pose.x - start_odom.pose.x,
                                              self.odom.pose.y - start_odom.pose.y)
                    if displacement > MAX_DISPLACEMENT_M:
                        reason = 'displacement_envelope'
                        raise RuntimeError(f'pose displacement {displacement:.3f}m exceeded {MAX_DISPLACEMENT_M:.2f}m')
                    corridor_error = distance_to_polyline(
                        (self.odom.pose.x, self.odom.pose.y), corridor)
                    if corridor_error > MAX_CORRIDOR_ERROR_M:
                        reason = 'geometric_corridor_envelope'
                        raise RuntimeError(
                            f'geometric route deviation {corridor_error:.3f}m '
                            f'exceeded {MAX_CORRIDOR_ERROR_M:.2f}m')
                    yaw_error = norm_angle(
                        state.reference.yaw_ref - control_feedback.pose.yaw)
                    if abs(yaw_error) > MAX_YAW_ERROR_RAD:
                        reason = 'yaw_envelope'
                        raise RuntimeError(f'yaw error {yaw_error:.3f}rad exceeded {MAX_YAW_ERROR_RAD:.2f}rad')
                    if not update_allowed:
                        held = select_control_command(
                            None, self._last_cmd, update_allowed=False,
                            stop_requested=stop['requested'])
                        if stop['requested']:
                            reason = 'operator_interrupt'
                            self.zero()
                            break
                        self._held_control_ticks += 1
                        self.write('control_hold',
                            phase=phase_name,
                            source_stamp_s=control_feedback.stamp,
                            repeated_control_ticks=self._held_control_ticks,
                            cmd_vx_mps=held[0], cmd_vy_mps=held[1],
                            cmd_wz_radps=held[2],
                            reason='waiting for a strictly newer odometry source stamp')
                        msg = Twist()
                        msg.linear.x, msg.linear.y, msg.angular.z = held
                        self.publisher.publish(msg)
                        self.service_callbacks()
                        deadline = next_control_deadline(deadline,
                                                         time.monotonic())
                        remaining = deadline - time.monotonic()
                        if remaining > 0 and stop_requested_wait(remaining):
                            break
                        continue
                    sample = state.speed_sample
                    primitive = phase_primitives[sample.primitive_index]
                    segment = primitive.meta.get('field_trial_segment',
                        ('north_out', 'left_arc_out', 'west_to_cell_center',
                         'east_back_to_arc', 'right_arc_back',
                         'south_back_to_start', 'settle')[sample.primitive_index])
                    global_primitive_index = primitive_offset + sample.primitive_index
                    if primitive.meta.get('field_trial_phase_stop') == 'midpoint':
                        segment = 'midpoint_stop'
                    elif primitive.kind == 'STOP':
                        segment = 'settle'
                    if sample.primitive_index != last_primitive:
                        self.write('segment_enter', segment=segment,
                            primitive_index=global_primitive_index,
                            phase=phase_name,
                            source_stamp_s=self.odom.stamp,
                            pose_x_m=self.odom.pose.x, pose_y_m=self.odom.pose.y,
                            pose_yaw_rad=self.odom.pose.yaw,
                            **yaw_log_values)
                        last_primitive = sample.primitive_index
                    if state.phase.value == 'HOLDING' and not phase_hold_logged:
                        self.write('phase_hold_start', segment=segment,
                            primitive_index=global_primitive_index,
                            phase=phase_name, source_stamp_s=self.odom.stamp,
                            pose_x_m=self.odom.pose.x, pose_y_m=self.odom.pose.y,
                            pose_yaw_rad=self.odom.pose.yaw,
                            **yaw_log_values,
                            position_error_m=state.position_error,
                            yaw_error_rad=state.yaw_error,
                            world_vx_mps=self.odom.vx_world,
                            world_vy_mps=self.odom.vy_world,
                            odom_wz_radps=self.odom.wz,
                            corridor_error_m=corridor_error,
                            reason='profile complete; waiting for measured settle')
                        phase_hold_logged = True
                    reference = state.reference
                    command = state.command
                    imu_values = control_imu_log_fields(
                        self.imu, self.imu_estimator, now)
                    vx, vy, wz = limit_command(command.vx, command.vy,
                                               command.wz, options.command_cap)
                    if options.odom_synchronous_control:
                        selected = select_control_command(
                            (vx, vy, wz), self._last_cmd,
                            update_allowed=update_allowed,
                            stop_requested=stop['requested'])
                        vx, vy, wz = selected
                        self._last_cmd = selected
                        self._held_control_ticks = 0
                    if stop['requested'] and options.odom_synchronous_control:
                        reason = 'operator_interrupt'
                        self.zero()
                        break
                    self.write('control_sample', segment=segment,
                        primitive_index=global_primitive_index,
                        phase=phase_name,
                        source_stamp_s=self.odom.stamp,
                        frame_id=self.odom.frame_id,
                        child_frame_id=self.odom.child_frame_id,
                        pose_x_m=self.odom.pose.x, pose_y_m=self.odom.pose.y,
                        pose_yaw_rad=self.odom.pose.yaw,
                        **yaw_log_values,
                        world_vx_mps=self.odom.vx_world,
                        world_vy_mps=self.odom.vy_world,
                        odom_wz_radps=self.odom.wz,
                        ref_x_m=reference.x, ref_y_m=reference.y,
                        measured_progress_m=state.measured_progress_s,
                        reference_progress_m=reference.progress_s,
                        ref_yaw_rad=reference.yaw_ref,
                        ref_vx_mps=reference.vx_world,
                        ref_vy_mps=reference.vy_world,
                        position_error_m=state.position_error,
                        corridor_error_m=corridor_error,
                        yaw_error_rad=state.yaw_error,
                        cmd_vx_mps=vx, cmd_vy_mps=vy, cmd_wz_radps=wz,
                        displacement_from_start_m=displacement,
                        reason=f'progress={sample.progress_s:.4f};speed={sample.speed:.4f};settled={state.complete}',
                        **imu_values)
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
                    self._last_cmd = (vx, vy, wz)
                    self.publisher.publish(msg)
                    if state.complete:
                        self.write('phase_hold_complete', segment=segment,
                            primitive_index=global_primitive_index,
                            phase=phase_name, source_stamp_s=self.odom.stamp,
                            pose_x_m=self.odom.pose.x, pose_y_m=self.odom.pose.y,
                            pose_yaw_rad=self.odom.pose.yaw,
                            **yaw_log_values,
                            position_error_m=state.position_error,
                            yaw_error_rad=state.yaw_error,
                            world_vx_mps=self.odom.vx_world,
                            world_vy_mps=self.odom.vy_world,
                            odom_wz_radps=self.odom.wz,
                            corridor_error_m=corridor_error,
                            reason='position, speed, yaw and dwell tolerances met')
                        if phase_index == 0:
                            phase_index = 1
                            phase_name = 'return'
                            primitive_offset = return_index
                            phase_primitives = returning
                            follower = build_follower(
                                returning, returning_start,
                                transform=follower.transform)
                            phase_started = time.monotonic()
                            last_progress_s = 0.0
                            progress_stall_since = phase_started
                            phase_hold_logged = False
                            last_primitive = None
                            self.service_callbacks()
                            deadline = next_control_deadline(
                                deadline, time.monotonic())
                            continue
                        reason = 'settled'
                        return
                    self.service_callbacks()
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
                    pose_yaw_rad=self.odom.pose.yaw if self.odom else '',
                    yaw_source=options.yaw_source,
                    control_yaw_rad=(self.imu_estimator.yaw_rad
                        if self.imu_estimator is not None
                        and self.imu_estimator.yaw_rad is not None
                        else self.odom.pose.yaw if self.odom else ''),
                    control_yaw_rate_radps=(self.imu_estimator.yaw_rate_radps
                        if self.imu_estimator is not None
                        and self.imu_estimator.yaw_rate_radps is not None
                        else self.odom.wz if self.odom else ''),
                    imu_bias_radps=(self.imu_estimator.bias_radps
                        if self.imu_estimator is not None else ''))
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
