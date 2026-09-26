"""Pure-Python adapter for ROS nav_msgs/Odometry values.

ROS packages are intentionally not imported here so frame and kinematics
checks can be exercised on development hosts without ROS installed.
"""

from dataclasses import dataclass
import math

from .pose import Pose2D


@dataclass(frozen=True)
class OdometryState:
    pose: Pose2D
    vx_world: float
    vy_world: float
    wz: float
    stamp: float
    frame_id: str
    child_frame_id: str


def odometry_from_msg(msg, *, expected_odom_frame: str,
                      expected_base_frame: str) -> OdometryState:
    """Validate an Odometry message and convert child-frame twist to world."""
    if not expected_odom_frame or not expected_base_frame:
        raise ValueError('expected odom and base frames must be explicit')
    frame = msg.header.frame_id
    child = msg.child_frame_id
    if not frame or not child:
        raise ValueError('Odometry header.frame_id and child_frame_id must be nonempty')
    if frame != expected_odom_frame or child != expected_base_frame:
        raise ValueError(f'unexpected odometry frames: {frame!r}/{child!r}')
    sec, nanosec = int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('Odometry stamp fields are out of range')
    stamp = float(sec) + float(nanosec) / 1e9
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.twist.twist
    values = (stamp, p.x, p.y, q.x, q.y, q.z, q.w,
              v.linear.x, v.linear.y, v.angular.z)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('Odometry contains nonfinite values')
    qnorm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if qnorm < 1e-9 or abs(qnorm - 1.0) > 1e-3:
        raise ValueError(f'invalid quaternion norm {qnorm!r}')
    # Normalize small numerical drift before extracting yaw.
    x, y, z, w = q.x/qnorm, q.y/qnorm, q.z/qnorm, q.w/qnorm
    yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    c, s = math.cos(yaw), math.sin(yaw)
    return OdometryState(Pose2D(float(p.x), float(p.y), yaw),
                         c*float(v.linear.x) - s*float(v.linear.y),
                         s*float(v.linear.x) + c*float(v.linear.y),
                         float(v.angular.z), stamp, frame, child)


class OdometryMonitor:
    """Pure freshness and timestamp monotonicity gate for adapted samples."""
    def __init__(self, max_age=0.5):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('max_age must be positive and finite')
        self.max_age = max_age
        self.last_stamp = None

    def accept(self, sample: OdometryState, *, now: float, received: float,
               expected_odom_frame: str, expected_base_frame: str):
        if sample.frame_id != expected_odom_frame or sample.child_frame_id != expected_base_frame:
            raise ValueError('odometry frame mismatch')
        if not math.isfinite(now) or not math.isfinite(received) or now < received:
            raise ValueError('invalid odometry receipt time')
        if now - received > self.max_age:
            raise ValueError('odometry receipt is stale')
        if self.last_stamp is not None and sample.stamp <= self.last_stamp:
            raise ValueError('odometry stamp did not advance')
        self.last_stamp = sample.stamp
        return sample
