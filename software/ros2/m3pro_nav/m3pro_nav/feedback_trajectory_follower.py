"""ROS-independent execution of a compiled fixed-geometry trajectory.

Execution phases (GPT 定稿: WAIT/HOLD 不是 trajectory, 是 follower 的明确状态):
  TRACKING  正常采样 SpeedProfile + TrajectoryReference, 位置环闭环
  HOLDING   STOP-only 链或速度计划已耗尽: 平移前馈 = 0, 保持当前参考位形
            与 yaw_ref, 等待实测收玫 (STOP 不属于弧长参数化轨迹)
  FINISHED  计划结束且实测位姿/速度/角速度在容差内稳定 settle_time
"""

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import math

from .frame_transform import RigidFrameTransform
from .motion_planner import validate_geometry
from .motion_primitive import GEOMETRY_EPS
from .odometry_adapter import OdometryState
from .pose import Pose2D, Twist2D, norm_angle
from .position_controller import PositionController
from .speed_profile import SpeedProfile
from .trajectory_reference import ReferenceState, TrajectoryReference


class FollowerPhase(Enum):
    TRACKING = 'TRACKING'
    HOLDING = 'HOLDING'
    FINISHED = 'FINISHED'


@dataclass(frozen=True)
class FollowerState:
    elapsed: float
    reference: object
    command: Twist2D
    position_error_x: float
    position_error_y: float
    position_error: float
    yaw_error: float
    schedule_complete: bool
    complete: bool
    phase: FollowerPhase


class FeedbackTrajectoryFollower:
    """Follow planner-frame primitives using an odometry-frame pose sample.

    The anchors locate the same physical start in planner and odom coordinates.
    Completion requires the STOP schedule to expire and measured pose, velocity,
    and yaw rate to remain settled for ``settle_time``.

    Source-stamp freshness/monotonicity and command bounds are owned by the
    integrating ROS gate; this class checks feedback values and elapsed time.
    """

    def __init__(self, primitives, *, planner_start: Pose2D, odom_start: Pose2D,
                 a_acc, a_dec, controller: PositionController, start_speed=0.0,
                 position_tolerance, yaw_tolerance, velocity_tolerance,
                 yaw_rate_tolerance, settle_time, expected_odom_frame=None,
                 expected_base_frame=None):
        self.primitives = tuple(deepcopy(tuple(primitives)))
        if not self.primitives:
            raise ValueError('primitives must be nonempty')
        validate_geometry(self.primitives)
        if self.primitives[-1].kind != 'STOP':
            raise ValueError('trajectory must end with STOP')
        first = self.primitives[0]
        if first.kind in ('STRAIGHT', 'REVERSE'):
            geometric_start = first.p0
        elif first.kind == 'ARC':
            radius = first.meta['r']
            geometric_start = (first.p0[0] + radius * math.cos(first.yaw0),
                               first.p0[1] + radius * math.sin(first.yaw0))
        else:
            geometric_start = (first.start_pose.x, first.start_pose.y)
        if math.hypot(planner_start.x - geometric_start[0],
                      planner_start.y - geometric_start[1]) > GEOMETRY_EPS:
            raise ValueError('planner_start position does not match compiled geometry start')
        self.transform = RigidFrameTransform(planner_start, odom_start)
        # The explicit anchor carries the physical chassis heading. Compiled
        # primitive start_pose yaw fields may be geometry placeholders.
        yaw_ref = planner_start.yaw
        self.profile = SpeedProfile(self.primitives, start_speed=start_speed,
                                    a_acc=a_acc, a_dec=a_dec)
        if self.profile.end_speed != 0.0:
            raise ValueError('trajectory must end at zero speed')
        self._stop_only = all(p.kind == 'STOP' for p in self.primitives)
        self.reference = (None if self._stop_only else
                          TrajectoryReference(self.primitives, yaw_ref=yaw_ref))
        self._stationary_reference = self.transform.transform_reference(
            ReferenceState(planner_start.x, planner_start.y, yaw_ref,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        if not isinstance(controller, PositionController):
            raise ValueError('an explicit PositionController is required')
        self.controller = controller
        bounds = (position_tolerance, yaw_tolerance, velocity_tolerance,
                  yaw_rate_tolerance, settle_time)
        if not all(math.isfinite(v) and v >= 0 for v in bounds):
            raise ValueError('settling tolerances and time must be finite and nonnegative')
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.velocity_tolerance = velocity_tolerance
        self.yaw_rate_tolerance = yaw_rate_tolerance
        self.settle_time = settle_time
        self.expected_odom_frame = expected_odom_frame
        self.expected_base_frame = expected_base_frame
        self._last_elapsed = None
        self._settled_since = None
        self._complete = False

    def update(self, elapsed, odometry: OdometryState) -> FollowerState:
        """Sample and control at monotonic elapsed seconds from trajectory start."""
        if not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('elapsed time must be finite and nonnegative')
        elapsed = float(elapsed)
        if self._last_elapsed is not None and elapsed < self._last_elapsed:
            raise ValueError('elapsed time must be monotonic')
        self._validate_odometry(odometry)
        self._last_elapsed = elapsed

        speed = self.profile.sample(elapsed)
        if self._stop_only:
            ref = self._stationary_reference
        else:
            local_ref = self.reference.sample(speed.progress_s, speed.speed,
                                              speed.acceleration)
            ref = self.transform.transform_reference(local_ref)
        pose = odometry.pose
        ex, ey = ref.x - pose.x, ref.y - pose.y
        yaw_error = norm_angle(ref.yaw_ref - pose.yaw)
        schedule_complete = elapsed >= self.profile.duration
        settled = (schedule_complete and math.hypot(ex, ey) <= self.position_tolerance
                   and abs(yaw_error) <= self.yaw_tolerance
                   and math.hypot(odometry.vx_world, odometry.vy_world) <= self.velocity_tolerance
                   and abs(odometry.wz) <= self.yaw_rate_tolerance)
        if settled:
            if self._settled_since is None:
                self._settled_since = elapsed
            if elapsed - self._settled_since >= self.settle_time:
                self._complete = True
        else:
            self._settled_since = None
        command = (Twist2D() if self._complete or self._stop_only else self.controller.update(
            ref, pose, measured_velocity_world=(odometry.vx_world,
                                                odometry.vy_world),
            yaw_rate=odometry.wz))
        if self._complete:
            phase = FollowerPhase.FINISHED
        elif self._stop_only or schedule_complete:
            # HOLDING: 平移前馈 0 (STOP-only 直接零指令; 计划耗尽则位置环
            # 保持终端参考位形), yaw_ref 不变, 等实测 settle
            phase = FollowerPhase.HOLDING
        else:
            phase = FollowerPhase.TRACKING
        return FollowerState(elapsed, ref, command, ex, ey, math.hypot(ex, ey),
                             yaw_error, schedule_complete, self._complete, phase)

    def _validate_odometry(self, sample):
        if sample is None or not isinstance(sample, OdometryState):
            raise ValueError('valid odometry feedback is required')
        if (not sample.frame_id or not sample.child_frame_id or
                (self.expected_odom_frame is not None and
                 sample.frame_id != self.expected_odom_frame) or
                (self.expected_base_frame is not None and
                 sample.child_frame_id != self.expected_base_frame)):
            raise ValueError('odometry frames are missing or unexpected')
        values = (sample.pose.x, sample.pose.y, sample.pose.yaw,
                  sample.vx_world, sample.vy_world, sample.wz, sample.stamp)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('odometry feedback must be finite')
