"""ROS-independent execution of a compiled fixed-geometry trajectory.

Execution phases (GPT 定稿: WAIT/HOLD 不是 trajectory, 是 follower 的明确状态):
  TRACKING  正常采样 SpeedProfile + TrajectoryReference, 位置环闭环
  HOLDING   STOP-only 链或速度计划已耗尽: 平移前馈 = 0, 位置环保持参考位形
            与 yaw_ref (有漂移会被拉回), 等待实测收玫或后续 suffix
  FINISHED  仅终端 STOP 链 (无 meta['wait']): 计划结束且实测位姿/速度/角速度
            在容差内稳定 settle_time

STOP 语义 (meta 区分, GPT 审查定稿):
  WAIT STOP (meta['wait'])    未知边界等待 —— 永久 HOLDING, 只有 suffix /
                              cancel / fault 才离开, 绝不因 settle 变 FINISHED
  终端 STOP (无 meta['wait']) 任务终点 —— settle 后 FINISHED
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


def is_wait_stop(prim):
    """WAIT STOP: ActionHorizon 在未知边界排的等待 (meta['wait'])."""
    return prim.kind == 'STOP' and prim.meta.get('wait') is not None


def primitive_start_tangent(prim):
    """起点切向 (planner frame); STOP 无切向 → None."""
    if prim.kind == 'STRAIGHT':
        dx = prim.p1[0] - prim.p0[0]
        dy = prim.p1[1] - prim.p0[1]
        length = math.hypot(dx, dy)
        return dx / length, dy / length
    if prim.kind == 'ARC':
        sign = math.copysign(1.0, prim.yaw1)
        return -sign * math.sin(prim.yaw0), sign * math.cos(prim.yaw0)
    return None


class FeedbackTrajectoryFollower:
    """Follow planner-frame primitives using an odometry-frame pose sample.

    The anchors locate the same physical start in planner and odom coordinates.
    Completion (FINISHED) requires a terminal (non-wait) STOP chain, the STOP
    schedule to expire, and measured pose, velocity, and yaw rate to remain
    settled for ``settle_time``. A wait-tail chain never self-finishes.

    Source-stamp freshness/monotonicity and command bounds are owned by the
    integrating ROS gate; this class checks feedback values and elapsed time.
    """

    def __init__(self, primitives, *, planner_start: Pose2D, odom_start: Pose2D,
                 a_acc, a_dec, controller: PositionController, start_speed=0.0,
                 position_tolerance, yaw_tolerance, velocity_tolerance,
                 yaw_rate_tolerance, settle_time, expected_odom_frame=None,
                 expected_base_frame=None, transform=None, elapsed_offset=0.0):
        self.primitives = tuple(deepcopy(tuple(primitives)))
        if not self.primitives:
            raise ValueError('primitives must be nonempty')
        validate_geometry(self.primitives)
        if self.primitives[-1].kind != 'STOP':
            raise ValueError('trajectory must end with STOP')
        first = self.primitives[0]
        if first.kind == 'STRAIGHT':
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
        # The explicit anchor carries the physical chassis heading. Compiled
        # primitive start_pose yaw fields may be geometry placeholders.
        self.yaw_ref = planner_start.yaw
        if transform is not None:
            # Splice 续接: 沿用原 follower 的同一物理锚 (帧变换不得重锚).
            if not isinstance(transform, RigidFrameTransform):
                raise ValueError('transform must be a RigidFrameTransform')
            self.transform = transform
        else:
            if odom_start is None:
                raise ValueError('odom_start anchor is required without an '
                                 'explicit transform')
            self.transform = RigidFrameTransform(planner_start, odom_start)
        # WAIT 尾链: 未知边界等待, 永不自结束
        self._wait_tail = is_wait_stop(self.primitives[-1])
        self._a_acc = a_acc
        self._a_dec = a_dec
        self.profile = SpeedProfile(self.primitives, start_speed=start_speed,
                                    a_acc=a_acc, a_dec=a_dec)
        if self.profile.end_speed != 0.0:
            raise ValueError('trajectory must end at zero speed')
        self._stop_only = all(p.kind == 'STOP' for p in self.primitives)
        self.reference = (None if self._stop_only else
                          TrajectoryReference(self.primitives, yaw_ref=self.yaw_ref))
        self._stationary_reference = self.transform.transform_reference(
            ReferenceState(planner_start.x, planner_start.y, self.yaw_ref,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        if not isinstance(controller, PositionController):
            raise ValueError('an explicit PositionController is required')
        self.controller = controller
        bounds = (position_tolerance, yaw_tolerance, velocity_tolerance,
                  yaw_rate_tolerance, settle_time, elapsed_offset)
        if not all(math.isfinite(v) and v >= 0 for v in bounds):
            raise ValueError('settling tolerances, time and elapsed offset '
                             'must be finite and nonnegative')
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.velocity_tolerance = velocity_tolerance
        self.yaw_rate_tolerance = yaw_rate_tolerance
        self.settle_time = settle_time
        self.expected_odom_frame = expected_odom_frame
        self.expected_base_frame = expected_base_frame
        self._elapsed_offset = elapsed_offset
        self._last_elapsed = elapsed_offset
        self._last_sample = None
        self._settled_since = None
        self._complete = False

    # ---- 保守 safe-extension (GPT P1-7) ----

    def try_splice(self, suffix):
        """删"尚未影响活动 primitive 速度计划"的终端 WAIT STOP, 无缝续 suffix.

        仅在全部条件满足时拼接 (否则返回 None, 调用方保守排队/停车续接):
          - 已有至少一次 update (有当前采样);
          - 活动 primitive 是 STRAIGHT 且其后恰好只剩终端 WAIT STOP
            (suffix 从链尾编译, 中间若有未开始运动段则不可跳过);
          - suffix 非空、不以 STOP 开头、以 STOP 结尾;
          - 拼接点切向连续 (STRAIGHT 同向 / ARC 入切向一致);
          - 重建的链/速度剖面合法 (SpeedProfile 自身做制动可行性校验).

        成功 → 返回新 follower (同帧变换、elapsed 时钟连续、参考位姿与速度
        在拼接瞬时不跳变); 原 follower 不被修改.
        """
        sample = self._last_sample
        if sample is None:
            return None
        if sample.acceleration < -1e-9:
            return None                       # 已进入制动区: STOP 已影响速度计划
        suffix = tuple(suffix)
        if (not suffix or suffix[0].kind == 'STOP'
                or suffix[-1].kind != 'STOP'):
            return None
        if not self._wait_tail:
            return None
        idx = sample.primitive_index
        # 活动 primitive 之后必须恰好只剩终端 WAIT STOP
        if len(self.primitives) - idx != 2:
            return None
        active = self.primitives[idx]
        if active.kind != 'STRAIGHT':
            return None                        # ARC 截断违反四分之一圆契约
        # 切向连续: suffix 首段切向必须与活动段方向一致
        at = (active.p1[0] - active.p0[0], active.p1[1] - active.p0[1])
        an = math.hypot(at[0], at[1])
        at = (at[0] / an, at[1] / an)
        st = primitive_start_tangent(suffix[0])
        if st is None or at[0] * st[0] + at[1] * st[1] < 1.0 - 1e-6:
            return None
        # 当前参考位置 (planner frame) = 截断点
        local = self.reference.sample(sample.progress_s, sample.speed, 0.0)
        remaining = active.length - (sample.progress_s - self._start_of(idx))
        if remaining <= GEOMETRY_EPS:
            return None
        truncated = deepcopy(active)
        truncated.p0 = (local.x, local.y)
        truncated.start_pose = Pose2D(local.x, local.y, 0.0)
        truncated.length = remaining
        truncated.progress = 0.0
        truncated.done = False
        junction = min(active.v_max, suffix[0].v_max)
        truncated.v_end = junction
        chain = (truncated,) + suffix
        try:
            validate_geometry(chain)
            spliced = FeedbackTrajectoryFollower(
                chain,
                planner_start=Pose2D(local.x, local.y, self.yaw_ref),
                odom_start=None, transform=self.transform,
                a_acc=self._a_acc, a_dec=self._a_dec,
                controller=self.controller,
                start_speed=sample.speed,
                position_tolerance=self.position_tolerance,
                yaw_tolerance=self.yaw_tolerance,
                velocity_tolerance=self.velocity_tolerance,
                yaw_rate_tolerance=self.yaw_rate_tolerance,
                settle_time=self.settle_time,
                expected_odom_frame=self.expected_odom_frame,
                expected_base_frame=self.expected_base_frame,
                elapsed_offset=self._last_elapsed)
        except ValueError:
            return None                         # 制动不可行等 → 保守路径
        return spliced

    def _start_of(self, index):
        """第 index 个 primitive 的链累计弧长起点."""
        acc = 0.0
        for p in self.primitives[:index]:
            if p.kind != 'STOP':
                acc += p.length
        return acc

    # ---- 周期更新 ----

    def update(self, elapsed, odometry: OdometryState) -> FollowerState:
        """Sample and control at monotonic elapsed seconds from plan start."""
        if not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('elapsed time must be finite and nonnegative')
        elapsed = float(elapsed)
        if elapsed < self._elapsed_offset - GEOMETRY_EPS:
            raise ValueError('elapsed time precedes the splice offset')
        if self._last_elapsed is not None and elapsed < self._last_elapsed:
            raise ValueError('elapsed time must be monotonic')
        self._validate_odometry(odometry)
        self._last_elapsed = elapsed

        speed = self.profile.sample(elapsed - self._elapsed_offset)
        self._last_sample = speed
        if self._stop_only:
            ref = self._stationary_reference
        else:
            local_ref = self.reference.sample(speed.progress_s, speed.speed,
                                              speed.acceleration)
            ref = self.transform.transform_reference(local_ref)
        pose = odometry.pose
        ex, ey = ref.x - pose.x, ref.y - pose.y
        yaw_error = norm_angle(ref.yaw_ref - pose.yaw)
        schedule_complete = (elapsed - self._elapsed_offset) >= self.profile.duration
        settled = (schedule_complete and math.hypot(ex, ey) <= self.position_tolerance
                   and abs(yaw_error) <= self.yaw_tolerance
                   and math.hypot(odometry.vx_world, odometry.vy_world) <= self.velocity_tolerance
                   and abs(odometry.wz) <= self.yaw_rate_tolerance)
        if settled and not self._wait_tail:
            # WAIT 尾链绝不自结束 (只有 suffix/cancel/fault 离开 HOLDING)
            if self._settled_since is None:
                self._settled_since = elapsed
            if elapsed - self._settled_since >= self.settle_time:
                self._complete = True
        else:
            self._settled_since = None
        # HOLDING 期间位置环保持参考位形 (v_ff=0, 有漂移拉回, yaw hold);
        # 零误差 + 零实测速度时 P+D 输出恰为零.
        command = (Twist2D() if self._complete else self.controller.update(
            ref, pose, measured_velocity_world=(odometry.vx_world,
                                                odometry.vy_world),
            yaw_rate=odometry.wz))
        if self._complete:
            phase = FollowerPhase.FINISHED
        elif self._stop_only or schedule_complete:
            # HOLDING: 平移前馈 0, 位置环保持参考位形, yaw_ref 不变
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
