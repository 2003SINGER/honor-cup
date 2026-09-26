#!/usr/bin/env python3
"""MotionRuntimeCore —— 运动执行状态机 (纯 Python, 无 ROS, 无 plant).

职责边界 (GPT 定稿):
  本模块只负责: 计划生命周期 (load_plan/append_suffix/reset)、安全状态
  (DISARMED/ARMED/FAULT)、反馈新鲜度看门狗、把 FeedbackTrajectoryFollower
  的相位映射为运行时状态与零/非零指令。
  不做 PID 数学 (在 follower/controller 内), 不做地图/DFS/雷达, 不 import ROS。

状态分两层, 绝不混合:
  RuntimeState : IDLE / TRACKING / HOLDING / FINISHED / FAULT
      IDLE      无计划 → 零指令
      TRACKING  正在执行 STRAIGHT/ARC/REVERSE → follower 采样
      HOLDING   ActionHorizon 到 UNKNOWN / 显式 STOP / 计划耗尽:
                平移前馈 0, 保持位形与 yaw_ref, progress 冻结, 等待后续 suffix
      FINISHED  计划正常结束 → 零指令
      FAULT     反馈超时/NaN/frame 错/计划非法 → 零指令, 必须显式 reset
  SafetyState  : DISARMED / ARMED / FAULT
      DISARMED  不允许发运动指令 (默认)
      ARMED     允许 follower 指令上线
      FAULT     强制零指令, 不可自动恢复, 必须 reset() 后重新 arm()

计划生命周期原则:
  load_plan()      仅 IDLE/FINISHED 且 ARMED 时可调; 需要已有实测反馈作锚点
  append_suffix()  仅 TRACKING/HOLDING 可调; 只影响未来未开始部分:
                   - 活动 primitive 永不被重编 (排队到计划耗尽时应用)
                   - 必须与本计划尾部几何连续, 否则立即拒绝 (禁止自动连接线)
  HOLDING + 连续 suffix → TRACKING (无位姿跳变, 不重算已执行速度曲线)
  STOP 永不吞掉: 无法安全消除时保留 STOP"""

from enum import Enum
from dataclasses import dataclass
import math

from .feedback_trajectory_follower import (FeedbackTrajectoryFollower,
                                           FollowerPhase)
from .motion_primitive import MotionPrimitive, GEOMETRY_EPS
from .pose import Pose2D, Twist2D
from .position_controller import PositionController


class RuntimeState(Enum):
    IDLE = 'IDLE'
    TRACKING = 'TRACKING'
    HOLDING = 'HOLDING'
    FINISHED = 'FINISHED'
    FAULT = 'FAULT'


class SafetyState(Enum):
    DISARMED = 'DISARMED'
    ARMED = 'ARMED'
    FAULT = 'FAULT'


@dataclass(frozen=True)
class FeedbackSample:
    """外部实测反馈 (唯一位姿来源; 本模块绝不自积分位姿)."""
    time: float            # 反馈时间戳 (秒, 与 update 的 tick_time 同钟)
    pose: Pose2D           # 世界(odom)系实测位姿
    vx: float              # 世界系实测平移速度
    vy: float
    wz: float              # 实测角速度


@dataclass(frozen=True)
class RuntimeOutput:
    state: RuntimeState
    safety: SafetyState
    command: Twist2D
    follower: object            # FollowerState 或 None (IDLE/FAULT)
    fault_reason: str | None


def primitive_end(prim):
    """primitive 的几何终点 (planning frame)."""
    if prim.kind == 'STRAIGHT':
        return prim.p1
    if prim.kind == 'ARC':
        r = prim.meta['r']
        a = prim.yaw0 + prim.yaw1
        return (prim.p0[0] + r * math.cos(a), prim.p0[1] + r * math.sin(a))
    return (prim.start_pose.x, prim.start_pose.y)


def _validate_chain(primitives):
    chain = tuple(primitives)
    if not chain:
        raise ValueError('plan must contain at least one primitive')
    for prim in chain:
        if not isinstance(prim, MotionPrimitive):
            raise ValueError('plan entries must be MotionPrimitive instances')
    return chain


class MotionRuntimeCore:
    def __init__(self, *, a_acc=1.0, a_dec=1.0, controller=None,
                 position_tolerance=0.01, yaw_tolerance=0.02,
                 velocity_tolerance=0.01, yaw_rate_tolerance=0.02,
                 settle_time=0.1, max_feedback_age_s=0.5,
                 expected_odom_frame=None, expected_base_frame=None):
        self._a_acc = a_acc
        self._a_dec = a_dec
        self._controller = (controller if controller is not None
                            else PositionController())
        self._position_tolerance = position_tolerance
        self._yaw_tolerance = yaw_tolerance
        self._velocity_tolerance = velocity_tolerance
        self._yaw_rate_tolerance = yaw_rate_tolerance
        self._settle_time = settle_time
        self._max_feedback_age_s = max_feedback_age_s
        self._expected_odom_frame = expected_odom_frame
        self._expected_base_frame = expected_base_frame

        self._safety = SafetyState.DISARMED
        self._state = RuntimeState.IDLE
        self._follower = None
        self._queued_suffix = None       # TRACKING 期间排队, HOLDING 时应用
        self._plan_started_at = None     # follower elapsed 的时钟原点
        self._last_feedback_time = None  # 最新反馈时间戳 (新鲜度看门狗)
        self._last_pose = None           # 最新实测位姿 (load/apply 锚点)
        self._fault_reason = None

    # ---- 状态只读 ----

    @property
    def state(self):
        return self._state

    @property
    def safety(self):
        return self._safety

    @property
    def fault_reason(self):
        return self._fault_reason

    @property
    def has_feedback(self):
        """是否已有实测反馈 (load_plan 的 odom 锚点前提)."""
        return self._last_feedback_time is not None

    @property
    def plan_tail(self):
        """当前计划 (含排队 suffix) 的几何尾点; 无计划 → None."""
        chain = self._active_chain()
        return primitive_end(chain[-1]) if chain else None

    def _active_chain(self):
        if self._follower is None:
            return None
        chain = list(self._follower.primitives)
        if self._queued_suffix:
            chain += list(self._queued_suffix)
        return chain

    # ---- 安全 ----

    def arm(self):
        if self._safety == SafetyState.FAULT:
            raise RuntimeError('cannot arm from FAULT: explicit reset required')
        self._safety = SafetyState.ARMED

    def disarm(self):
        if self._safety != SafetyState.FAULT:
            self._safety = SafetyState.DISARMED

    def reset(self):
        """FAULT 的唯一出路: 清计划与故障, 回 DISARMED (需再 arm)."""
        self._safety = SafetyState.DISARMED
        self._state = RuntimeState.IDLE
        self._follower = None
        self._queued_suffix = None
        self._plan_started_at = None
        self._fault_reason = None

    def _fault(self, reason):
        self._safety = SafetyState.FAULT
        self._state = RuntimeState.FAULT
        self._fault_reason = reason
        return self._zero_output()

    def report_fault(self, reason):
        """外部安全事件入口 (如 ARMED 期间 /cmd_vel 出现其他发布者):
        FAULT + 零指令, 不可自动恢复, 必须 reset() 后重新 arm()."""
        if self._safety != SafetyState.FAULT:
            return self._fault(reason or 'external fault reported')
        return self._zero_output()

    # ---- 计划生命周期 ----

    def load_plan(self, primitives, planner_yaw=None):
        """装载完整计划. 仅 IDLE/FINISHED + ARMED; 需要已有实测反馈作锚点.

        planner_yaw: 计划坐标系 (planner frame) 的底盘朝向参考. None →
        沿用实测 odom yaw (规划系与 odom 对齐的探针场景); maze 帧计划
        (NavRuntime) 必须显式传 maze 帧朝向, 否则帧变换会差一个锚定角."""
        if self._state not in (RuntimeState.IDLE, RuntimeState.FINISHED):
            raise RuntimeError(
                f'load_plan requires IDLE/FINISHED, current={self._state.value}')
        if self._safety != SafetyState.ARMED:
            raise RuntimeError('load_plan requires ARMED runtime')
        if self._last_feedback_time is None:
            raise ValueError('a measured feedback sample is required before '
                             'load_plan (anchor for odom frame)')
        chain = _validate_chain(primitives)
        follower = self._build_follower(chain, self._last_pose,
                                        planner_yaw=planner_yaw)
        self._follower = follower
        self._queued_suffix = None
        self._plan_started_at = None       # 由下一次 update 的 tick_time 锚定
        self._state = (RuntimeState.HOLDING
                       if follower.primitives[0].kind == 'STOP'
                       else RuntimeState.TRACKING)

    def tail_wait_stop(self):
        """当前计划尾部若是 WAIT STOP → (start_pose(maze/planner 帧), cell);
        否则 None. NavRuntime 用它决定何时延长 horizon / 从哪续编."""
        chain = self._active_chain()
        if not chain:
            return None
        last = chain[-1]
        if last.kind == 'STOP' and last.meta.get('wait') is not None:
            return (last.start_pose.x, last.start_pose.y), last.meta['wait']
        return None

    def append_suffix(self, primitives):
        """追加未来 suffix. 只影响未开始部分; 活动 primitive 永不重编;
        必须与当前计划尾部几何连续, 否则立即拒绝 (Gate I).

        TRACKING 时的两种路径 (GPT P1-7 定稿):
          safe-extension  终端 WAIT STOP 尚未影响活动 primitive 速度计划 →
                          删 STOP 无缝续 suffix, 不停车 (follower.try_splice);
          保守排队        其余情形 → 刹到计划尾再续 (与仿真端语义一致)."""
        if self._state not in (RuntimeState.TRACKING, RuntimeState.HOLDING):
            raise RuntimeError(
                f'append_suffix requires TRACKING/HOLDING, current={self._state.value}')
        suffix = _validate_chain(primitives)
        if suffix[-1].kind != 'STOP':
            raise ValueError('suffix must end with STOP')
        tail = self.plan_tail
        first = suffix[0]
        if first.kind == 'STRAIGHT':
            suffix_start = first.p0
        elif first.kind == 'ARC':
            r = first.meta['r']
            suffix_start = (first.p0[0] + r * math.cos(first.yaw0),
                            first.p0[1] + r * math.sin(first.yaw0))
        else:
            suffix_start = (first.start_pose.x, first.start_pose.y)
        if math.hypot(suffix_start[0] - tail[0], suffix_start[1] - tail[1]) > GEOMETRY_EPS:
            raise ValueError(
                f'suffix is not continuous with plan tail: '
                f'tail={tail} suffix_start={suffix_start}')
        if self._state == RuntimeState.HOLDING:
            self._apply_suffix(suffix)
            return
        # TRACKING: 先尝试保守 safe-extension (删尚未影响速度计划的 WAIT STOP)
        if self._queued_suffix is None:
            spliced = self._follower.try_splice(suffix)
            if spliced is not None:
                self._follower = spliced       # elapsed 时钟连续, 不重锚
                return
            self._queued_suffix = suffix       # 排队: 计划耗尽进入 HOLDING 时应用
        else:
            # 已有排队 suffix: 新 suffix 须与其尾连续, 追加到队尾
            queued = self._queued_suffix
            qtail = primitive_end(queued[-1])
            if math.hypot(suffix_start[0] - qtail[0],
                          suffix_start[1] - qtail[1]) > GEOMETRY_EPS:
                raise ValueError(
                    f'suffix is not continuous with queued tail: '
                    f'tail={qtail} suffix_start={suffix_start}')
            self._queued_suffix = queued + suffix

    def _apply_suffix(self, suffix):
        """从 HOLDING 位形无缝续接 suffix: 沿用原帧变换 (不重锚);
        无原 follower (理论不可达) 时以实测位姿为锚."""
        first = suffix[0]
        if first.kind == 'STRAIGHT':
            start_xy = first.p0
        elif first.kind == 'ARC':
            r = first.meta['r']
            start_xy = (first.p0[0] + r * math.cos(first.yaw0),
                        first.p0[1] + r * math.sin(first.yaw0))
        else:
            start_xy = (first.start_pose.x, first.start_pose.y)
        if self._follower is not None:
            # 同一计划系内续接: 帧变换与 yaw_ref 必须与原 follower 一致
            planner_start = Pose2D(start_xy[0], start_xy[1],
                                   self._follower.yaw_ref)
            self._follower = FeedbackTrajectoryFollower(
                suffix, planner_start=planner_start,
                odom_start=self._last_pose,
                transform=self._follower.transform,
                a_acc=self._a_acc, a_dec=self._a_dec,
                controller=self._controller,
                position_tolerance=self._position_tolerance,
                yaw_tolerance=self._yaw_tolerance,
                velocity_tolerance=self._velocity_tolerance,
                yaw_rate_tolerance=self._yaw_rate_tolerance,
                settle_time=self._settle_time,
                expected_odom_frame=self._expected_odom_frame,
                expected_base_frame=self._expected_base_frame)
        else:
            self._follower = self._build_follower(suffix, self._last_pose)
        self._plan_started_at = None          # 重置 elapsed 原点
        self._queued_suffix = None
        self._state = (RuntimeState.HOLDING
                       if suffix[0].kind == 'STOP'
                       else RuntimeState.TRACKING)

    def _build_follower(self, chain, odom_start, planner_yaw=None):
        first = chain[0]
        if first.kind == 'STRAIGHT':
            start_xy = first.p0
        elif first.kind == 'ARC':
            r = first.meta['r']
            start_xy = (first.p0[0] + r * math.cos(first.yaw0),
                        first.p0[1] + r * math.sin(first.yaw0))
        else:
            start_xy = (first.start_pose.x, first.start_pose.y)
        # 底盘朝向参考: 显式 planner_yaw 优先 (maze 帧计划); 否则假设
        # 规划系与 odom 对齐, 取实测 yaw (探针场景). 编译链的 start_pose.yaw
        # 是几何占位符, 不承载物理朝向.
        yaw_ref = odom_start.yaw if planner_yaw is None else float(planner_yaw)
        planner_start = Pose2D(start_xy[0], start_xy[1], yaw_ref)
        return FeedbackTrajectoryFollower(
            chain, planner_start=planner_start, odom_start=odom_start,
            a_acc=self._a_acc, a_dec=self._a_dec, controller=self._controller,
            position_tolerance=self._position_tolerance,
            yaw_tolerance=self._yaw_tolerance,
            velocity_tolerance=self._velocity_tolerance,
            yaw_rate_tolerance=self._yaw_rate_tolerance,
            settle_time=self._settle_time,
            expected_odom_frame=self._expected_odom_frame,
            expected_base_frame=self._expected_base_frame)

    # ---- 周期更新 ----

    def update(self, tick_time, feedback):
        """每控制周期调用一次. tick_time 单调递增 (控制钟);
        feedback 可为 None (本 tick 无新样本, 看门狗照常运行)."""
        if not math.isfinite(tick_time) or tick_time < 0:
            raise ValueError('tick_time must be finite and nonnegative')
        if (self._safety == SafetyState.FAULT
                or self._state == RuntimeState.FAULT):
            return self._zero_output()

        # 反馈新鲜度看门狗 (Gate K)
        if feedback is not None:
            if not math.isfinite(feedback.time):
                return self._fault('feedback timestamp is not finite')
            if (self._last_feedback_time is not None
                    and feedback.time < self._last_feedback_time - 1e-9):
                return self._fault('feedback time went backwards')
            self._last_feedback_time = feedback.time
            self._last_pose = feedback.pose
        if (self._last_feedback_time is not None
                and tick_time - self._last_feedback_time > self._max_feedback_age_s):
            return self._fault('feedback is stale')

        if self._state == RuntimeState.IDLE or self._follower is None:
            self._state = RuntimeState.IDLE
            return self._zero_output()

        if self._plan_started_at is None:
            self._plan_started_at = tick_time
        elapsed = tick_time - self._plan_started_at

        try:
            fs = self._follower.update(elapsed, self._to_odometry(feedback))
        except ValueError as exc:
            return self._fault(f'follower rejected feedback: {exc}')

        # 计划耗尽进入 HOLDING 且有排队 suffix → 无缝续接 (Gate G)
        if (fs.phase in (FollowerPhase.HOLDING, FollowerPhase.FINISHED)
                and self._queued_suffix is not None):
            suffix = self._queued_suffix
            self._queued_suffix = None
            self._apply_suffix(suffix)
            if feedback is not None:
                return self.update(tick_time, feedback)
            return self._zero_output()

        if fs.phase == FollowerPhase.TRACKING:
            self._state = RuntimeState.TRACKING
        elif fs.phase == FollowerPhase.HOLDING:
            self._state = RuntimeState.HOLDING
        else:
            self._state = RuntimeState.FINISHED

        command = fs.command
        if self._safety != SafetyState.ARMED:
            command = Twist2D()               # DISARMED: 不允许发运动指令
        return RuntimeOutput(self._state, self._safety, command, fs,
                             self._fault_reason)

    # ---- 内部 ----

    def _last_feedback_time_check(self):
        return self._last_feedback_time

    def _to_odometry(self, feedback):
        from .odometry_adapter import OdometryState
        return OdometryState(feedback.pose, feedback.vx, feedback.vy,
                             feedback.wz, feedback.time,
                             self._expected_odom_frame or 'odom',
                             self._expected_base_frame or 'base_link')

    def _zero_output(self):
        return RuntimeOutput(self._state, self._safety, Twist2D(), None,
                             self._fault_reason)
