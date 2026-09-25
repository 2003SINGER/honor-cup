#!/usr/bin/env python3
"""MotionPrimitive —— 运动基元契约 (R3 规范 §6; 麦轮平移模型).

KINDS: STRAIGHT / ARC / REVERSE / STOP
  STRAIGHT  直线平移 (p0→p1), body yaw 不变
  ARC       四分之一圆弧平移: p0 字段存圆心, yaw0 存起始极角, yaw1 存带符号角跨度,
            meta['r'] 存半径; 速度矢量 = 切向, 速度模长受加减速及 v_end 约束
  REVERSE   沿来路反向平移 (死路折返), body yaw 不变 (planner 编译为两段 STRAIGHT)
  STOP      v=0 等待 (CellMark 未完成时唯一合法行为)

铁律: primitive 只修改连续 Pose 的位置, 绝对禁止:
  修改 cell / TraversalMap / yaw / 调用导航 —— 离散状态只来自 GridEventDetector 几何事件."""

from dataclasses import dataclass, field
import math
from .pose import Pose2D

KINDS = ('STRAIGHT', 'ARC', 'REVERSE', 'STOP')
ARC_RADIUS = 0.2
GEOMETRY_EPS = 1e-9


@dataclass
class MotionPrimitive:
    kind: str                          # STRAIGHT / ARC / REVERSE / STOP
    start_pose: Pose2D                 # 起点位姿 (= 上一 primitive 终点, 链式连续)
    p0: tuple = None                   # 起点位置 (x, y); ARC: 圆心
    p1: tuple = None                   # 终点位置 (STRAIGHT)
    yaw0: float = 0.0                  # STRAIGHT: body yaw (恒定); ARC: 起始极角
    yaw1: float = 0.0                  # ARC: 带符号角跨度 (±π/2)
    length: float = 0.0                # 路径长度 (平移类 progress 域; ARC=弧长)
    duration: float = 0.0              # 时长 (STOP)
    v_max: float = 0.0                 # 本段速度上限
    v_end: float = 0.0                 # 段末速度
    preconditions: tuple = ()          # 满足才可启动 (如 'mark_complete')
    commit_point: float = None         # 越过即不可取消的进度点 (米)
    cancel_deadline: float = None      # 最迟取消/完成线 (米)
    progress: float = 0.0              # 已走长度
    done: bool = False
    meta: dict = field(default_factory=dict)   # {'grab': cell} / {'turn': side} / {'wait': cell}

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"unknown motion primitive kind: {self.kind!r}")
        if self.kind == 'STRAIGHT':
            if self.p0 is None or self.p1 is None or len(self.p0) != 2 or len(self.p1) != 2:
                raise ValueError("STRAIGHT requires 2D p0 and p1 endpoints")
            if not all(math.isfinite(v) for v in (*self.p0, *self.p1)):
                raise ValueError("STRAIGHT endpoints must be finite")
            dx = abs(self.p1[0] - self.p0[0])
            dy = abs(self.p1[1] - self.p0[1])
            if dx <= GEOMETRY_EPS and dy <= GEOMETRY_EPS:
                raise ValueError("STRAIGHT must have nonzero axis-aligned length")
            if dx > GEOMETRY_EPS and dy > GEOMETRY_EPS:
                raise ValueError(
                    f"STRAIGHT must be axis aligned; p0={self.p0}, p1={self.p1}")
            expected_length = math.hypot(dx, dy)
            if abs(self.length - expected_length) > GEOMETRY_EPS:
                raise ValueError(
                    f"STRAIGHT length must match endpoints; expected {expected_length}, got {self.length}")
        elif self.kind == 'ARC':
            radius = self.meta.get('r')
            if (not isinstance(radius, (int, float)) or not math.isfinite(radius) or
                    abs(radius - ARC_RADIUS) > GEOMETRY_EPS):
                raise ValueError(
                    f"ARC radius must be {ARC_RADIUS}; got {radius!r}")
            if (not math.isfinite(self.yaw0) or not math.isfinite(self.yaw1) or
                    abs(abs(self.yaw1) - math.pi / 2) > GEOMETRY_EPS):
                raise ValueError(
                    f"ARC sweep must be a signed quarter turn (±pi/2); got {self.yaw1!r}")
            expected_length = ARC_RADIUS * math.pi / 2
            if abs(self.length - expected_length) > GEOMETRY_EPS:
                raise ValueError(
                    f"ARC length must equal quarter-circle length {expected_length}; got {self.length}")
