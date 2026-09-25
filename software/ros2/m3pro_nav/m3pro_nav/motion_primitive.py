#!/usr/bin/env python3
"""MotionPrimitive —— 运动基元契约 (R3 规范 §6).

统一字段: start_pose / 几何 (p0→p1, yaw0→yaw1) / preconditions / commit_point /
cancel_deadline / progress / done / meta.
铁律: primitive 只修改连续 Pose (由 executor 积分), 绝对禁止:
  修改 cell / TraversalMap / 调用 DFS / mark_crossed —— 离散状态只来自几何事件."""

from dataclasses import dataclass, field
from .pose import Pose2D

KINDS = ('STRAIGHT', 'CUT90', 'SPIN', 'CREEP', 'STOP')


@dataclass
class MotionPrimitive:
    kind: str                          # STRAIGHT / CUT90 / SPIN / CREEP / STOP
    start_pose: Pose2D                 # 起点位姿 (= 上一 primitive 终点, 链式连续)
    p0: tuple = None                   # 起点位置 (x, y)
    p1: tuple = None                   # 终点位置 (STRAIGHT/CUT90/CREEP)
    yaw0: float = 0.0
    yaw1: float = 0.0                  # 终点朝向 (CUT90/SPIN)
    length: float = 0.0                # 路径长度 (平移类 progress 域)
    duration: float = 0.0              # 时长 (SPIN/STOP)
    v_max: float = 0.0                 # 本段速度上限
    v_end: float = 0.0                 # 段末速度
    preconditions: tuple = ()          # 满足才可启动 (如 'mark_complete', 'before_commit_line')
    commit_point: float = None         # 越过即不可取消的进度点 (米)
    cancel_deadline: float = None      # 最迟取消/完成线 (米, CREEP latest_safe_stop)
    progress: float = 0.0              # 已走长度
    done: bool = False
    meta: dict = field(default_factory=dict)   # {'grab': cell} / {'cut': (d2, d3)}
