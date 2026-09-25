#!/usr/bin/env python3
"""MotionPrimitive —— 运动基元契约 (R3 规范 §6; 麦轮平移模型).

KINDS: STRAIGHT / ARC / REVERSE / STOP
  STRAIGHT  直线平移 (p0→p1), body yaw 不变
  ARC       四分之一圆弧平移: p0 字段存圆心, yaw0 存起始极角, yaw1 存带符号角跨度,
            meta['r'] 存半径; 速度矢量 = 切向, |v| = v_max 恒定
  REVERSE   沿来路反向平移 (死路折返), body yaw 不变 (planner 编译为两段 STRAIGHT)
  STOP      v=0 等待 (CellMark 未完成时唯一合法行为)

铁律: primitive 只修改连续 Pose 的位置, 绝对禁止:
  修改 cell / TraversalMap / yaw / 调用导航 —— 离散状态只来自 GridEventDetector 几何事件."""

from dataclasses import dataclass, field
from .pose import Pose2D

KINDS = ('STRAIGHT', 'ARC', 'REVERSE', 'STOP')


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
