#!/usr/bin/env python3
"""MoveIntent —— StreamNav 的唯一输出契约 (规范 §8; R3 边界冻结).

只含拓扑意图, 不含任何执行细节 (end_o/v_end/turn_here/far_cut 属于
MotionPlanner, 由其根据实时 Pose 生成 MotionPrimitive —— 旧 plan dict 已废除).

mode:
  EXPLORE   — 深入新分支
  BACKTRACK — 沿已知路径回溯 (走向 DFS 栈顶父 branch / 原路退出死路)
  EXIT      — 冲出口 (收齐方块后回家)
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class MoveIntent:
    next_edge: str                      # 本格出发走哪条边 ('N'/'E'/'S'/'W')
    mode: str = 'EXPLORE'               # EXPLORE / BACKTRACK / EXIT
    target_cell: Optional[tuple] = None     # next_edge 的邻格
    preferred_continuation: Optional[str] = None  # 纯拓扑预览: far 格的出口 (切弯建议, 非承诺)
    requires_stop: bool = False         # 到 target 后必须停车 (抓方块/死路确认)

    def __post_init__(self):
        if self.target_cell is None and self.next_edge is not None:
            from .edge_map import DIRV
            # target_cell 由调用方填 (StreamNav 填); 此处仅兜底防 None
