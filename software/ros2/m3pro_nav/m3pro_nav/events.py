#!/usr/bin/env python3
"""事件模型 (R3 规范 §4).

关键 invariant:
  TraversalMap.walked 只能由 CrossedEdgeEvent 写入;
  CellVisit.entered_from 只能由 EnteredCellEvent 建立 —— 永远不从当前 heading 反推.
任何离散状态变化都必须能追溯到一个真实连续物理事件."""

from dataclasses import dataclass


@dataclass
class CrossedEdgeEvent:
    from_cell: tuple            # 跨越前所在格
    direction: str              # 跨越方向 'N'/'E'/'S'/'W'
    to_cell: tuple              # 跨越后格 (场外时出界, handler 负责拒绝)
    crossing_point: tuple       # (x, y) 几何跨越点
    timestamp: float = 0.0


@dataclass
class EnteredCellEvent:
    cell: tuple
    entered_from_side: str      # 从哪条边进入 (= OPP[direction])
    timestamp: float = 0.0
    visit_id: int = 0
