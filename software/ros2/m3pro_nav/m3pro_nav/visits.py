#!/usr/bin/env python3
"""CellVisit —— 迟分类 branch 的正式机制 (R3 规范 §5, 取代 _pending hack).

EnteredCellEvent 发生时立即创建 Visit, 永久保存 first_entered_from (真实父方向).
之后雷达让该格变 COMPLETE 且为 BRANCH → commit(parent = first_entered_from),
无论重访时从哪个方向回来. 树形迷宫中首次物理进入必经父边, 因此 first 即真."""

from dataclasses import dataclass, field


@dataclass
class CellVisit:
    cell: tuple
    first_entered_from: str         # 首次物理进入的边 (真父方向, 不可变)
    latest_entered_from: str        # 最近一次进入的边 (本次来向, 标 explored 用)
    first_ts: float
    visit_count: int = 1
    branch_committed: bool = False


class VisitRegistry:
    def __init__(self):
        self.cells = {}             # cell -> CellVisit (branch_committed 跨访问持久)

    def on_entered(self, cell, entered_from, ts=0.0):
        v = self.cells.get(cell)
        if v is None:
            v = CellVisit(cell, entered_from, entered_from, ts)
            self.cells[cell] = v
        else:
            v.latest_entered_from = entered_from
            v.visit_count += 1
        return v

    def get(self, cell):
        return self.cells.get(cell)
