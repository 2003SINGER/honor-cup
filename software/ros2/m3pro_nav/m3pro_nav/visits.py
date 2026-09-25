#!/usr/bin/env python3
"""VisitRegistry —— 到访记录 (已降级为辅助元数据).

规划不再依赖 visit (规划 = resolve_next(prev_cell, cell) 纯图递推).
visit 只服务: branch 迟提交的 parent 依据 / 调试 / 回放 / 一致性校验.
绝对禁止: "visit 缺失 → CellAction 算不出 → runtime 卡死" 的反向依赖."""

from dataclasses import dataclass


@dataclass
class CellVisit:
    cell: tuple
    first_from_cell: tuple          # 首次真实进入的前驱格 (None = 根/入口), 不可变
    latest_from_cell: tuple         # 最近一次进入的前驱格
    first_ts: float
    visit_count: int = 1


class VisitRegistry:
    def __init__(self):
        self.cells = {}

    def on_entered(self, cell, from_cell, ts=0.0):
        v = self.cells.get(cell)
        if v is None:
            v = CellVisit(cell, from_cell, from_cell, ts)
            self.cells[cell] = v
        else:
            v.latest_from_cell = from_cell
            v.visit_count += 1
        return v

    def get(self, cell):
        return self.cells.get(cell)
