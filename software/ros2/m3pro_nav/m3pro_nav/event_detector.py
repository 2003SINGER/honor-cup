#!/usr/bin/env python3
"""GridEventDetector —— 连续位姿 → 离散拓扑的唯一桥 (R3 规范 §3/§15).

输入 prev/cur 两帧连续 Pose, 输出该运动线段真实穿过 grid boundary 的
CrossedEdgeEvent 序列 (按 t 排序, 大 dt 跨多条边按顺序全出).
绝对禁止用 o >= 0.4 之类簿记人为判定进格 —— 事件只来自几何."""

import math
from .pose import Pose2D, C, DIRV
from .events import CrossedEdgeEvent

_EPS = 1e-9


class GridEventDetector:
    def __init__(self, n, cell_size=C):
        self.n = n
        self.C = cell_size

    def detect(self, prev: Pose2D, cur: Pose2D, timestamp=0.0):
        """返回沿运动线段依序发生的跨越事件. 无跨越 → []."""
        events = []                                   # (t, axis, m)
        for axis, (p0, p1) in ((0, (prev.x, cur.x)), (1, (prev.y, cur.y))):
            d = p1 - p0
            if abs(d) < _EPS:
                continue
            lo, hi = (p0, p1) if d > 0 else (p1, p0)
            m_start = max(0, math.floor(lo / self.C - _EPS) + 1)
            m_end = min(self.n, math.ceil(hi / self.C + _EPS) - 1)
            for m in range(m_start, m_end + 1):
                line = m * self.C
                if lo + _EPS < line < hi - _EPS:
                    t = (line - p0) / d
                    events.append((t, axis, m, d))
        events.sort(key=lambda e: e[0])
        out = []
        for t, axis, m, d in events:
            # 跨越点稍前的位置 = from_cell (严格在线前)
            tb = max(0.0, t - 1e-6)
            bx = prev.x + (cur.x - prev.x) * tb
            by = prev.y + (cur.y - prev.y) * tb
            fi = min(max(int(bx // self.C), 0), self.n - 1)
            fj = min(max(int(by // self.C), 0), self.n - 1)
            from_cell = (fi, fj)
            if axis == 0:
                direction = 'E' if d > 0 else 'W'
            else:
                direction = 'N' if d > 0 else 'S'
            dv = DIRV[direction]
            to_cell = (from_cell[0] + dv[0], from_cell[1] + dv[1])
            cx = prev.x + (cur.x - prev.x) * t
            cy = prev.y + (cur.y - prev.y) * t
            out.append(CrossedEdgeEvent(from_cell, direction, to_cell,
                                        (cx, cy), timestamp))
        return out
