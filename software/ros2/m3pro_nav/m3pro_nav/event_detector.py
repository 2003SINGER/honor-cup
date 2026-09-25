#!/usr/bin/env python3
"""GridEventDetector —— 连续位姿 → 离散拓扑的唯一桥 (R3 规范 §3/§15).

输入 prev/cur 两帧连续 Pose, 输出该运动线段真实穿过 grid boundary 的
CrossedEdgeEvent 序列 (按 t 排序, 大 dt 跨多条边按顺序全出).
绝对禁止用 o >= 0.4 之类簿记人为判定进格 —— 事件只来自几何.

边界停靠语义: 固定运动模板链的接缝恰好落在格线中点 (切点几何必然),
因此"上一帧恰在格线上、本帧离开格线"同样是一次真实跨越 (跨越完成于离线帧).
判定规则 (per grid line L, 容差 tol 消浮点噪声):
  prev 与 cur 分居 L 两侧 (严格)  → 跨越
  prev 恰在 L 上且 cur 离开 L     → 跨越 (离线方向 = 运动方向)
  prev 恰在 L 上且 cur 也在 L 上  → 无 (停靠, 无运动)"""

import math
from .pose import Pose2D, C, DIRV
from .events import CrossedEdgeEvent

_TOL = 1e-6          # "恰在格线上" 容差 (浮点噪声 << 1 tick 位移 ~1.4cm)
_EPS = 1e-9


class GridEventDetector:
    def __init__(self, n, cell_size=C):
        self.n = n
        self.C = cell_size

    def detect(self, prev: Pose2D, cur: Pose2D, timestamp=0.0):
        """返回沿运动线段依序发生的跨越事件. 无跨越 → []."""
        dx = cur.x - prev.x
        dy = cur.y - prev.y
        if abs(dx) < _EPS and abs(dy) < _EPS:
            return []
        events = []                                   # (t, axis, m, sign)
        for axis, (p0, p1) in ((0, (prev.x, cur.x)), (1, (prev.y, cur.y))):
            lo, hi = (p0, p1) if p1 >= p0 else (p1, p0)
            m_lo = max(0, math.floor(lo / self.C - _TOL))
            m_hi = min(self.n, math.ceil(hi / self.C + _TOL))
            for m in range(m_lo, m_hi + 1):
                line = m * self.C
                a, b = p0 - line, p1 - line           # 带符号偏移
                if abs(a) <= _TOL:
                    a = 0.0
                if abs(b) <= _TOL:
                    b = 0.0
                crossed = (a < 0 < b) or (b < 0 < a) or (a == 0.0 and b != 0.0)
                if crossed:
                    t = (line - p0) / (p1 - p0) if abs(p1 - p0) > _EPS else 0.0
                    t = min(max(t, 0.0), 1.0)
                    events.append((t, axis, m, 1.0 if p1 > p0 else -1.0))
        events.sort(key=lambda e: e[0])
        # 运动单位向量 (事件回退/前进取点用)
        norm = math.hypot(dx, dy)
        ux, uy = dx / norm, dy / norm
        out = []
        for t, axis, m, sign in events:
            cx = prev.x + dx * t
            cy = prev.y + dy * t
            # from_cell = 跨越点沿运动反方向回退 (严格离开格线); to_cell 同理前进
            fx = min(max(int((cx - ux * _TOL) // self.C), 0), self.n - 1)
            fy = min(max(int((cy - uy * _TOL) // self.C), 0), self.n - 1)
            from_cell = (fx, fy)
            if axis == 0:
                direction = 'E' if sign > 0 else 'W'
            else:
                direction = 'N' if sign > 0 else 'S'
            dv = DIRV[direction]
            to_cell = (from_cell[0] + dv[0], from_cell[1] + dv[1])
            out.append(CrossedEdgeEvent(from_cell, direction, to_cell,
                                        (cx, cy), timestamp))
        return out
