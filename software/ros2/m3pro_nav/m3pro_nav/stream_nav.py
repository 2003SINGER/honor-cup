#!/usr/bin/env python3
"""StreamNav —— 薄 coordinator (规范 §8/§12; 麦轮平移模型重构版).

本类不拥有任何运动/位姿状态, 只做认知编排:
  observe()      → EdgeMap (soft 证据) + recompute derived (TreeInference, 纯 base 重算)
  on_crossed()   → crossed-edge 事件 → TraversalMap (唯一 walked 写入点)
  on_entered()   → EnteredCell 事件 → CellVisit + 岔路局部状态机 (无全局 DFS stack:
                   树结构本身是隐式递归栈, branch[cell] 记 parent/children/done/next)
  resolve_exit() → CellAction 查表: (cell, entry_side) → exit_side
  refresh_branch() → 迟分类: 当前格 CellMark 完成 → 用 first_entered_from 建状态
  home_route()   → TraversalMap walked 图 BFS (不做探索决策)

运动执行完全属于 MotionPlanner (模板编译) + MotionExecutor (位姿唯一 owner)."""

import math

from .edge_map import (EdgeMap, TraversalMap, DIRV, OPP, DIRS,
                       UNKNOWN, WALL, OPEN)
from . import tree_inference
from .cell_classifier import classify as _classify, transition as _transition
from .visits import VisitRegistry

C = 0.4


def _order_rank(d, heading, order):
    rel_map = {heading: 'F', OPP[heading]: 'B',
               {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[heading]: 'R',
               {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[heading]: 'L'}
    rel = rel_map[d]
    return order.index(rel) if rel in order else 99


class StreamNav:
    def __init__(self, entry, order='LFR', n=7,
                 dphi_deg=0.30, gate=0.06, rng=0.02, confirm_near=0.6):
        # 注意: 无 assume_tree —— 赛题官方保证迷宫为树, 规则 A 恒成立 (不设虚假开关)
        self.n = n
        self.entry = entry
        self.order = order
        self.DPHI = math.radians(dphi_deg)
        self.gate = gate
        self.rng = rng
        self.confirm_near = confirm_near

        self.edges = EdgeMap(n)                  # 墙真相 (唯一)
        self.traversal = TraversalMap(n)         # walked (唯一)
        self.blocks_seen = set()
        self.blocks_gone = set()
        self.got = 0
        self.visits = VisitRegistry()       # entered_from 只来自 EnteredCell 事件
        self.branch = {}                    # 岔路局部状态机 (无全局 DFS stack):
                                            # cell -> {'parent','children','done','next'}

    # ---- 委托查询 ----
    def is_wall(self, c, d):
        return self.edges.is_wall(c, d, self.traversal)

    def is_open(self, c, d):
        return self.edges.is_open(c, d, self.traversal)

    def is_boundary(self, c, d):
        return self.edges.is_boundary(c, d)

    def resolved(self, c, d):
        return self.edges.resolved(c, d, self.traversal)

    def cell_classified(self, c):
        return self.edges.cell_classified(c, self.traversal)

    def all_resolved(self):
        return self.edges.all_resolved(self.traversal)

    def front(self, c):
        """规范定义 frontier: OPEN 且未走过"""
        return list(self.edges.frontier(c, self.traversal))

    def mark(self, c):
        """CellMark 只能由 CellClassifier 产生 (无缓存副本)"""
        return _classify(self.edges, c, self.traversal)

    def has_block(self, c):
        return c in self.blocks_seen and c not in self.blocks_gone

    def set_block_seen(self, c, seen):
        (self.blocks_seen.add if seen else self.blocks_seen.discard)(c)

    def set_block_collected(self, c):
        self.blocks_gone.add(c)
        self.blocks_seen.discard(c)

    # ---- 出口: 从 EdgeMap 派生 (无旁路真相, 可撤销) ----

    def exit_cells(self):
        """出口候选 = boundary 边 effective OPEN 的格子 (入口除外).
        传感器误 OPEN 翻 WALL 后自动消失 —— 单一真相."""
        out = set()
        for i in range(self.n):
            for j in range(self.n):
                c = (i, j)
                if c == self.entry:
                    continue
                for d in DIRS:
                    if self.edges.is_boundary(c, d) and self.is_open(c, d):
                        out.add(c)
        return out

    # ---- 置信 ----

    def _err(self, dist, alpha):
        a = min(abs(alpha), math.radians(89.9))
        D = max(0.05, dist * math.cos(a))
        return max(self.rng, D * self.DPHI / max(math.cos(a) ** 2, 1e-3))

    def observe(self, hits, opens):
        """每帧: soft 证据 (每帧都更新, 已确认状态也更新) → derived 纯重算."""
        for (ck, axis), (dist, alpha) in hits.items():
            if self._err(dist, alpha) >= self.gate:
                continue
            if self.traversal.is_walked(ck, axis):
                self.edges.contradictions += 1   # 传感器说墙, 车走过 → 诊断信号
            self.edges.observe_wall(ck, axis, dist, self.confirm_near)
        for (ck, axis, dist, alpha) in opens:
            if self._err(dist, alpha) >= self.gate:
                continue
            self.edges.observe_open(ck, axis, dist, self.confirm_near)
        # derived 纯重算 (前提撤销 → derived 自动消失)
        self.edges.derived = tree_inference.recompute_derived(self.edges, self.traversal)

    # ---- 事件 API (R3 契约: 离散状态只能由真实几何事件改变) ----

    def on_crossed(self, from_cell, direction, ts=0.0):
        """CrossedEdgeEvent → TraversalMap.walked (全系统唯一写入点)."""
        self.traversal.mark_crossed(from_cell, direction)

    def on_entered(self, cell, entered_from_side, ts=0.0):
        """EnteredCellEvent → 岔路局部状态机:
        首次进入: parent=entered_from 冻结, children 排序, next=第一个 child;
        从 child X 回来: X=done, next=下一个未完成 child (全完 → parent).
        树结构本身就是隐式递归栈 —— 无需全局 DFS stack."""
        self.visits.on_entered(cell, entered_from_side, ts)
        mark = self.mark(cell)
        if mark is None or mark['kind'] != 'BRANCH':
            return None
        st = self.branch.get(cell)
        if st is None:
            parent = entered_from_side
            ch = [d for d in mark['opens']
                  if d != parent and not self.is_boundary(cell, d)]
            ch.sort(key=lambda d: _order_rank(d, OPP[parent], self.order))
            self.branch[cell] = {'parent': parent, 'children': tuple(ch),
                                 'done': set(), 'next': ch[0] if ch else parent}
            return self.branch[cell]['next']
        # 重访: 从 child 回来 → 标记完成; next = 下一个未完成 child 或 parent
        if entered_from_side in st['children']:
            st['done'].add(entered_from_side)
        rem = [d for d in st['children'] if d not in st['done']]
        st['next'] = rem[0] if rem else st['parent']
        return st['next']

    def refresh_branch(self, cell):
        """迟分类: 车在格内观测使其变 COMPLETE BRANCH → 用首访 entered_from 建状态."""
        if cell in self.branch:
            return None
        visit = self.visits.get(cell)
        if visit is None:
            return None
        return self.on_entered(cell, visit.first_entered_from)

    def resolve_exit(self, cell, entry_side):
        """CellAction 查表: 该格从 entry_side 进 → 从哪条边出.
        WAY/DEAD 纯查表; BRANCH 读局部状态 (未进过 → 纯 preview, 零副作用).
        返回 exit_side | None (CellMark 未完成 → WAIT)."""
        mark = self.mark(cell)
        if mark is None:
            return None
        if mark['kind'] == 'WAY':
            return _transition(mark, entry_side)
        if mark['kind'] == 'DEAD':
            return entry_side
        st = self.branch.get(cell)
        if st is None:
            # preview: 纯预测 (不建状态 —— 状态只能由真实 EnteredCell 建立)
            ch = [d for d in mark['opens']
                  if d != entry_side and not self.is_boundary(cell, d)]
            ch.sort(key=lambda d: _order_rank(d, OPP[entry_side], self.order))
            return ch[0] if ch else None
        return st['next']

    def home_route(self, cell):
        """收齐方块 → 回出口 (EXIT). 返回 (方向序列, 出口格, 出口边方向) 或 None.
        出口边方向 = 出口格上 effective OPEN 的 boundary 边 (派生, 可撤销)."""
        best = None
        for exc in self.exit_cells():
            if cell == exc:
                seg = []                         # 已在出口格: 合法, 直接沿出口边出场
            else:
                seg = self.route_between(cell, exc)
                if not seg:
                    continue                     # 不可达
            edirs = [d for d in DIRS if self.is_boundary(exc, d) and self.is_open(exc, d)]
            if not edirs:
                continue
            if best is None or len(seg) < len(best[0]):
                best = (seg, exc, edirs[0])
        return best

    def v_cap(self, d_remain):
        """最坏情况刹车距离反推 (规范 §7): v ≤ √(2·a_dec·d_remain)"""
        return math.sqrt(max(0.0, 2 * self.a_dec * max(0.0, d_remain)))

    # ---- RoutePlanner (walked 图 BFS; 不做探索决策) ----

    def _choose(self, front, heading):
        return sorted(front, key=lambda dd: self.order.index(self.rel_of(dd, heading))
                      if self.rel_of(dd, heading) in self.order else 99)[0]

    def rel_of(self, d, heading):
        m = {heading: 'F', OPP[heading]: 'B'}
        m[{'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[heading]] = 'R'
        m[{'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[heading]] = 'L'
        return m[d]

    def route_between(self, a, b):
        """已走边图 BFS 最短路 (树形=唯一路径). 返回方向序列."""
        if a == b:
            return []
        reach = {a: None}
        q = [a]
        while q:
            cc = q.pop(0)
            for d, dv in DIRV.items():
                if not self.traversal.is_walked(cc, d):
                    continue
                nb = (cc[0] + dv[0], cc[1] + dv[1])
                if nb not in reach:
                    reach[nb] = (cc, d)
                    if nb == b:
                        seg = []
                        cur = nb
                        while reach[cur] is not None:
                            pc, pd = reach[cur]
                            seg.append(pd)
                            cur = pc
                        return list(reversed(seg))
                    q.append(nb)
        return []
