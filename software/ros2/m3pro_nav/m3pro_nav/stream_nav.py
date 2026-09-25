#!/usr/bin/env python3
"""StreamNav —— 薄 coordinator (规范 §8/§12; R2.5.1 终版).

本类不拥有任何认知状态, 只做编排; 唯一输出 = MoveIntent (拓扑意图):
  observe()      → EdgeMap (soft 证据) + recompute derived (TreeInference, 纯 base 重算)
  crossed()      → crossed-edge 事件 → TraversalMap (R3: 唯一 walked 写入点)
  commit_cell()  → 真实进格事件 → DFSExplorer.commit_enter
  plan_intent()  → MoveIntent (无 end_o/v_end/turn_here/far_cut —— 属 MotionPlanner)
  route_between()→ TraversalMap walked 图 BFS (RoutePlanner, 不做探索决策)

R2.5.1 修复: exit 不再是旁路真相 —— 出口候选从 boundary 边 effective OPEN 派生
(传感器误 OPEN 后翻 WALL → 出口候选自动消失); nearest-frontier 探索 fallback 已删."""

import math

from .edge_map import (EdgeMap, TraversalMap, DIRV, OPP, DIRS,
                       UNKNOWN, WALL, OPEN)
from . import tree_inference
from .cell_classifier import classify as _classify, transition as _transition
from .move_intent import MoveIntent
from .visits import VisitRegistry

C = 0.4
P_HALF = 0.2


class StreamNav:
    def __init__(self, entry, order='LFR', n=7,
                 v_cruise=0.70, a_acc=1.0, a_dec=1.0, a_lat=0.7,
                 dphi_deg=0.30, gate=0.06, rng=0.02, confirm_near=0.6):
        # 注意: 无 assume_tree —— 赛题官方保证迷宫为树, 规则 A 恒成立 (不设虚假开关)
        self.n = n
        self.entry = entry
        self.order = order
        self.v_cruise = v_cruise
        self.a_acc, self.a_dec, self.a_lat = a_acc, a_dec, a_lat
        self.DPHI = math.radians(dphi_deg)
        self.gate = gate
        self.rng = rng
        self.confirm_near = confirm_near
        self.v_cut = v_cruise                    # 斜切全程不降速 (几何余量 6.9cm)

        self.edges = EdgeMap(n)                  # 墙真相 (唯一)
        self.traversal = TraversalMap(n)         # walked (唯一)
        from .dfs_explorer import DFSExplorer as _DFS
        self.dfs = _DFS(entry, order)
        self.blocks_seen = set()
        self.blocks_gone = set()
        self.got = 0
        self.visits = VisitRegistry()       # CellVisit: entered_from 只来自 EnteredCell 事件

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
        """EnteredCellEvent → 建立/更新 CellVisit; 若该格是 COMPLETE BRANCH:
        未 commit → 首次 commit (parent=first_entered_from);
        已 commit → 仅标记 arrived child 已消费 (branch 得以正常弹栈)."""
        visit = self.visits.on_entered(cell, entered_from_side, ts)
        return self._process_branch(cell, visit)

    def refresh_visit(self, cell):
        """迟分类: 观测使格变 COMPLETE 后调用 (车仍在格内)."""
        visit = self.visits.get(cell)
        if visit is None:
            return None
        return self._process_branch(cell, visit)

    def _process_branch(self, cell, visit):
        mark = self.mark(cell)
        if mark is None or mark['kind'] != 'BRANCH':
            return None
        first_commit = not visit.branch_committed
        visit.branch_committed = True
        return self.dfs.commit_enter(
            cell, mark, visit.first_entered_from,
            lambda c, d: self.traversal.is_walked(c, d),
            arrived_side=visit.latest_entered_from,
            forbidden=self._forbidden_dirs(cell)) if (first_commit or
            self.dfs._find_state(cell) or cell in self.dfs.done) else None

    # ---- 纯拓扑预览 (MotionPlanner 用; 不 commit) ----

    def peek_continuation(self, far, d2):
        """far 格从 OPP[d2] 进入时的出口方向 (切弯建议).
        纯几何/拓扑: way 用 transition; branch 用 DFS peek (纯函数).
        边界方向 → None (探索不许冲出场)."""
        m = self.mark(far)
        if m is None:
            return None
        entry_side = OPP[d2]
        if m['kind'] == 'WAY':
            d3 = _transition(m, entry_side)
            if d3 is None or self.is_boundary(far, d3):
                return None
            return d3 if d3 != d2 and d3 != OPP[d2] else None
        if m['kind'] == 'BRANCH':
            pk = self.dfs.peek_choice(far, m, entry_side,
                                      lambda c, dd: self.traversal.is_walked(c, dd))
            if pk and pk[1] == 'explore':
                d3 = pk[0]
                if not self.is_boundary(far, d3) and d3 != d2 and d3 != OPP[d2]:
                    return d3
        return None

    # ---- 决策: 唯一输出 MoveIntent ----

    def _forbidden_dirs(self, cell):
        """不可探索方向: 场外出口边界 (出口由 home_route EXIT 流程走)"""
        return frozenset(d for d in DIRS if self.is_boundary(cell, d))

    def _backtrack_step(self, cell):
        """DFS 决定回溯 → RoutePlanner 算到栈顶父 branch 的第一步.
        栈空 → 回访迟分类 pending (观测滞后遗留的 DFS 承诺).
        返回 d2 | 'wait' (有未确认区域暂不可达) | 'home' (栈空且全图确认)."""
        tgt = self.dfs.backtrack_target()
        if tgt is None:
            # 栈空: 无未决 branch. 有未确认边 → 'wait' 不可达区域 (诊断), 否则 home
            return 'home' if self.all_resolved() else 'wait'
        seg = self.route_between(cell, tgt.cell)
        return seg[0] if seg else ('home' if self.all_resolved() else 'wait')

    def plan_intent(self, cell, entry_side):
        """标记驱动决策 → MoveIntent. 'wait'/'home' 为控制信号.
        entry_side 必须来自 CellVisit (真实进入事件), 禁止从当前 heading 反推."""
        mark = self.mark(cell)
        if mark is None:
            return 'wait'                        # 交 CREEP_OBSERVE (MotionPlanner)
        heading = OPP[entry_side]                # 探索排序用行进方向

        if mark['kind'] == 'WAY':
            d2 = _transition(mark, entry_side)
            if d2 is None:
                d2 = entry_side                  # 死路化 revisit → 原路退
            walked_out = self.traversal.is_walked(cell, d2)
            unexplored_front = self.front(cell)
            if walked_out and unexplored_front and d2 not in unexplored_front:
                d2 = self._choose(unexplored_front, heading)   # 重访但有未探分支
                mode = 'EXPLORE'
            elif walked_out:
                step = self._backtrack_step(cell)
                if step in ('wait', 'home'):
                    return step
                d2, mode = step, 'BACKTRACK'
            else:
                mode = 'EXPLORE'

        elif mark['kind'] == 'BRANCH':
            # commit 只发生在 on_entered/refresh_visit (真实事件);
            # plan_intent 对 branch 永远是纯 peek, 零副作用
            pk = self.dfs.peek_choice(cell, mark, entry_side,
                                      lambda c, d: self.traversal.is_walked(c, d),
                                      forbidden=self._forbidden_dirs(cell))
            if pk is None:
                return 'wait'
            d2, mode = pk
            if mode == 'backtrack':
                step = self._backtrack_step(cell)
                if step in ('wait', 'home'):
                    return step
                d2, mode = step, 'BACKTRACK'
            else:
                mode = 'EXPLORE'

        else:                                        # DEAD
            d2 = _transition(mark, entry_side) or entry_side
            mode = 'BACKTRACK'

        if self.is_boundary(cell, d2):               # 探索不许冲出场 (出口走 EXIT 流程)
            step = self._backtrack_step(cell)
            if step in ('wait', 'home'):
                return step
            d2, mode = step, 'BACKTRACK'

        far = (cell[0] + DIRV[d2][0], cell[1] + DIRV[d2][1])
        cont = self.peek_continuation(far, d2) if mode == 'EXPLORE' else None
        requires_stop = self.has_block(far)          # 到 far 需停车抓方块
        return MoveIntent(next_edge=d2, mode=mode, target_cell=far,
                          preferred_continuation=cont, requires_stop=requires_stop)

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
