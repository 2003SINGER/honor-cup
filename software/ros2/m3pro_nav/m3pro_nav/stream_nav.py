#!/usr/bin/env python3
"""StreamNav —— 薄 coordinator (规范 §8/§12; GPT 四审 R2.5: 单一认知栈).

本类不拥有任何认知状态, 只做编排:
  observe()      → EdgeMap (soft 证据) + recompute derived (TreeInference)
  commit_cell()  → 真实进格事件 → DFSExplorer.commit_enter (branch) + TraversalMap
  plan_edge()    → CellClassifier.classify → transition/peek (纯) → MoveIntent
  route_between()→ TraversalMap walked 图 BFS (RoutePlanner)
禁止: 第二套 score/mark/frontier、nearest_frontier 探索策略、MazeMap.seen 参与决策."""

import math

from .edge_map import (EdgeMap, TraversalMap, DIRV, OPP, DIRS,
                       UNKNOWN, WALL, OPEN, T_CONFIRM, T_FLIP)
from . import tree_inference
from .cell_classifier import classify as _classify, transition as _transition

C = 0.4
P_HALF = 0.2


class StreamNav:
    def __init__(self, entry, order='LFR', n=7,
                 v_cruise=0.70, a_acc=1.0, a_dec=1.0, a_lat=0.7,
                 dphi_deg=0.30, gate=0.06, rng=0.02, confirm_near=0.6,
                 assume_tree=True):
        self.n = n
        self.entry = entry
        self.order = order
        self.v_cruise = v_cruise
        self.a_acc, self.a_dec, self.a_lat = a_acc, a_dec, a_lat
        self.DPHI = math.radians(dphi_deg)
        self.gate = gate
        self.rng = rng
        self.confirm_near = confirm_near
        self.assume_tree = assume_tree
        self.v_arc = math.sqrt(a_lat * P_HALF)

        self.edges = EdgeMap(n)             # 墙真相 (唯一)
        self.traversal = TraversalMap(n)    # walked (唯一)
        from .dfs_explorer import DFSExplorer as _DFS
        self.dfs = _DFS(entry, order)
        self.exit_open = set()              # 已确认场外开口 (policy 层用)
        self.exit_cell = None
        self.blocks_seen = set()
        self.blocks_gone = set()
        self.got = 0

    # ---- 委托查询 ----
    def is_wall(self, c, d):
        return self.edges.is_wall(c, d, self.traversal)

    def is_open(self, c, d):
        return self.edges.is_open(c, d, self.traversal)

    def resolved(self, c, d):
        return self.edges.resolved(c, d, self.traversal)

    def cell_classified(self, c):
        return self.edges.cell_classified(c, self.traversal)

    def front(self, c):
        """规范定义 frontier: OPEN 且未走过 (EdgeMap+TraversalMap 组合)"""
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

    # ---- 置信 ----
    def _err(self, dist, alpha):
        a = min(abs(alpha), math.radians(89.9))
        D = max(0.05, dist * math.cos(a))
        return max(self.rng, D * self.DPHI / max(math.cos(a) ** 2, 1e-3))

    def observe(self, hits, opens):
        """每帧: soft 证据 → derived 重算 (TreeInference) → 边界/出口."""
        newly = 0
        for (ck, axis), (dist, alpha) in hits.items():
            if self._err(dist, alpha) >= self.gate:
                continue
            if self.traversal.is_walked(ck, axis):
                self.edges.contradictions += 1      # 传感器说墙, 车走过 → 诊断信号
            self.edges.observe_wall(ck, axis, dist, self.confirm_near)
        for (ck, axis, dist, alpha) in opens:
            if self._err(dist, alpha) >= self.gate:
                continue
            self.edges.observe_open(ck, axis, dist, self.confirm_near)
            if dist < self.confirm_near and (ck, axis) not in self.exit_open:
                nb = (ck[0] + DIRV[axis][0], ck[1] + DIRV[axis][1])
                if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
                    self.exit_open.add((ck, axis))
                    if self.exit_cell is None and ck != self.entry:
                        self.exit_cell = ck
                    newly += 1
        # derived 重算 (TreeInference: 前提撤销 → derived 自动消失)
        self.edges.derived = tree_inference.recompute_derived(
            self.edges, self.traversal, self.assume_tree)
        return newly

    # ---- walked 登记 (R3 前由 sim 的进格动作调用; R3 后改 crossed-edge 事件) ----
    def mark_walked(self, cell, d):
        self.traversal.mark_crossed(cell, d)

    def commit_cell(self, cell, heading):
        """真实进格事件: branch → DFS commit_enter. 返回 (d2, mode)|None"""
        mark = self.mark(cell)
        if mark and mark['kind'] == 'BRANCH':
            return self.dfs.commit_enter(cell, mark, OPP[heading],
                                         lambda c, d: self.traversal.is_walked(c, d))
        return None

    # ---- 决策 (plan_edge 纯读: branch 用 peek, 不 commit) ----
    def plan_edge(self, cell, heading):
        mark = self.mark(cell)
        if mark is None:
            return 'wait'                        # 交 CREEP_OBSERVE (R4 正式化)
        entry_side = OPP[heading]
        if mark['kind'] == 'WAY':
            d2 = _transition(mark, entry_side)
            if d2 is None or d2 == entry_side:
                # transition 失效 (重访) → 本格 frontier 或回溯
                front = self.front(cell)
                if front:
                    d2 = self._choose(front, heading)
                else:
                    nf = self._route_to_frontier(cell)
                    if nf is None:
                        return 'home' if self.edges.all_resolved(self.traversal) else 'wait'
                    d2 = nf[1]
        elif mark['kind'] == 'BRANCH':
            pk = self.dfs.peek_choice(cell, mark, entry_side,
                                      lambda c, d: self.traversal.is_walked(c, d))
            if pk is None:
                return 'wait'
            d2, mode = pk
            if mode == 'backtrack':
                # 回溯: 沿 route BFS 走向弹栈后的父 branch (树形=父链)
                tgt = self.dfs.backtrack_target()
                if tgt is None:
                    return 'home' if self.edges.all_resolved(self.traversal) else 'wait'
                seg = self.route_between(cell, tgt.cell)
                if not seg:
                    return 'wait'
                d2 = seg[0]
        else:                                        # DEAD
            d2 = _transition(mark, entry_side)
            if d2 is None:
                return 'wait'
        far = (cell[0] + DIRV[d2][0], cell[1] + DIRV[d2][1])
        turn_here = None if d2 == heading else ('cut' if d2 != OPP[heading] else 'rev')
        if turn_here == 'cut':
            turn_here = 'spin90'                     # 格心垂直转向来不及切 (激励提前打标)
        # peek far 转弯: 纯拓扑
        end_o, v_end, d3, far_cut = 0.4, self.v_cruise, None, False
        mfar = self.mark(far)
        if mfar and mfar['kind'] == 'DEAD':
            v_end = 0.0
        elif mfar and mfar['kind'] == 'WAY':
            d3 = _transition(mfar, OPP[d2])
            if d3 is not None and d3 != d2 and d3 != OPP[d2]:
                far_cut = True
                end_o, v_end = 0.25, self.v_cruise
        elif self.cell_classified(far):
            f_far = self.front(far)
            if f_far:
                d3 = self._choose(f_far, d2)
                if d3 != d2 and d3 != OPP[d2]:
                    far_cut = True
                    end_o, v_end = 0.25, self.v_cruise
            else:
                v_end = 0.0
        return {'d2': d2, 'far': far, 'end_o': end_o, 'v_end': v_end,
                'turn_here': turn_here, 'd3': d3, 'far_cut': far_cut}

    def v_cap(self, plan, o):
        if plan in ('wait', 'home', None):
            return 0.0
        far = plan['far']
        if self.mark(far) is None and not self.cell_classified(far):
            return math.sqrt(max(0.0, 2 * self.a_dec * max(0.0, 0.4 - o - 0.05)))
        return plan['v_end']

    # ---- RoutePlanner (walked 图 BFS; 与探索策略分离) ----
    def _choose(self, front, heading):
        return sorted(front, key=lambda dd: self.order.index(self.rel_of(dd, heading))
                      if self.rel_of(dd, heading) in self.order else 99)[0]

    def rel_of(self, d, heading):
        m = {heading: 'F', OPP[heading]: 'B'}
        m[{'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[heading]] = 'R'
        m[{'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[heading]] = 'L'
        return m[d]

    def route_between(self, a, b):
        """已走边图 BFS 最短路 (RoutePlanner; 树形=唯一路径). 返回方向序列."""
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

    def _route_to_frontier(self, cell):
        """BFS walked 图 → 最近的有 (OPEN 且未走) 分支的格"""
        reach = {cell: (0, None)}
        q = [cell]
        while q:
            cc = q.pop(0)
            if cc != cell and self.front(cc):
                return reach[cc]
            for d, dv in DIRV.items():
                if not self.traversal.is_walked(cc, d):
                    continue
                nb = (cc[0] + dv[0], cc[1] + dv[1])
                if nb not in reach:
                    reach[nb] = (reach[cc][0] + 1, d if cc == cell else reach[cc][1])
                    q.append(nb)
        return None


