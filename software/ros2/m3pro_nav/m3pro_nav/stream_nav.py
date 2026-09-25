#!/usr/bin/env python3
"""StreamNav —— 拓扑游标 coordinator (规划 = 图递推, 2026-09-25 定稿).

规划上下文 = TopologicalCursor (prev_cell, cell): 计划正沿 prev→cell 处理 cell.
核心转移:

    resolve_next(prev_cell, cell) -> next_cell | None

  - 未成熟 (四边未全解析)        → None (horizon 到此, 边界等待)
  - OPEN 邻居 = {prev}           → prev          (DEAD)
  - OPEN 邻居 = {prev, C}        → C             (WAY)
  - OPEN 邻居 ⊇ {prev, C, D,..}  → BranchState / preview  (BRANCH)

entry_side/exit_side 不再是规划状态 —— 它们是 (prev,cell,nxt) 的派生量,
只在固定模板几何内部临时计算. "从哪来"由地图拓扑天然已知
(既然在沿 A→B 规划, A-B 这条边必然已 OPEN), 不需要事件告诉规划器.

事件职责降级 (只 commit / verify):
  CrossedEdge → TraversalMap (唯一 walked 写入点)
  EnteredCell → branch commit (parent 冻结 / child 完成) + 一致性校验

任务层 (blocks 模式): BlockMap (EMPTY 负信息) + 空死枝剪枝 (task_pruning,
动态派生证明). 物理地图绝不被剪枝污染: PRUNED ≠ WALL ≠ WALKED ≠ DONE."""

import math

from .edge_map import (EdgeMap, TraversalMap, DIRV, OPP, DIRS,
                       UNKNOWN, WALL, OPEN)
from . import tree_inference
from .cell_classifier import classify as _classify
from .visits import VisitRegistry
from .block_map import BlockMap
from . import task_pruning

C = 0.4

_DV2DIR = {(0, 1): 'N', (1, 0): 'E', (0, -1): 'S', (-1, 0): 'W'}


def _dir_of(dv):
    return _DV2DIR[(dv[0], dv[1])]


def _rel_rank(child_dv, incoming_dv, order):
    """child 方向相对行进方向的 L/F/R 序 (B=来向不会出现在候选中)."""
    m = {incoming_dv: 'F',
         (-incoming_dv[0], -incoming_dv[1]): 'B',
         (incoming_dv[1], -incoming_dv[0]): 'R',
         (-incoming_dv[1], incoming_dv[0]): 'L'}
    rel = m.get((child_dv[0], child_dv[1]))
    return order.index(rel) if rel in order else 99


class StreamNav:
    def __init__(self, entry, order='LFR', n=7,
                 dphi_deg=0.30, gate=0.06, rng=0.02, confirm_near=0.6,
                 task_mode=False):
        # 无 assume_tree —— 赛题官方保证迷宫为树, 规则 A 恒成立
        self.n = n
        self.entry = entry
        self.order = order
        self.DPHI = math.radians(dphi_deg)
        self.gate = gate
        self.rng = rng
        self.confirm_near = confirm_near
        self.task_mode = task_mode              # blocks 模式才启用任务剪枝

        self.edges = EdgeMap(n)                 # 墙真相 (唯一)
        self.traversal = TraversalMap(n)        # walked (唯一)
        self.visits = VisitRegistry()           # 降级: 到访记录 + 迟提交依据
        self.branch = {}                        # cell -> branch 状态 (commit 后)
        self.block_map = BlockMap()
        self.got = 0
        self.pruned_cells = set()               # 审计: 曾被证明的空死枝走廊

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

    def mark(self, c):
        return _classify(self.edges, c, self.traversal)

    def is_exploration_branch(self, cell):
        """Whether DFS owns multiple interior children at this cell.

        Boundary openings are not maze neighbors. The configured entry has one
        virtual parent outside the maze, so two interior neighbors make it a
        branch; every other cell needs three interior neighbors.
        """
        if self.mark(cell) is None:
            return False
        degree = len(self.open_neighbors(cell))
        return degree >= (2 if cell == self.entry else 3)

    # ---- 图递推核心 ----

    def open_neighbors(self, cell):
        """effective OPEN 且界内的相邻格集合 (从 EdgeMap+TreeInference 派生)."""
        out = set()
        for d, dv in DIRV.items():
            nb = (cell[0] + dv[0], cell[1] + dv[1])
            if 0 <= nb[0] < self.n and 0 <= nb[1] < self.n and self.is_open(cell, d):
                out.add(nb)
        return out

    def resolve_next(self, prev_cell, cell, plan_state=None):
        """图局部转移: 计划沿 prev→cell, 返回 next_cell; 信息不足 → None.
        绝不读取 Pose / Visit / 事件 —— "从哪来"由图本身已知.
        plan_state: 编译器显式推进的虚拟 branch overlay。查询本身是纯函数；
        overlay 只由 plan_branch_enter/return 等显式计划转移修改。"""
        if self.mark(cell) is None:
            return None                          # 未成熟: horizon 到此
        nbrs = self.open_neighbors(cell)
        if prev_cell is None:
            # 根 (入口格): 入口边界开口视作来向, parent_side='S'
            others = sorted(nbrs)
            deg = len(nbrs) + 1
            if deg < 2:
                return None
            if not self.is_exploration_branch(cell):
                return others[0] if others else None
            return self._branch_pick(cell, None, (0, 1), others, plan_state)
        if prev_cell not in nbrs:
            return None                          # 来路边尚未入图: 等待

        # Returning to the configured entry is a return to the virtual root.
        # Its entrance edge is not in open_neighbors, so a one-child root must
        # terminate instead of treating that child as a DEAD-cell reversal.
        if cell == self.entry:
            if len(nbrs) == 1:
                return None
            return self._branch_pick(cell, None, (0, 1), sorted(nbrs), plan_state)

        others = sorted(nbrs - {prev_cell})
        if len(nbrs) == 1:
            return prev_cell                     # DEAD: 原路回头
        din = (cell[0] - prev_cell[0], cell[1] - prev_cell[1])
        if len(nbrs) == 2:
            fwd = others[0]
            if self.task_mode and task_pruning.prove_empty_dead_branch(self, cell, fwd):
                return prev_cell                 # 前向空死枝: 当场回头 (P7)
            return fwd                           # WAY: 纯查表
        return self._branch_pick(cell, prev_cell, din, others, plan_state)

    def _branch_pick(self, cell, prev_cell, incoming_dv, other_cells, plan_state=None):
        """BRANCH 下一支路。读取 committed 或虚拟快照；不推进任何状态。

        一个 child 只有在计划明确模拟“从 child 返回 branch”时才算虚拟完成。
        进入 child 不等于完成它的子树。
        """
        pst = (plan_state or {}).get(cell) or {}
        plan_done = pst.get('done', set())
        st = self.branch.get(cell)
        if st is not None and st.get('committed'):
            done_v = set(st['done']) | plan_done
            active = [c for c in st['children']
                      if c not in done_v and not self._is_pruned(cell, c)]
            if active:
                return active[0]
            return st['parent_cell']             # 全完 → 回父 (根 = None → 早停)

        # 未 commit 分支：使用冻结的虚拟快照；单次 preview 不写回 plan_state。
        parent = pst.get('parent_cell', prev_cell)
        children = pst.get('children')
        inc = pst.get('incoming') or incoming_dv
        if children is None:
            children = [c for c in other_cells if c != parent]
            children.sort(key=lambda c: _rel_rank(
                (c[0] - cell[0], c[1] - cell[1]), inc, self.order))
        active = [c for c in children
                  if c not in plan_done and not self._is_pruned(cell, c)]
        return active[0] if active else parent

    def plan_branch_enter(self, plan_state, cell, parent_cell):
        """在计划 overlay 中冻结 branch 的 parent、incoming 和 child 顺序。

        这是可重复调用的幂等操作。真实 BranchState 优先作为快照来源；
        未 commit 的 branch 则从当前已知 OPEN 邻居生成快照。"""
        if cell in plan_state:
            return plan_state[cell]
        committed = self.branch.get(cell)
        if committed is not None and committed.get('committed'):
            state = {
                'parent_cell': committed['parent_cell'],
                'incoming': ((0, 1) if committed['parent_cell'] is None else
                             (cell[0] - committed['parent_cell'][0],
                              cell[1] - committed['parent_cell'][1])),
                'children': tuple(committed['children']),
                'done': set(committed['done']),
                'active_child': None,
                'committed_snapshot': True,
            }
        else:
            incoming = ((0, 1) if parent_cell is None else
                        (cell[0] - parent_cell[0], cell[1] - parent_cell[1]))
            children = [c for c in self.open_neighbors(cell) if c != parent_cell]
            children.sort(key=lambda c: _rel_rank(
                (c[0] - cell[0], c[1] - cell[1]), incoming, self.order))
            state = {'parent_cell': parent_cell, 'incoming': incoming,
                     'children': tuple(children), 'done': set(),
                     'active_child': None, 'committed_snapshot': False}
        plan_state[cell] = state
        return state

    @staticmethod
    def plan_branch_descend(plan_state, branch_cell, child_cell):
        """记录虚拟选择的 child；不会将 child 标成完成。"""
        state = plan_state[branch_cell]
        if child_cell not in state['children'] or child_cell in state['done']:
            raise ValueError(f"invalid virtual branch descent: {branch_cell} -> {child_cell}")
        active = state.get('active_child')
        if active not in (None, child_cell):
            raise ValueError(f"virtual child still active: {branch_cell} -> {active}")
        state['active_child'] = child_cell

    @staticmethod
    def plan_branch_return(plan_state, branch_cell, child_cell, allow_pending=False):
        """完成计划 overlay 中 child 子树，仅在模拟返回 branch 时调用。

        `allow_pending` 只用于新 horizon 从 (child, committed_branch) 边界开始，
        且 EnteredCell commit 尚未落库的情形；它不能代替本 horizon 中的 descend。
        """
        state = plan_state[branch_cell]
        active_child = state.get('active_child')
        already_committed = (allow_pending and state.get('committed_snapshot') and
                             active_child is None and child_cell in state['done'])
        resumed_committed_child = (
            allow_pending and state.get('committed_snapshot') and
            active_child is None and child_cell in state['children'] and
            child_cell not in state['done'] and child_cell != state['parent_cell'])
        if active_child != child_cell and not resumed_committed_child and not already_committed:
            raise ValueError(f"virtual return does not match active child: "
                             f"{branch_cell} <- {child_cell}")
        if already_committed:
            return
        state['done'].add(child_cell)
        state['active_child'] = None

    def _is_pruned(self, parent_cell, child_cell):
        """任务剪枝动态派生: 证明成立才剪; 方块新观测使证明失效 => 自动恢复."""
        if not self.task_mode:
            return False
        return bool(task_pruning.prove_empty_dead_branch(self, parent_cell, child_cell))

    # ---- 任务层 (BlockMap) ----

    def observe_blocks(self, obs):
        """相机批量观测 (直线连通可直达格): {cell: 'EMPTY'|'BLOCK'}."""
        self.block_map.observe(obs)

    def has_block(self, cell):
        return self.block_map.has_uncollected_block(cell)

    def collect_block(self, cell):
        self.block_map.mark_collected(cell)
        self.got += 1

    # ---- 事件 (只 commit / verify) ----

    def on_crossed(self, from_cell, direction, ts=0.0):
        self.traversal.mark_crossed(from_cell, direction)

    def on_entered(self, cell, from_cell, ts=0.0):
        """EnteredCell: 只 commit branch 状态 + 到访记录, 不喂规划器."""
        self.visits.on_entered(cell, from_cell, ts)
        if self.is_exploration_branch(cell):
            self._commit_branch(cell, from_cell)

    def _commit_branch(self, cell, from_cell):
        if not self.is_exploration_branch(cell):
            return
        st = self.branch.get(cell)
        if st is not None and st.get('committed'):
            if from_cell is not None and from_cell in st['children']:
                st['done'].add(from_cell)        # 从 child 子树回来 => 完成
            return
        if from_cell is None:
            incoming = (0, 1)                    # 根: 从入口边界北上
        else:
            incoming = (cell[0] - from_cell[0], cell[1] - from_cell[1])
        ch = [c for c in self.open_neighbors(cell) if c != from_cell]
        ch.sort(key=lambda c: _rel_rank((c[0] - cell[0], c[1] - cell[1]),
                                        incoming, self.order))
        self.branch[cell] = {'parent_cell': from_cell, 'children': tuple(ch),
                             'done': set(), 'committed': True}

    def refresh_branch(self, cell):
        """迟分类: visit 已存在 (真实进入过) 且 CellMark 现在完成且为 BRANCH
        → 用 first_from_cell (真父, 事件已记录) 补 commit. 绝不重复登记 visit."""
        st = self.branch.get(cell)
        if st is not None and st.get('committed'):
            return
        v = self.visits.get(cell)
        if v is None:
            return
        if not self.is_exploration_branch(cell):
            return
        self._commit_branch(cell, v.first_from_cell)

    # ---- 置信 ----

    def _err(self, dist, alpha):
        a = min(abs(alpha), math.radians(89.9))
        D = max(0.05, dist * math.cos(a))
        return max(self.rng, D * self.DPHI / max(math.cos(a) ** 2, 1e-3))

    def observe(self, hits, opens):
        for (ck, axis), (dist, alpha) in hits.items():
            if self._err(dist, alpha) >= self.gate:
                continue
            if self.traversal.is_walked(ck, axis):
                self.edges.contradictions += 1
            self.edges.observe_wall(ck, axis, dist, self.confirm_near)
        for (ck, axis, dist, alpha) in opens:
            if self._err(dist, alpha) >= self.gate:
                continue
            self.edges.observe_open(ck, axis, dist, self.confirm_near)
        self.edges.derived = tree_inference.recompute_derived(self.edges, self.traversal)

    # ---- 出口 / 返航 (唯一允许 BFS 的非探索任务) ----

    def exit_cells(self):
        out = set()
        for i in range(self.n):
            for j in range(self.n):
                c = (i, j)
                if c == self.entry:
                    continue
                for d in DIRS:
                    if self.is_boundary(c, d) and self.is_open(c, d):
                        out.add(c)
        return out

    def active_frontier(self):
        """还存在任务上有意义的待探边吗?
        fullinfo: 未全解析即有; blocks: OPEN-unwalked 边若不能证明空死枝即有."""
        if not self.task_mode:
            return not self.all_resolved()
        for i in range(self.n):
            for j in range(self.n):
                c = (i, j)
                for d, dv in DIRV.items():
                    nb = (i + dv[0], j + dv[1])
                    if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
                        continue
                    if not self.is_open(c, d) or self.traversal.is_walked(c, d):
                        continue
                    if not self.cell_classified(nb):
                        return True               # 未成熟: 无法证明, 必须去看
                    if not task_pruning.prove_empty_dead_branch(self, c, nb):
                        return True               # 该支路有任务价值
        return False

    def route_cells(self, a, b):
        """在已确认 OPEN 的物理图上 BFS (树形=唯一路径), 返回 [a..b].

        返航可沿已知但尚未实际走过的边规划；UNKNOWN 边不属于此图。
        """
        if a == b:
            return [a]
        reach = {a: None}
        q = [a]
        while q:
            cc = q.pop(0)
            for d, dv in DIRV.items():
                if not self.is_open(cc, d):
                    continue
                nb = (cc[0] + dv[0], cc[1] + dv[1])
                if nb in reach:
                    continue
                reach[nb] = cc
                if nb == b:
                    path = [b]
                    cur = b
                    while reach[cur] is not None:
                        cur = reach[cur]
                        path.append(cur)
                    return list(reversed(path))
                q.append(nb)
        return None

    def home_route(self, cell):
        """收齐方块 → 回出口. 返回 (path_cells, exit_dir) 或 None.
        exit_dir = 出口格上 effective OPEN 的边界边 (派生, 可撤销)."""
        best = None
        for exc in self.exit_cells():
            path = self.route_cells(cell, exc)
            if path is None:
                continue
            edirs = [d for d in DIRS
                     if self.is_boundary(exc, d) and self.is_open(exc, d)]
            if not edirs:
                continue
            if best is None or len(path) < len(best[0]):
                best = (path, edirs[0])
        return best
