#!/usr/bin/env python3
"""DFSExplorer —— 探索策略, 只管 BRANCH (规范 §6, GPT 四审 §4/§5).

WAY/DEAD 永不进入 DFS 决策 (它们的运动由 CellMark.transition 决定).
BranchState: parent_side 在第一次进入时冻结, 从 child 返回不改变.
peek_choice() 纯函数不修改状态 (KnownHorizon 预览用);
commit_enter()/commit_backtrack() 只在真实进入/离开事件时调用."""

DIRS = ('N', 'E', 'S', 'W')


class BranchState:
    def __init__(self, cell, parent_side, child_edges):
        self.cell = cell
        self.parent_side = parent_side      # 第一次进入时冻结, 永不改变
        self.children = child_edges         # 除 parent 外的全部开口
        self.explored = set()               # 已 commit 的 child


class DFSExplorer:
    def __init__(self, entry, order='LFR'):
        self.entry = entry
        self.order = order
        self.stack = []                     # [BranchState] 未完成 branch
        self.done = {}                      # cell -> BranchState (已全部探索, 防重入误 push)
        self.rel_rank = {'F': 0, 'L': 1, 'R': 2, 'B': 3}

    def rel_of(self, d, heading):
        m = {heading: 'F', OPP_M[heading]: 'B'}
        m[{'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[heading]] = 'R'
        m[{'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[heading]] = 'L'
        return m[d]

    def peek_choice(self, branch_cell, mark, entry_side, walked_fn, forbidden=()):
        """纯函数预览: 该 BRANCH 本次会走哪个 child (KnownHorizon 用, 零副作用).
        forbidden: 不可探索方向 (如场外出口边界), 从根源不作为 child.
        全部 children 已 explored → 返回 (parent_side, 'backtrack').
        非岔路 → None."""
        if mark is None or mark['kind'] != 'BRANCH':
            return None
        st = self._find_state(branch_cell)
        known_parent = st.parent_side if st else entry_side
        known_explored = set(st.explored) if st else set()
        unexplored = [d for d in mark['opens']
                      if d != known_parent and d not in known_explored
                      and d not in forbidden
                      and not walked_fn(branch_cell, d)]
        if unexplored:
            d2 = sorted(unexplored, key=lambda dd: self.order.index(self.rel_of(dd, OPP_M[known_parent]))
                        if self.rel_of(dd, OPP_M[known_parent]) in self.order else 99)[0]
            return d2, 'explore'
        return known_parent, 'backtrack'

    def commit_enter(self, branch_cell, mark, parent_side, walked_fn,
                     arrived_side=None, forbidden=()):
        """真实进入 branch 事件: 创建/沿用 BranchState, 返回 (d2, mode) 同 peek.
        parent_side: 首次创建时冻结的父方向 (不可从重访来向推测!);
        arrived_side: 本次实际从哪条边到格 (重访自 child 返回时标记 explored);
        forbidden: 不可探索方向 (场外出口), 不进 children."""
        st = self._find_state(branch_cell)
        if st is None:
            st = self.done.get(branch_cell)
            if st is not None:
                # 已完成 branch 重访 (回溯路过): 不重 push, 沿父方向继续
                return st.parent_side, 'backtrack'
            st = BranchState(branch_cell, parent_side,
                             tuple(d for d in mark['opens']
                                   if d != parent_side and d not in forbidden))
            self.stack.append(st)
        if arrived_side is not None and arrived_side in st.children:
            st.explored.add(arrived_side)   # 从该 child 子树返回 → 已消费
        unexplored = [d for d in st.children
                      if d not in st.explored and not walked_fn(branch_cell, d)]
        if unexplored:
            d2 = sorted(unexplored, key=lambda dd: self.order.index(self.rel_of(dd, OPP_M[st.parent_side]))
                        if self.rel_of(dd, OPP_M[st.parent_side]) in self.order else 99)[0]
            return d2, 'explore'
        # children 全部 explored → 弹栈归档, 沿冻结的 parent_side 返回
        self.stack.pop()
        self.done[branch_cell] = st
        return st.parent_side, 'backtrack'

    def commit_choice(self, branch_cell, child_edge):
        st = self._find_state(branch_cell)
        if st:
            st.explored.add(child_edge)

    def backtrack_target(self):
        return self.stack[-1] if self.stack else None

    def _find_state(self, cell):
        for st in self.stack:
            if st.cell == cell:
                return st
        return None


OPP_M = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
