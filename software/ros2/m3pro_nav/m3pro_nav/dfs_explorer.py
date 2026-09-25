#!/usr/bin/env python3
"""DFSExplorer —— 探索策略, 只管 BRANCH 分支调度 (规范 §6).

stack[(branch_cell, entry_side)]; 子树探完回 branch 选下一个未探 child;
nearest_frontier 作为探索策略已被废除 (破坏深度优先的空间局部性).
BFS/最短路属于 RoutePlanner (mazemap.path_between), 职责不同."""

DIRS = ('N', 'E', 'S', 'W')
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}


class DFSExplorer:
    def __init__(self, entry, order='LFR'):
        self.entry = entry
        self.order = order
        self.stack = []                    # [(branch_cell, entry_side)]
        self.rel_rank = {'F': 0, 'L': 1, 'R': 2, 'B': 3}

    def rel_of(self, d, heading):
        m = {heading: 'F', OPP[heading]: 'B'}
        m[{'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[heading]] = 'R'
        m[{'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[heading]] = 'L'
        return m[d]

    def _choose(self, dirs, heading):
        return sorted(dirs, key=lambda dd: self.order.index(self.rel_of(dd, heading))
                      if self.rel_of(dd, heading) in self.order else 99)[0]

    def on_enter(self, cell, mark, heading, walked_fn):
        """踏入一格后的 DFS 决策 (mark 来自 CellClassifier, 可为 None=未 COMPLETE).
        返回 (d2, mode):
          mode 'explore'  — 深入新分支 (d2)
          mode 'backtrack'- 沿回溯方向走 (d2 指向父 branch 方向)
          mode 'exit'     — 冲出口 (d2)
          mode 'wait'     — 该格未 COMPLETE (交 KnownHorizon/CREEP_OBSERVE)
          mode 'done'     — 全图探完"""
        if mark is None:
            return None, 'wait'
        entry_side = OPP[heading]          # 车沿 heading 进入 → 来向边
        if mark['kind'] == 'BRANCH':
            outs = [d for d in mark['opens'] if d != entry_side]
            unexplored = [d for d in outs if not walked_fn(cell, d)]
            if unexplored:
                if not self.stack or self.stack[-1][0] != cell:
                    self.stack.append((cell, entry_side))
                d2 = self._choose(unexplored, heading)
                return d2, 'explore'
            # 全部 children 已探 → 回溯
            while self.stack:
                bcell, bentry = self.stack[-1]
                if bcell == cell:
                    self.stack.pop()
                    continue
                # 朝栈顶 branch 走 (树形迷宫父链 = 当前格的 entry 链, 简化为逐格回退)
                return entry_side, 'backtrack'
            return None, 'done'
        if mark['kind'] == 'WAY':
            outs = [d for d in mark['opens'] if d != entry_side]
            unexplored = [d for d in outs if not walked_fn(cell, d)]
            if len(unexplored) == 1:
                return unexplored[0], 'explore'
            if len(unexplored) == 0:
                # 重访 way 格 (回溯穿行): 交给调用方沿回溯方向继续
                return entry_side, 'backtrack'
            return None, 'wait'
        # DEAD: 原路返回
        return entry_side, 'backtrack'
