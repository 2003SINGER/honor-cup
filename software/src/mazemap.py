#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MazeMap —— 在线迷宫地图数据结构（纯 Python，无 ROS 依赖，车上/仿真/可视化共用）

建模：节点 = 格中心(路口)，边 = 黑线走廊，格距 0.4m，7×7。

两态生命周期（对应传感器能告诉你什么）：
  节点: seen   从父节点的分叉里"知道它存在"（还没踏入）
        visited 踏入过 → 路口检测会报出它的全部分支
  边:   seen   在路口看到分叉存在（知道有路，不知道通向哪）
        walked 开过至少一遍（已知走向）
  开口: 节点某方向通向边界外 = 出口开口 → 该节点标 exit

关键查询：
  frontier(cell)        该路口还没走过的已知分支（DFS 的选择集）
  fully_explored()      图遍历完成 = 没有任何 seen-未走 的边
  path_between(a, b)    已走边上的最短路（树形迷宫=唯一路径）→ 回终点用
  to_json / from_json   存盘 / 传给可视化
  render()              ASCII 图：`---` walked · ` ~ ` seen未走 · 空格 unknown

决策约定（与仿真/实车一致）：
  1. DFS 沿 frontier 走，没 frontier 就沿栈回溯
  2. 出口已发现 且 方块收齐 → 停止探索，path_between 直奔出口（"跳过"）
  3. fully_explored 后仍未收齐 → 也回出口（剩下的靠罚时权衡，规则公布后调）
"""

import json

DIRS = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}


class MazeMap:
    def __init__(self, n=7, entry=None):
        self.n = n
        self.entry = entry
        self.nodes = {}          # (i,j) -> {'visited':bool,'exit':bool,'block':bool,'edges':{dir:'seen'|'walked'}}
        self.exit_cell = None    # 出口所在格（发现即记）

    # ---- 节点/边登记 ----

    def _node(self, cell):
        if cell not in self.nodes:
            self.nodes[cell] = {'visited': False, 'exit': False, 'block': False, 'edges': {}}
        return self.nodes[cell]

    def touch(self, cell):
        """踏入节点（路口检测生效的前提）"""
        self._node(cell)['visited'] = True

    def open_edge(self, cell, d):
        """在 cell 看到朝 d 的分叉。通向边界外 → 标记出口。返回邻格坐标(界内)或 None。
           注意：不把 walked 降级成 seen（重复探测同一路口是常态）。"""
        # 注意: open_edge 是"远程观测到开口", 不算踏入 → 不 touch (visited 语义=车真进过)
        nb = (cell[0] + DIRS[d][0], cell[1] + DIRS[d][1])
        if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
            if cell != self.entry:          # 入口格的对外开口 = 来时的路，不是出口
                self._node(cell)['exit'] = True
                if self.exit_cell is None:
                    self.exit_cell = cell
            return None
        if self._node(cell)['edges'].get(d) != 'walked':
            self._node(cell)['edges'][d] = 'seen'
        if self._node(nb)['edges'].get(OPP[d]) != 'walked':
            self._node(nb)['edges'][OPP[d]] = 'seen'
        return nb

    def walk_edge(self, cell, d):
        """沿 d 开过去（物理走一遍）。返回邻格。"""
        nb = self.open_edge(cell, d)
        if nb is None:
            raise ValueError(f'cannot walk out of maze at {cell}:{d}')
        self._node(cell)['edges'][d] = 'walked'
        self._node(nb)['edges'][OPP[d]] = 'walked'
        self.touch(nb)
        return nb

    def mark_block(self, cell):
        self._node(cell)['block'] = True

    # ---- 查询 ----

    def frontier(self, cell):
        """该路口还没走过的已知分支"""
        return [d for d, s in self._node(cell)['edges'].items() if s == 'seen']

    def fully_explored(self):
        """所有已发现边的走向都已知（树形迷宫：= 全图遍历完成）"""
        for nd in self.nodes.values():
            for s in nd['edges'].values():
                if s == 'seen':
                    return False
        return True

    def path_between(self, a, b):
        """只走 walked 边的最短路（BFS）。树形迷宫 = 唯一路径。返回方向序列。"""
        if a == b:
            return []
        prev = {a: None}
        queue = [a]
        while queue:
            c = queue.pop(0)
            for d, s in self._node(c)['edges'].items():
                if s != 'walked':
                    continue
                nb = (c[0] + DIRS[d][0], c[1] + DIRS[d][1])
                if nb not in prev:
                    prev[nb] = (c, d)
                    if nb == b:
                        path = []
                        cur = nb
                        while prev[cur] is not None:
                            pc, pd = prev[cur]
                            path.append(pd)
                            cur = pc
                        path.reverse()
                        return path
                    queue.append(nb)
        raise ValueError(f'no walked path {a} -> {b}')

    # ---- 序列化 / 可视化 ----

    def to_json(self):
        return json.dumps({
            'n': self.n, 'entry': list(self.entry) if self.entry else None,
            'exit_cell': list(self.exit_cell) if self.exit_cell else None,
            'nodes': {f'{i},{j}': nd for (i, j), nd in self.nodes.items()},
        }, ensure_ascii=False, indent=1)

    @classmethod
    def from_json(cls, s):
        d = json.loads(s)
        m = cls(d['n'], tuple(d['entry']) if d['entry'] else None)
        m.exit_cell = tuple(d['exit_cell']) if d['exit_cell'] else None
        for k, nd in d['nodes'].items():
            i, j = map(int, k.split(','))
            m.nodes[(i, j)] = nd
        return m

    def render(self):
        """ASCII：`---` walked · ` ~ ` seen未走 · 空格 unknown；E入口 X出口 *方块"""
        g = [['+' + '   +' * self.n for _ in range(0)] for _ in range(0)]
        top = '+' + ''.join('---+' if 'N' in self.nodes.get((i, self.n - 1), {}).get('edges', {})
                            and self.nodes[(i, self.n - 1)]['edges']['N'] == 'walked' else '   +'
                            for i in range(self.n))
        lines = [top]
        for j in reversed(range(self.n)):
            row = ''
            for i in range(self.n):
                nd = self.nodes.get((i, j))
                ch = ' . '
                if nd is None:
                    ch = ' ? '
                else:
                    if nd['exit']:
                        ch = ' X '
                    elif (i, j) == self.entry:
                        ch = ' E '
                    elif nd['block']:
                        ch = ' * '
                    elif not nd['visited']:
                        ch = ' , '
                row += ch + ('|' if nd and nd['edges'].get('E') == 'walked'
                             else '·' if nd and nd['edges'].get('E') == 'seen' else ' ')
            lines.append(row)
            brow = '+'
            for i in range(self.n):
                nd = self.nodes.get((i, j))
                s = nd['edges'].get('S') if nd else None
                brow += ('---+' if s == 'walked' else ' · +' if s == 'seen' else '   +')
            lines.append(brow)
        return '\n'.join(lines)


# ---------------- 自检：与仿真器对拍 ----------------

if __name__ == '__main__':
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
    from maze_sim import gen_maze, DIRS as SD, OPP as SO, C, explore, N

    seed, order = 3, 'LFR'
    walls, entry, ex, _ = gen_maze(seed)
    true_open = {c: [d for d in SD if d not in walls[c]] for c in walls}
    blocks = {(1, 5), (3, 4), (6, 2), (0, 3), (4, 3), (2, 6), (5, 0), (6, 6)} - {entry, ex}

    # --- 用 MazeMap 跑一遍 DFS（显式栈，frontier 驱动）---
    import random as _r
    rnd = _r.Random(seed)
    m = MazeMap(N, entry)
    cell, heading = entry, 'N'
    dist, turns = 0.0, 0
    stack = [entry]
    m.touch(entry)

    def rel_of(d, h):
        rot_r = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[h]
        rot_l = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[h]
        return {h: 'F', SO[h]: 'B', rot_r: 'R', rot_l: 'L'}[d]

    def dir_to(a, b):
        return next(d for d, (dx, dy) in SD.items() if (a[0] + dx, a[1] + dy) == b)

    while True:
        for d in true_open[cell]:            # 路口检测：报出全部分支
            m.open_edge(cell, d)
        if cell in blocks:
            m.mark_block(cell)

        front = m.frontier(cell)
        if front:
            front.sort(key=lambda d: order.index(rel_of(d, heading)) if rel_of(d, heading) in order else 99)
            d = front[0]
            forward = True
        elif len(stack) > 1:                 # 回溯：退到路径栈上一格
            stack.pop()
            d = dir_to(cell, stack[-1])
            forward = False
        else:
            break                            # 回到入口且无 frontier → 遍历完成

        dist += C
        if heading != d:
            turns += 1
            heading = d
        cell = m.walk_edge(cell, d)
        if forward:
            stack.append(cell)

    print(f'[MazeMap DFS] seed={seed} order={order}  路程={dist:.1f}m  节点={len(m.nodes)}  '
          f'fully_explored={m.fully_explored()}  exit={m.exit_cell}')

    # 与仿真器对拍（同种子同顺序；仿真器含提前终止，无终止版路程应等于 DFS 全遍历 2×边数×C）
    ref = explore(walls, entry, ex, order, blocks)
    print(f'[maze_sim  ] ref 路程={ref["dist"]:.1f}m (含提前终止)   全遍历理论值={2 * (N * N - 1) * C:.1f}m')
    print(f'[JSON 往返]  {"✅" if MazeMap.from_json(m.to_json()).nodes == m.nodes else "❌"}')
    print()
    print(m.render())
