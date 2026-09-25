#!/usr/bin/env python3
"""TreeInference —— 树结构公理闭包 (规范 §4).

只读 EdgeMap, 不看雷达距离/速度/heading/DFS 顺序.
规则 A (成环必墙): 恒启用 —— UNKNOWN 边两端已被 confirmed-OPEN 图连通 → WALL.
规则 B/C/D (桥必开/唯一出口/边数): 仅 assume_tree=True 且赛题正式保证连通树时启用,
当前未实现 (flag 占位, 未经题面保证禁止引入)."""

DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
DIRS = ('N', 'E', 'S', 'W')


def infer(edges, assume_tree=False):
    """对 EdgeMap 做一轮推理闭包. 返回新确定边数."""
    newly = 0
    # 规则 A: 成环必墙 (union-find on confirmed-OPEN 图)
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for i in range(edges.n):
        for j in range(edges.n):
            for d in DIRS:
                dv = DIRV[d]
                nb = (i + dv[0], j + dv[1])
                if not (0 <= nb[0] < edges.n and 0 <= nb[1] < edges.n):
                    continue
                if edges.is_open((i, j), d):          # 含 walked (物理事实也是 OPEN)
                    a, b = find((i, j)), find(nb)
                    if a != b:
                        parent[a] = b
    for i in range(edges.n):
        for j in range(edges.n):
            for d in DIRS:
                dv = DIRV[d]
                nb = (i + dv[0], j + dv[1])
                if not (0 <= nb[0] < edges.n and 0 <= nb[1] < edges.n):
                    continue
                if edges.resolved((i, j), d):
                    continue
                if find((i, j)) == find(nb):          # 两端已连通 → OPEN 会成环 → WALL
                    if edges.infer_wall((i, j), d):
                        newly += 1
    # 规则 B/C/D: 待赛题正式保证后启用 (assume_tree flag 占位)
    return newly
