#!/usr/bin/env python3
"""TreeInference —— derived 事实重算 (规范 §4).

派生层: 每次从当前 base 事实 (soft/hard) 重算 derived closure, 不持久盖章.
前提 (SENSOR OPEN) 撤销 → 下一轮 recompute → derived WALL 自动消失.
7×7 规模 O(49×4) 全量重算, 优先简单正确."""

DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
DIRS = ('N', 'E', 'S', 'W')


def recompute_derived(edges, traversal, assume_tree=False):
    """重算 derived facts (当前仅规则 A: 成环必墙). 返回 derived dict.
    调用方负责把它放回 edges.derived (StreamNav 每帧调用)."""
    derived = {}
    # 规则 A: UNKNOWN 边两端已被 effective-OPEN 图连通 → WALL
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def is_open_edge(c, d):
        nb = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
        if not (0 <= nb[0] < edges.n and 0 <= nb[1] < edges.n):
            return False
        return edges.is_open(c, d, traversal)

    for i in range(edges.n):
        for j in range(edges.n):
            for d in DIRS:
                if is_open_edge((i, j), d):
                    a, b = find((i, j)), find((i + DIRV[d][0], j + DIRV[d][1]))
                    if a != b:
                        parent[a] = b
    for i in range(edges.n):
        for j in range(edges.n):
            for d in DIRS:
                dv = DIRV[d]
                nb = (i + dv[0], j + dv[1])
                if not (0 <= nb[0] < edges.n and 0 <= nb[1] < edges.n):
                    continue
                if edges.resolved((i, j), d, traversal):
                    continue
                if find((i, j)) == find(nb):
                    derived[edges.edge_key((i, j), d)] = 'WALL'
    # 规则 B/C/D (桥必开/唯一出口/边数): assume_tree 且赛题正式保证后实现
    return derived
