#!/usr/bin/env python3
"""TreeInference —— derived 事实纯重算 (规范 §4; GPT 四审 §8: 只读 base facts).

derived_new = f(base facts)  —— 纯函数, 不读旧 derived (无自举推理).
前提 (SENSOR OPEN) 撤销 → 下轮 recompute → derived WALL 自动消失.
本项目赛题官方保证迷宫为树 → 规则 A (成环必墙) 恒成立, 不设虚假可配置项.
规则 B/C/D (桥必开等) 未经题面保证, 禁止引入."""

DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
DIRS = ('N', 'E', 'S', 'W')


def recompute_derived(edges, traversal):
    """纯重算 derived (当前仅规则 A: 成环必墙). 只读 base facts."""
    derived = {}
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def base_open(c, d):
        nb = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
        if not (0 <= nb[0] < edges.n and 0 <= nb[1] < edges.n):
            return False
        return edges.base_is_open(c, d, traversal)   # base only!

    # confirmed-OPEN 图 (base) 建 union-find
    for i in range(edges.n):
        for j in range(edges.n):
            for d in DIRS:
                if base_open((i, j), d):
                    a, b = find((i, j)), find((i + DIRV[d][0], j + DIRV[d][1]))
                    if a != b:
                        parent[a] = b
    # UNKNOWN (base) 边两端已连通 → 成环 → WALL
    for i in range(edges.n):
        for j in range(edges.n):
            for d in DIRS:
                dv = DIRV[d]
                nb = (i + dv[0], j + dv[1])
                if not (0 <= nb[0] < edges.n and 0 <= nb[1] < edges.n):
                    continue
                if edges.base_resolved((i, j), d, traversal):
                    continue
                if find((i, j)) == find(nb):
                    derived[edges.edge_key((i, j), d)] = 'WALL'
    return derived
