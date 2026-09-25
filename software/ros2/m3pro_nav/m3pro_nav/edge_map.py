#!/usr/bin/env python3
"""EdgeMap + TraversalMap —— 规范 §2/§3 的物理分离实现.

EdgeMap: 墙几何事实唯一 owner, 三层事实模型:
  hard    : WALKED (物理事实) / BOUNDARY (场地边界几何) —— 不可被传感器投票推翻
  soft    : SENSOR signed score (可撤销, 真迟滞 ±T_CONFIRM/±T_FLIP)
  derived : TREE_INFERENCE —— 每次从 base 事实重算 (前提撤销 → derived 自动撤销)
TraversalMap: WALKED/NOT_WALKED —— R3 起 crossed-edge 事件的唯一写入目标.

同一条物理边 canonical key 只有一行."""

import json

DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
DIRS = ('N', 'E', 'S', 'W')

UNKNOWN, WALL, OPEN = 'UNKNOWN', 'WALL', 'OPEN'
P_SENSOR, P_WALKED, P_BOUNDARY, P_TREE = 'SENSOR', 'WALKED', 'BOUNDARY', 'TREE_INFERENCE'

T_CONFIRM = 2
T_FLIP = 4


class TraversalMap:
    """WALKED/NOT_WALKED 唯一 owner (R3: crossed-edge 事件写入)."""

    def __init__(self, n):
        self.n = n
        self.walked = set()            # canonical keys

    def edge_key(self, c, d):
        nb = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
        if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
            return ('B', c, d)
        return (c, d) if c <= nb else (nb, OPP[d])

    def mark_crossed(self, c, d):
        self.walked.add(self.edge_key(c, d))

    def is_walked(self, c, d):
        return self.edge_key(c, d) in self.walked

    def neighbors_walked(self, c):
        out = []
        for d in DIRS:
            nb = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
            if self.is_walked(c, d) or \
               (0 <= nb[0] < self.n and 0 <= nb[1] < self.n and
                self.edge_key(nb, OPP[d]) in self.walked):
                out.append(d)
        return out

    def to_json(self):
        return {'walked': [self._fk(k) for k in self.walked]}

    def from_json(self, d):
        self.walked = {self._pk(k) for k in d['walked']}

    def _fk(self, k):
        if k[0] == 'B':
            return f"B|{k[1][0]}|{k[1][1]}|{k[2]}"
        return f"I|{k[0][0]}|{k[0][1]}|{k[1]}"

    def _pk(self, s):
        t, x, y, dd = s.split('|')
        return self.edge_key((int(x), int(y)), dd)


class EdgeMap:
    """墙几何事实: hard(WALKED/BOUNDARY) > derived(TREE) > soft(SENSOR)."""

    def __init__(self, n):
        self.n = n
        self.soft = {}                 # key -> {'score','state'} (SENSOR, 可撤销)
        self.hard = {}                 # key -> (state, prov)  (WALKED/BOUNDARY)
        self.derived = {}              # key -> WALL (TREE, 每帧重算)
        self.contradictions = 0

    # ---- key (TraversalMap 同构, 独立实现避免互相依赖) ----
    def edge_key(self, c, d):
        nb = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
        if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
            return ('B', c, d)
        return (c, d) if c <= nb else (nb, OPP[d])

    # ---- 观测 (soft, 每帧都更新) ----
    def observe_wall(self, c, d, dist=1.0, confirm_near=0.6):
        return self._observe(c, d, +1, dist, confirm_near)

    def observe_open(self, c, d, dist=1.0, confirm_near=0.6):
        return self._observe(c, d, -1, dist, confirm_near)

    def _observe(self, c, d, sign, dist, confirm_near):
        key = self.edge_key(c, d)
        bel = self.soft.setdefault(key, {'score': 0.0, 'state': UNKNOWN})
        mag = float(T_CONFIRM) if dist < confirm_near else 1.0
        bel['score'] += sign * mag
        old = bel['state']
        s = bel['score']
        if bel['state'] == UNKNOWN:
            if s >= T_CONFIRM:
                bel['state'] = WALL
            elif s <= -T_CONFIRM:
                bel['state'] = OPEN
        elif bel['state'] == WALL:
            if s <= -T_FLIP:
                bel['state'] = OPEN
        elif bel['state'] == OPEN:
            if s >= T_FLIP:
                bel['state'] = WALL
        return bel['state'] != old

    # ---- hard 事实 ----
    def set_boundary(self, c, d, state):
        """场地边界几何 (hard): state=WALL(普通外墙) 或 OPEN(入口/出口边)"""
        self.hard[self.edge_key(c, d)] = (state, P_BOUNDARY)

    def walked(self, c, d, traversal):
        """查询: 该边是否被走过 (hard OPEN)"""
        return traversal.is_walked(c, d)

    # ---- 查询 (effective state) ----
    def state(self, c, d, traversal=None):
        key = self.edge_key(c, d)
        if traversal is not None and traversal.is_walked(c, d):
            return OPEN                              # hard
        if key in self.hard:
            return self.hard[key][0]                 # hard (BOUNDARY)
        if key in self.derived:
            return self.derived[key]                 # derived (TREE)
        bel = self.soft.get(key)
        return bel['state'] if bel else UNKNOWN      # soft

    def provenance(self, c, d, traversal=None):
        key = self.edge_key(c, d)
        if traversal is not None and traversal.is_walked(c, d):
            return P_WALKED
        if key in self.hard:
            return self.hard[key][1]
        if key in self.derived:
            return P_TREE
        bel = self.soft.get(key)
        return bel['prov'] if 'prov' in (bel or {}) else (P_SENSOR if bel else None)

    # ---- base 查询 (hard+soft, 不含 derived) —— TreeInference 只许读这些 ----
    def base_state(self, c, d, traversal=None):
        key = self.edge_key(c, d)
        if traversal is not None and traversal.is_walked(c, d):
            return OPEN                              # hard
        if key in self.hard:
            return self.hard[key][0]                 # hard (BOUNDARY)
        bel = self.soft.get(key)
        return bel['state'] if bel else UNKNOWN      # soft (不含 derived!)

    def base_is_open(self, c, d, traversal=None):
        return self.base_state(c, d, traversal) == OPEN

    def base_resolved(self, c, d, traversal=None):
        return self.base_state(c, d, traversal) != UNKNOWN

    def is_boundary(self, c, d):
        return self.edge_key(c, d)[0] == 'B'

    def is_wall(self, c, d, traversal=None):
        return self.state(c, d, traversal) == WALL

    def is_open(self, c, d, traversal=None):
        return self.state(c, d, traversal) == OPEN

    def resolved(self, c, d, traversal=None):
        return self.state(c, d, traversal) != UNKNOWN

    def cell_classified(self, c, traversal=None):
        return all(self.resolved(c, d, traversal) for d in DIRS)

    def frontier(self, c, traversal):
        """OPEN 且未走过 (规范定义; belief 撤销 → 自动消失)"""
        return tuple(d for d in DIRS
                     if self.is_open(c, d, traversal) and not traversal.is_walked(c, d))

    def all_resolved(self, traversal=None):
        for i in range(self.n):
            for j in range(self.n):
                for d in DIRS:
                    nb = (i + DIRV[d][0], j + DIRV[d][1])
                    if 0 <= nb[0] < self.n and 0 <= nb[1] < self.n and \
                            not self.resolved((i, j), d, traversal):
                        return False
        return True

    # ---- 持久化 ----
    def _fk(self, k):
        if k[0] == 'B':
            return f"B|{k[1][0]}|{k[1][1]}|{k[2]}"
        return f"I|{k[0][0]}|{k[0][1]}|{k[1]}"

    def _pk(self, s):
        t, x, y, dd = s.split('|')
        return self.edge_key((int(x), int(y)), dd)

    def to_json(self):
        return {
            'n': self.n,
            'soft': {self._fk(k): v for k, v in self.soft.items()},
            'hard': {self._fk(k): v for k, v in self.hard.items()},
        }

    def from_json(self, d):
        self.n = d['n']
        self.soft = {self._pk(k): v for k, v in d['soft'].items()}
        self.hard = {self._pk(k): tuple(v) for k, v in d['hard'].items()}
        self.derived = {}
        return self
