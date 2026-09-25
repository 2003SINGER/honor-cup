#!/usr/bin/env python3
"""EdgeMap —— 墙几何事实的唯一 owner (规范 §2/§3).

EdgeBelief: signed score, 每帧更新(可撤销), 真迟滞状态机.
Traversal (walked) 数据也存这里, 但语义独立: walked=物理事实, 传感器矛盾只记诊断.
canonical key: 同一物理边只有一份 belief."""

import json

DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
DIRS = ('N', 'E', 'S', 'W')

UNKNOWN, WALL, OPEN = 'UNKNOWN', 'WALL', 'OPEN'
P_SENSOR, P_WALKED, P_BOUNDARY, P_TREE = 'SENSOR', 'WALKED', 'BOUNDARY', 'TREE_INFERENCE'

T_CONFIRM = 2      # UNKNOWN → 确认阈值 (2 帧一致, 或近距 1 帧满幅度)
T_FLIP = 4         # 已确认状态翻转阈值 (真实参与翻转条件)


class EdgeMap:
    def __init__(self, n):
        self.n = n
        self.beliefs = {}          # key -> {'score','state','prov'}
        self.walked = set()        # canonical keys (物理事实, R3 起由 crossed-edge 事件驱动)
        self.contradictions = 0    # 传感器 vs walked 矛盾 (诊断信号)

    # ---- canonical key ----
    def edge_key(self, c, d):
        nb = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
        if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
            return ('B', c, d)
        return (c, d) if c <= nb else (nb, OPP[d])

    def _walked(self, c, d):
        return self.edge_key(c, d) in self.walked

    # ---- 观测摄入 (每帧都更新, 已确认状态也更新 —— 可撤销的前提) ----
    def observe_wall(self, c, d, dist=1.0, confirm_near=0.6):
        return self._observe(c, d, +1, dist, confirm_near)

    def observe_open(self, c, d, dist=1.0, confirm_near=0.6):
        return self._observe(c, d, -1, dist, confirm_near)

    def _observe(self, c, d, sign, dist, confirm_near):
        key = self.edge_key(c, d)
        bel = self.beliefs.setdefault(key, {'score': 0.0, 'state': UNKNOWN, 'prov': P_SENSOR})
        if key in self.walked:
            if sign > 0:                       # 传感器说墙, 车走过 → 矛盾诊断
                self.contradictions += 1
            bel['state'], bel['score'], bel['prov'] = OPEN, -float(T_FLIP), P_WALKED
            return False
        mag = float(T_CONFIRM) if dist < confirm_near else 1.0
        bel['score'] += sign * mag
        old = bel['state']
        bel['state'] = self._transition(old, bel['score'])
        if bel['state'] != old:
            bel['prov'] = P_SENSOR
            return True
        return False

    @staticmethod
    def _transition(state, score):
        """真迟滞: 确认 ±T_CONFIRM, 翻转需越过 ±T_FLIP (通用分支不许吃掉 T_FLIP)"""
        if state == UNKNOWN:
            if score >= T_CONFIRM:
                return WALL
            if score <= -T_CONFIRM:
                return OPEN
        elif state == WALL:
            if score <= -T_FLIP:
                return OPEN
        elif state == OPEN:
            if score >= T_FLIP:
                return WALL
        return state

    # ---- walked / 推理 写入 ----
    def mark_walked(self, c, d):
        key = self.edge_key(c, d)
        if key not in self.walked:
            self.walked.add(key)
        bel = self.beliefs.setdefault(key, {'score': 0.0, 'state': UNKNOWN, 'prov': P_SENSOR})
        bel['state'], bel['score'], bel['prov'] = OPEN, -float(T_FLIP), P_WALKED

    def infer_wall(self, c, d):
        """TreeInference 结论写入 (provenance=TREE_INFERENCE)"""
        key = self.edge_key(c, d)
        if key in self.walked:
            return False                       # 推理与物理事实矛盾 → 调用方应告警
        bel = self.beliefs.setdefault(key, {'score': 0.0, 'state': UNKNOWN, 'prov': P_SENSOR})
        if bel['state'] != WALL:
            bel['state'], bel['score'], bel['prov'] = WALL, float(T_CONFIRM), P_TREE
            return True
        return False

    # ---- 查询 ----
    def state(self, c, d):
        bel = self.beliefs.get(self.edge_key(c, d))
        return bel['state'] if bel else UNKNOWN

    def provenance(self, c, d):
        bel = self.beliefs.get(self.edge_key(c, d))
        return bel['prov'] if bel else None

    def is_wall(self, c, d):
        return self.state(c, d) == WALL

    def is_open(self, c, d):
        return self.state(c, d) == OPEN

    def openings(self, c):
        """已确认 OPEN 的方向 (规范 §5: CellMark 的 openings)"""
        return tuple(d for d in DIRS if self.is_open(c, d))

    def frontier(self, c):
        """规范定义: OPEN 且未走过 (belief 撤销 → frontier 自动消失, 无 stale seen)"""
        return tuple(d for d in DIRS if self.is_open(c, d) and not self._walked(c, d))

    def resolved(self, c, d):
        return self.state(c, d) != UNKNOWN

    def cell_classified(self, c):
        return all(self.resolved(c, d) for d in DIRS)

    def all_resolved(self):
        for i in range(self.n):
            for j in range(self.n):
                for d in DIRS:
                    nb = (i + DIRV[d][0], j + DIRV[d][1])
                    if 0 <= nb[0] < self.n and 0 <= nb[1] < self.n and                              not self.resolved((i, j), d):
                        return False
        return True

    # ---- 持久化 ----
    def to_json(self):
        return json.dumps({
            'n': self.n,
            'beliefs': self._flat(),
            'walked': [self._flat_key(k) for k in self.walked],
            'contradictions': self.contradictions,
        })

    def _flat_key(self, k):
        if k[0] == 'B':
            return f"B|{k[1][0]}|{k[1][1]}|{k[2]}"
        return f"I|{k[0][0]}|{k[0][1]}|{k[1]}"

    def _parse_key(self, s):
        t, x, y, dd = s.split('|')
        return self.edge_key((int(x), int(y)), dd)

    def _flat(self):
        """canonical key → 统一 4 元组 (I:x,y,d / B:x,y,d)"""
        out = {}
        for k, v in self.beliefs.items():
            if k[0] == 'B':
                out[f"B|{k[1][0]}|{k[1][1]}|{k[2]}"] = v
            else:
                out[f"I|{k[0][0]}|{k[0][1]}|{k[1]}"] = v
        return out

    def from_json(self, s):
        d = json.loads(s)
        self.n = d['n']
        self.beliefs = {}
        for k, v in d['beliefs'].items():
            t, x, y, dd = k.split('|')
            key = self.edge_key((int(x), int(y)), dd)
            self.beliefs[key] = v
        self.walked = {self._parse_key(k) for k in d['walked']}
        self.contradictions = d.get('contradictions', 0)
        return self
