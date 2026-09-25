#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StreamNav —— 流式探索决策核心 (上车版; 与仿真 explore_stream 同构, 已验证)

分层: 决策/建图在这里; 传感器(雷达原始数据)与运动执行在 ROS 节点层.
世界/小车分离原则: 本类只消费 sense 数据, 永不接触真值地图.

上车接线 (decision_node):
  雷达 /scan → 节点层转成 (hits, opens) 喂 ingest()
    hits[(cell,dir)] = 实测距离(带噪), opens[(cell,dir)] = 确认开口
    (cell/dir 由当前定位+已知格线几何反算; 置信门限在本类内做)
  定位 → cell / o (格+边内偏移, 来自 里程计+雷达吸附)
  本类输出 plan (d2, end_o, v_end, ...) → tracker / cmd_vel

置信模型 (T-mini Plus 实测 ±20mm 全量程恒定):
  正入射: err ≈ σ (远也准)
  掠射:   err ≈ P/sin²φ·δφ, P=0.2m 半通道, φ=atan(P/s) —— 远才可信
  d_conf ≈ 侧向 3~4 格 / 正前更远 (实测后回填 δφ/gate)

状态: 核心算法已在 30 种子仿真验证 (违规 0, 平均速 0.36-0.38 m/s)。
"""

import math

C = 0.4
P_HALF = 0.2
DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}

from .mazemap import MazeMap


class StreamNav:
    def __init__(self, entry, order='LFR', n=7,
                 v_cruise=0.70, a_acc=1.0, a_dec=1.0, a_lat=0.7,
                 dphi_deg=0.30, gate=0.06):
        self.n = n
        self.entry = entry
        self.order = order
        self.v_cruise = v_cruise
        self.a_acc, self.a_dec, self.a_lat = a_acc, a_dec, a_lat
        self.DPHI = math.radians(dphi_deg)
        self.gate = gate
        self.v_arc = math.sqrt(a_lat * P_HALF)

        self.m = MazeMap(n, entry)              # 认知地图
        self.wall_known = {}                    # (cell,dir) -> 已确认的墙
        self.exit_open = set()                  # (cell,dir) -> 已确认出口开口
        self.got = 0
        self.exit_cell = None

    # ---- 认知查询 ----

    def resolved(self, c, d):
        return (c, d) in self.wall_known or (c, d) in self.exit_open or \
               self.m.nodes.get(c, {}).get('edges', {}).get(d) in ('seen', 'walked')

    def cell_classified(self, c):
        """本格四边是否全部确认 (流式: 分支没扫到 ≠ 不存在, 由节点层保证进格前扫完)"""
        return all(self.resolved(c, d) for d in DIRV)

    def front(self, c):
        return [d for d in self.m.frontier(c)]

    # ---- 观测摄入 (唯一入口; 只信数据, 不问真值) ----

    def ingest(self, hits, opens):
        """hits[(cell,dir)]=实测距离; opens=[(cell,dir)] 确认开口.
           节点层负责把 /scan 转成这两个字典 (几何反算), 本类做置信门限."""
        newly = 0
        for (ck, axis), d_meas in hits.items():
            if self.resolved(ck, axis):
                continue
            if d_meas > 0.25:                        # 掠射段
                phi = math.atan2(P_HALF, max(d_meas, 0.21))
                err = max(0.02, P_HALF / math.sin(phi) ** 2 * self.DPHI)
            else:                                    # 正入射段
                err = math.hypot(0.02, d_meas * self.DPHI)
            if err < self.gate:
                self.wall_known[(ck, axis)] = True
                newly += 1
        for (ck, axis) in opens:
            if self.resolved(ck, axis):
                continue
            if 0.02 < self.gate:                     # 开口由正入射轴扫描给出
                nb = self.m.open_edge(ck, axis)
                if nb is None:
                    self.exit_open.add((ck, axis))
                    self.exit_cell = ck if self.exit_cell is None else self.exit_cell
                newly += 1
        return newly

    # ---- 决策 ----

    def nearest_frontier(self, cell):
        """BFS 已走边图 → 最近的有 seen 分支的格 (第一步方向, 步数) 或 None"""
        reach = {cell: (0, None)}
        q = [cell]
        while q:
            cc = q.pop(0)
            if cc != cell and self.m.frontier(cc):
                return reach[cc]
            for d, dv in DIRV.items():
                if self.m.nodes.get(cc, {}).get('edges', {}).get(d) != 'walked':
                    continue
                nb = (cc[0] + dv[0], cc[1] + dv[1])
                if nb not in reach:
                    reach[nb] = (reach[cc][0] + 1, d if cc == cell else reach[cc][1])
                    q.append(nb)
        return None

    def rel_of(self, d, heading):
        m = {heading: 'F', OPP[heading]: 'B'}
        m[{'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[heading]] = 'R'
        m[{'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[heading]] = 'L'
        return m[d]

    def choose(self, front, heading):
        return sorted(front, key=lambda dd: self.order.index(self.rel_of(dd, heading))
                      if self.rel_of(dd, heading) in self.order else 99)[0]

    def plan_edge(self, cell, heading):
        """格心决策 → 本边 plan dict.
           'wait'  = 本格未确认完 (节点层应减速就地进行 360° 确认)
           'home'  = 探索完 (收齐/全图) → 上层切速度跑回出口"""
        if not self.cell_classified(cell):
            return 'wait'
        front = self.front(cell)
        # 数墙剪枝: 远格三面墙已确认 + 本边开口 → 死路格, 无需进入即完结
        # (不需要"迷宫是树"假设, 有环场地也安全)
        def is_dead_end(d2):
            far = (cell[0] + DIRV[d2][0], cell[1] + DIRV[d2][1])
            w = sum(1 for d in DIRV if d != d2 and (far, d) in self.wall_known)
            return w == 3
        front = [d for d in front if not is_dead_end(d)]
        if front:
            d2 = self.choose(front, heading)
        else:
            nf = self.nearest_frontier(cell)
            if nf is None:
                return 'home'
            d2 = nf[1]
        far = (cell[0] + DIRV[d2][0], cell[1] + DIRV[d2][1])
        turn_here = None if d2 == heading else ('arc' if d2 != OPP[heading] else 'rev')
        # far 格转弯 → 本边 end_o / v_end (peek: 提前知道要弯就提前降速)
        end_o, v_end, d3, far_arc = 0.4, self.v_cruise, None, False
        if self.cell_classified(far):
            f_far = self.front(far)
            if f_far:
                d3 = self.choose(f_far, d2)
                if d3 != d2 and d3 != OPP[d2]:
                    far_arc = True
                    end_o, v_end = 0.2, self.v_arc
            else:
                v_end = 0.0
        return {'d2': d2, 'far': far, 'end_o': end_o, 'v_end': v_end,
                'turn_here': turn_here, 'd3': d3, 'far_arc': far_arc}

    def v_cap(self, plan, o):
        """视界调速: far 未确认 → 保守停车上界; 已确认 → 按 end_o/v_end 几何刹车"""
        if not self.cell_classified(plan['far']):
            return math.sqrt(max(0.0, 2 * self.a_dec * max(0.0, C - o - 0.05)))
        return math.sqrt(max(0.0, plan['v_end'] ** 2 +
                             2 * self.a_dec * max(0.0, plan['end_o'] - o)))

    # ---- 地图持久化 ----

    def to_json(self):
        import json
        return json.dumps({'m': self.m.to_json(),
                           'wall_known': [list(k) for k in self.wall_known],
                           'exit_open': [list(k) for k in self.exit_open]})

    def from_json(self, s):
        import json
        d = json.loads(s)
        self.m.from_json(d['m'])
        self.wall_known = {tuple(k): True for k in d['wall_known']}
        self.exit_open = {tuple(k) for k in d['exit_open']}
