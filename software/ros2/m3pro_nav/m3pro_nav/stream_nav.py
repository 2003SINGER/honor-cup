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
        self.marks = {}                         # cell -> {'kind','d2'} 格子标记器
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
        """hits[(cell,dir)]=实测距离; opens=[(cell,dir,dist)] 确认开口(带距离).
           节点层负责把 /scan 转成这两个字典 (几何反算), 本类做置信门限.
           开口同样受掠射门限约束: 远处斜向的'开口'不可信, 靠近再确认."""
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
                nbm = (ck[0] + DIRV[axis][0], ck[1] + DIRV[axis][1])
                if 0 <= nbm[0] < self.n and 0 <= nbm[1] < self.n:
                    self.wall_known[(nbm, OPP[axis])] = True   # 边共享镜像
                newly += 1
        for (ck, axis, dist) in opens:
            if self.resolved(ck, axis):
                continue
            if dist < 0.25:                          # 正入射/贴身: 直接确认
                err = 0.02
            else:                                    # 对角掠射开口: 远才可信
                phi = math.atan2(P_HALF, dist)
                err = max(0.02, P_HALF / math.sin(phi) ** 2 * self.DPHI)
            if err < self.gate:
                nb = self.m.open_edge(ck, axis)
                if nb is None:
                    self.exit_open.add((ck, axis))
                    self.exit_cell = ck if self.exit_cell is None else self.exit_cell
                newly += 1
        return newly

    # ---- 认知一致性守卫 ----

    def consistency_guard(self):
        """walked 边是物理事实(车真走过), 推断墙与之冲突 → 删墙并作废相关标记.
           防'墙+walked并存'矛盾认知导致决策死循环 (仿真实测教训)."""
        n = 0
        for cc, nd in self.m.nodes.items():
            for d, s in nd['edges'].items():
                if s == 'walked' and (cc, d) in self.wall_known:
                    del self.wall_known[(cc, d)]
                    nbm = (cc[0] + DIRV[d][0], cc[1] + DIRV[d][1])
                    self.wall_known.pop((nbm, OPP[d]), None)
                    self.marks.pop(cc, None)
                    self.marks.pop(nbm, None)
                    n += 1
        return n

    # ---- 格子标记器 (纯局部观测: 四边登记齐 → 直接打"怎么走"标记) ----

    def try_mark(self, c):
        if c != self.entry and c not in self.m.nodes:
            return
        if any(not self.resolved(c, d) for d in DIRV):
            return                                   # 还有未知边: 靠近再说
        opens = [d for d in DIRV if (c, d) not in self.wall_known]
        n = len(opens)
        new_mk = ({'kind': 'dead', 'd2': None} if n == 0 else
                  {'kind': 'way', 'd2': opens[0]} if n == 1 else
                  {'kind': 'branch', 'd2': None})
        self.marks[c] = new_mk

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
        """标记驱动决策: 非岔路格读标记直接动(零推理); 岔路才进搜索层.
           'wait'  = 本格无标记 (四边未齐: 节点层减速靠近, 到置信范围即打标)
           'home'  = 探索完 (收齐/全图) → 上层切速度跑回出口"""
        mk = self.marks.get(cell)
        if mk is None:
            return 'wait'
        if mk['kind'] == 'way':
            d2 = mk['d2']                            # 唯一出口: 标记即答案
        else:                                        # branch
            front = self.front(cell)
            # dead 方向跳过 — 节点层可用 has_block(far) 例外(死路抓块必须进)
            def farc_of(d):
                return (cell[0] + DIRV[d][0], cell[1] + DIRV[d][1])
            front = [d for d in front
                     if self.marks.get(farc_of(d), {}).get('kind') != 'dead']
            if front:
                d2 = self.choose(front, heading)
            else:
                nf = self.nearest_frontier(cell)     # 回溯: BFS 最近未探分支
                if nf is None:
                    return 'home'
                d2 = nf[1]
        far = (cell[0] + DIRV[d2][0], cell[1] + DIRV[d2][1])
        turn_here = None if d2 == heading else ('arc' if d2 != OPP[heading] else 'rev')
        # peek far 转弯方式: 优先读标记; branch 用决策层预演
        end_o, v_end, d3, far_arc = 0.4, self.v_cruise, None, False
        mfar = self.marks.get(far)
        if mfar and mfar['kind'] == 'dead':
            v_end = 0.0
        elif mfar and mfar['kind'] == 'way':
            d3 = mfar['d2']
            if d3 != d2 and d3 != OPP[d2]:
                far_arc = True
                end_o, v_end = 0.2, self.v_arc
        elif self.cell_classified(far):
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
