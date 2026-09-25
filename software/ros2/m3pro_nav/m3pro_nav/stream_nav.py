#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StreamNav —— 流式探索决策核心 (唯一认知体; 仿真与 ROS 节点共用同一份, 禁止复制)

分层契约:
  节点层(仿真=World / 实车=ROS) → observe(): (cell,dir,dist,incidence) 观测
  本类: 证据累计(多帧确认/冲突撤销) → 格子标记 → 标记驱动决策 → plan
  运动执行在节点层 (tracker / cmd_vel)

观测几何 (入射角置信模型, 用户定义):
  err = max(σ_range, D·δα/cos²α)
    α  = 射线方向与离散墙法线的夹角 (0=正入射, →90°=掠射) —— 观测方按几何算出
    D  = 传感器到墙线的垂直距离 = dist·cos α
    δα = 雷达角分辨率噪声 (T-mini Plus ~0.3°)
  正入射再远误差也小 (2.4m×0.3°≈1.3cm); 掠射随 cos²α 急剧放大 —— 远才不可信的是掠射, 不是距离.

证据状态机 (防 first-observation-wins):
  UNKNOWN → (1 帧过门限) TENTATIVE → (第 2 帧一致 或 近距离<0.6m) CONFIRMED
  冲突证据 (墙证据遇开口证据或反之) → 计数清零重来; walked 是物理事实, 永远覆盖墙推断.

格子标记 (数墙=分类器, 纯局部零推理):
  degree = 开口边数; 1→dead(唯一开口即来路,掉头), 2→way(opens 存表,前进方向由来向解析),
  ≥3→branch(进树节点决策). way/dead 分支可被跳过除非 has_block.

上车接线 (decision_node):
  /scan + 定位 → 节点层几何反算 (cell,dir,dist,α) → observe()
  相机方块检测 → set_block_seen()
  plan_edge() → tracker → cmd_vel
"""

import math
import json

from .mazemap import MazeMap

C = 0.4
P_HALF = 0.2
DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
COS2_FLOOR = 1e-3                                  # cos²α 下限 (α→90° 数值防爆)


class StreamNav:
    def __init__(self, entry, order='LFR', n=7,
                 v_cruise=0.70, a_acc=1.0, a_dec=1.0, a_lat=0.7,
                 dphi_deg=0.30, gate=0.06, rng=0.02, confirm_near=0.6,
                 assume_tree=True):
        self.n = n
        self.entry = entry
        self.order = order
        self.v_cruise = v_cruise
        self.a_acc, self.a_dec, self.a_lat = a_acc, a_dec, a_lat
        self.DPHI = math.radians(dphi_deg)
        self.gate = gate
        self.rng = rng
        self.confirm_near = confirm_near
        self.assume_tree = assume_tree
        self.v_arc = math.sqrt(a_lat * P_HALF)

        self.m = MazeMap(n, entry)                  # 认知地图
        self.wall_known = {}                        # (cell,dir) -> True (tentative 或 confirmed)
        self.wall_conf = set()                      # 已 CONFIRMED 的墙 (2帧/近距)
        self.evidence = {}                          # (cell,dir) -> {'wall':n,'open':n}
        self.exit_open = set()                      # (cell,dir) -> 已确认出口开口
        self.marks = {}                             # cell -> {'kind','opens'}
        self.blocks_seen = set()                    # 相机报告的含块格 (视野受限)
        self.blocks_gone = set()                    # 已抓
        self.got = 0
        self.exit_cell = None

    # ---- 认知查询 ----

    def is_wall(self, c, d):
        return (c, d) in self.wall_known

    def resolved(self, c, d):
        return self.is_wall(c, d) or (c, d) in self.exit_open or \
               self.m.nodes.get(c, {}).get('edges', {}).get(d) in ('seen', 'walked')

    def cell_classified(self, c):
        return all(self.resolved(c, d) for d in DIRV)

    def front(self, c):
        return [d for d in self.m.frontier(c)]

    def has_block(self, c):
        """相机视角: 该格是否有未收集方块 (节点层喂 set_block_seen, 视野受限非上帝)"""
        return c in self.blocks_seen and c not in self.blocks_gone

    def set_block_seen(self, c, seen):
        (self.blocks_seen.add if seen else self.blocks_seen.discard)(c)

    def set_block_collected(self, c):
        self.blocks_gone.add(c)
        self.blocks_seen.discard(c)

    # ---- 证据与置信 ----

    def _err(self, dist, alpha):
        """入射角置信模型: 沿墙定位误差 = D·δα/cos²α, D=dist·cosα (α=射线与墙法线夹角)"""
        a = min(abs(alpha), math.radians(89.9))
        D = max(0.05, dist * math.cos(a))
        return max(self.rng, D * self.DPHI / max(math.cos(a) ** 2, COS2_FLOOR))

    def _confirm_ok(self, count, dist):
        return count >= 2 or dist < self.confirm_near

    def observe(self, hits, opens):
        """每帧入口: 证据累计(多帧确认/冲突撤销) + 剪枝 + 一致性守卫 + 打标.
           hits[(cell,dir)] = (dist, alpha);  opens[(cell,dir)] = (dist, alpha)"""
        newly = 0
        for (ck, axis), (dist, alpha) in hits.items():
            if self.resolved(ck, axis):
                continue
            if self._err(dist, alpha) >= self.gate:
                continue
            ev = self.evidence.setdefault((ck, axis), {'wall': 0, 'open': 0})
            if ev['open'] > 0:                       # 冲突: 推翻 open 证据, 重计
                ev['open'] = 0
                self.m.nodes.get(ck, {}).get('edges', {}).pop(axis, None)
            ev['wall'] += 1
            if self._confirm_ok(ev['wall'], dist):
                self.wall_known[(ck, axis)] = True
                if dist < self.confirm_near:
                    self.wall_conf.add((ck, axis))
                nbm = (ck[0] + DIRV[axis][0], ck[1] + DIRV[axis][1])
                if 0 <= nbm[0] < self.n and 0 <= nbm[1] < self.n:
                    self.wall_known[(nbm, OPP[axis])] = True   # 边共享镜像
                newly += 1
        for (ck, axis, dist, alpha) in opens:
            if self.resolved(ck, axis):
                continue
            if self._err(dist, alpha) >= self.gate:
                continue
            ev = self.evidence.setdefault((ck, axis), {'wall': 0, 'open': 0})
            if ev['wall'] > 0:                       # 冲突: 推翻未确认 wall 证据
                ev['wall'] = 0
                self.wall_known.pop((ck, axis), None)
                nbm = (ck[0] + DIRV[axis][0], ck[1] + DIRV[axis][1])
                if not self._confirmed_wall(ck, axis):
                    self.wall_known.pop((nbm, OPP[axis]), None)
            ev['open'] += 1
            if self._confirm_ok(ev['open'], dist):
                nb = self.m.open_edge(ck, axis)
                if nb is None:
                    self.exit_open.add((ck, axis))
                    if self.exit_cell is None and ck != self.entry:
                        self.exit_cell = ck             # 入口外来向不是出口
                newly += 1
        self.prune()
        self.consistency_guard()
        for c2 in list(self.m.nodes):
            self.try_mark(c2)
        self.try_mark(self.entry)
        return newly

    def _confirmed_wall(self, ck, axis):
        return (ck, axis) in self.wall_conf

    # ---- 认知一致性守卫 ----

    def consistency_guard(self):
        """walked 边是物理事实(车真走过), 推断墙与之冲突 → 删墙作废标记."""
        n = 0
        for cc, nd in self.m.nodes.items():
            for d, s in nd['edges'].items():
                if s == 'walked' and (cc, d) in self.wall_known:
                    del self.wall_known[(cc, d)]
                    self.evidence.pop((cc, d), None)
                    nbm = (cc[0] + DIRV[d][0], cc[1] + DIRV[d][1])
                    self.wall_known.pop((nbm, OPP[d]), None)
                    self.marks.pop(cc, None)
                    self.marks.pop(nbm, None)
                    n += 1
        return n

    def prune(self):
        """树环剪枝: 未知边两端点在已确认通道图已连通 → 必是墙 (加边即成环).
           仅 assume_tree=True (树形迷宫) 启用; 有环场地必须关闭."""
        newly = 0
        for (ck, axis) in list(self.wall_known):
            nbm = (ck[0] + DIRV[axis][0], ck[1] + DIRV[axis][1])
            if 0 <= nbm[0] < self.n and 0 <= nbm[1] < self.n and \
                    (nbm, OPP[axis]) not in self.wall_known:
                self.wall_known[(nbm, OPP[axis])] = True
                newly += 1
        if not self.assume_tree:
            return newly
        parent = {}

        def find(x):
            parent.setdefault(x, x)
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for c2, nd in self.m.nodes.items():
            for d, s in nd['edges'].items():
                if s not in ('seen', 'walked'):
                    continue
                a, b = find(c2), find((c2[0] + DIRV[d][0], c2[1] + DIRV[d][1]))
                if a != b:
                    parent[a] = b
        for i in range(self.n):
            for j in range(self.n):
                for d, dv in DIRV.items():
                    nb = (i + dv[0], j + dv[1])
                    if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
                        continue
                    ck = (i, j)
                    if self.resolved(ck, d):
                        continue
                    if find(ck) == find(nb):
                        self.wall_known[(ck, d)] = True
                        self.wall_known[(nb, OPP[d])] = True
                        newly += 1
        return newly

    # ---- 格子标记器 (度语义: 1=dead, 2=way, ≥3=branch) ----

    def try_mark(self, c):
        if c != self.entry and c not in self.m.nodes:
            return
        if any(not self.resolved(c, d) for d in DIRV):
            return                                   # 还有未知边: 靠近再说
        opens = tuple(d for d in DIRV if not self.is_wall(c, d))
        n = len(opens)
        new_mk = ({'kind': 'dead', 'opens': opens} if n <= 1 else
                  {'kind': 'way', 'opens': opens} if n == 2 else
                  {'kind': 'branch', 'opens': opens})
        self.marks[c] = new_mk

    def way_forward(self, c):
        """way 格(2 开口)的前进方向 = 唯一未走过的开口.
           不依赖 heading 推来向 —— 弯完成后 heading≠来向, 用 walked 状态才可靠.
           返回 None = 全走过(回溯)或来向不明(重访) → 调用方交给 BFS 导航"""
        mk = self.marks.get(c)
        if not mk or mk['kind'] != 'way':
            return None
        nd = self.m.nodes.get(c, {}).get('edges', {})
        fwd = [d for d in mk['opens']
               if nd.get(d) != 'walked' and (c, d) not in self.exit_open]
        return fwd[0] if len(fwd) == 1 else None

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
        """标记驱动决策: way 读标记直接动; branch 才进搜索层.
           'wait' = 本格无标记 (节点层减速靠近); 'home' = 探索完"""
        mk = self.marks.get(cell)
        if mk is None:
            return 'wait'
        if mk['kind'] == 'way':
            d2 = self.way_forward(cell)
            if d2 is None:
                front = self.front(cell)
                if front:                            # 重访但本格有未消费分支
                    d2 = self.choose(front, heading)
                else:
                    nf = self.nearest_frontier(cell)  # 全走过 → BFS 回溯
                    if nf is None:
                        return 'home'
                    d2 = nf[1]
        else:                                            # branch
            front = self.front(cell)
            def farc_of(d):
                return (cell[0] + DIRV[d][0], cell[1] + DIRV[d][1])
            front = [d for d in front
                     if self.marks.get(farc_of(d), {}).get('kind') != 'dead'
                     or self.has_block(farc_of(d))]      # 死路有方块必须进
            if front:
                d2 = self.choose(front, heading)
            else:
                nf = self.nearest_frontier(cell)
                if nf is None:
                    return 'home'
                d2 = nf[1]
        far = (cell[0] + DIRV[d2][0], cell[1] + DIRV[d2][1])
        turn_here = None if d2 == heading else ('cut' if d2 != OPP[heading] else 'rev')
        # peek far 转弯方式: 优先读标记; branch 用决策层预演
        # 转弯一律 45° 斜切 (cut): 入弯点 = 走廊交点前 0.15 (end_o=0.25), 全速直穿;
        # 斜线距内角点 0.177m, 车侧缘余量 6.9cm —— 
        # r=0.2 圆弧对 29cm 车头会探过内角墙 3.7cm (碰撞检查实测), 斜切无此问题
        end_o, v_end, d3, far_cut = 0.4, self.v_cruise, None, False
        mfar = self.marks.get(far)
        if mfar and mfar['kind'] == 'dead':
            v_end = 0.0
        elif mfar and mfar['kind'] == 'way':
            d3 = self.way_forward(far)
            if d3 is not None and d3 != d2 and d3 != OPP[d2]:
                far_cut = True
                end_o, v_end = 0.25, self.v_cruise
        elif self.cell_classified(far):
            f_far = self.front(far)
            if f_far:
                d3 = self.choose(f_far, d2)
                if d3 != d2 and d3 != OPP[d2]:
                    far_cut = True
                    end_o, v_end = 0.25, self.v_cruise
            else:
                v_end = 0.0
        if turn_here == 'cut':
            # 格心决策出的垂直转向: 切弯来不及(入弯点在来向边), 原地转 90°.
            # 切弯只属于 far_cut —— 上一边 peek 提前知道转弯时才可切. 这是对的激励:
            # 观测越早打标, 越多弯能切.
            turn_here = 'spin90'
        return {'d2': d2, 'far': far, 'end_o': end_o, 'v_end': v_end,
                'turn_here': turn_here, 'd3': d3, 'far_cut': far_cut}

    def v_cap(self, plan, o):
        """视界保守上界: 远格未打标 → v ≤ √(2a·(d_stop))"""
        if plan in ('wait', 'home', None):
            return 0.0
        far = plan['far']
        if self.marks.get(far) is None and not self.cell_classified(far):
            return math.sqrt(max(0.0, 2 * self.a_dec * max(0.0, 0.4 - o - 0.05)))
        return plan['v_end']

    # ---- 持久化 ----

    def to_json(self):
        return json.dumps({
            'entry': self.entry, 'order': self.order, 'n': self.n,
            'm': self.m.to_json(),
            'wall_known': [[list(k)] for k in self.wall_known],
            'wall_conf': [[list(k)] for k in self.wall_conf],
            'exit_open': [[list(k)] for k in self.exit_open],
            'marks': {f'{k[0]},{k[1]}': v for k, v in self.marks.items()},
            'blocks_seen': [list(b) for b in self.blocks_seen],
            'blocks_gone': [list(b) for b in self.blocks_gone],
            'got': self.got, 'exit_cell': self.exit_cell,
        })

    def from_json(self, s):
        d = json.loads(s)
        self.m = MazeMap.from_json(d['m'])           # 修复: 必须接住返回值
        self.wall_known = {tuple(k[0]): True for k in d['wall_known']}
        self.wall_conf = {tuple(k[0]) for k in d['wall_conf']}
        self.exit_open = {tuple(k[0]) for k in d['exit_open']}
        self.marks = {tuple(int(x) for x in k.split(',')): v
                      for k, v in d['marks'].items()}
        for v in self.marks.values():
            v['opens'] = tuple(v['opens'])
        self.blocks_seen = {tuple(b) for b in d['blocks_seen']}
        self.blocks_gone = {tuple(b) for b in d['blocks_gone']}
        self.got = d['got']
        self.exit_cell = tuple(d['exit_cell']) if d['exit_cell'] else None
        return self
