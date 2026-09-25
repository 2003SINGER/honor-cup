#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StreamNav —— 流式探索决策核心 (唯一认知体; 仿真与 ROS 共用, 禁止复制)

分层契约:
  节点层(仿真=World / 实车=ROS) → observe(): (cell,dir,dist,α) 观测
  本类: EdgeBelief(signed score, 可撤销) → 格子标记 → 标记驱动决策 → plan
  运动执行在节点层 (tracker / cmd_vel)

入射角置信模型 (用户定义):
  err = max(σ_range, D·δα/cos²α), α=射线与离散墙法线夹角, D=dist·cosα
  正入射再远误差也小; 掠射随 cos²α 放大 —— 远不可信的是掠射, 不是距离.

证据层 (belief 与拓扑状态分离, 可撤销):
  EdgeBelief.score: 墙证据 +1 / 开口证据 -1 (仅过 α 门限的观测计入, 每帧都更新)
  state: score ≥ +T_wall → WALL; ≤ -T_open → OPEN; 其间 UNKNOWN (迟滞, 翻转需越过 ±2T)
  walked = 物理事实 → 永远 OPEN; 传感器与之矛盾 → 计入 contradictions (诊断信号:
  说明定位/关联/时间同步出错), 不删 walked.
  边按 canonical key 存储: 同一物理边只有一份 belief (消灭 A:E=WALL B:W=OPEN).

topology 与 navigation 分离 (GPT 审计 P0-C):
  exit_given_entry(c, entry_side)   —— 纯拓扑: 已知从 entry_side 进, 唯一出口? (far peek 用)
  navigation_successor(c, heading)  —— 导航: 本格 way 的下一步 (walked 过滤+回退)
"""

import math
import json

from .mazemap import MazeMap

C = 0.4
P_HALF = 0.2
DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
COS2_FLOOR = 1e-3

T_CONFIRM = 2          # |score| 达到即确认 (2 帧一致, 或近距 1 帧直接给满)
T_FLIP = 4             # 已确认状态被翻转需要反向证据推进 score 越过 ±4 (迟滞)
SCORE_NEAR = 1.0       # 近距离观测直接给满幅度


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

        self.m = MazeMap(n, entry)
        self.beliefs = {}                       # edge_key -> {'score':float,'state':str}
        self.exit_open = set()                  # (cell,dir) 已确认场外开口(入口/出口)
        self.marks = {}                         # cell -> {'kind','opens'}
        self.blocks_seen = set()
        self.blocks_gone = set()
        self.got = 0
        self.exit_cell = None
        self.contradictions = 0                 # 传感器与 walked 矛盾计数 (诊断信号)

    # ---- canonical 边 key: 同一物理边只有一份 belief ----

    def edge_key(self, c, d):
        nb = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
        if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
            return ('B', c, d)                  # 边界边 (场外)
        return (c, d) if c <= nb else (nb, OPP[d])

    def _walked(self, c, d):
        """该物理边是否被车走过 (任一视角)"""
        nb = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
        nd = self.m.nodes.get(c, {}).get('edges', {})
        if nd.get(d) == 'walked':
            return True
        if 0 <= nb[0] < self.n and 0 <= nb[1] < self.n:
            if self.m.nodes.get(nb, {}).get('edges', {}).get(OPP[d]) == 'walked':
                return True
        return False

    # ---- 认知查询 ----

    def is_wall(self, c, d):
        bel = self.beliefs.get(self.edge_key(c, d))
        return bel is not None and bel['state'] == 'WALL'

    def is_open(self, c, d):
        return (c, d) in self.exit_open or \
               self.m.nodes.get(c, {}).get('edges', {}).get(d) in ('seen', 'walked')

    def resolved(self, c, d):
        return self.is_wall(c, d) or self.is_open(c, d)

    def cell_classified(self, c):
        return all(self.resolved(c, d) for d in DIRV)

    def front(self, c):
        return [d for d in self.m.frontier(c)]

    def has_block(self, c):
        return c in self.blocks_seen and c not in self.blocks_gone

    def set_block_seen(self, c, seen):
        (self.blocks_seen.add if seen else self.blocks_seen.discard)(c)

    def set_block_collected(self, c):
        self.blocks_gone.add(c)
        self.blocks_seen.discard(c)

    # ---- 证据与置信 ----

    def _err(self, dist, alpha):
        a = min(abs(alpha), math.radians(89.9))
        D = max(0.05, dist * math.cos(a))
        return max(self.rng, D * self.DPHI / max(math.cos(a) ** 2, COS2_FLOOR))

    def observe(self, hits, opens):
        """每帧入口: 逐观测更新 belief (resolved 也更新! 可撤销), 再剪枝/守卫/打标.
           hits[(cell,dir)]=(dist,α); opens[(cell,dir,dist,α)]"""
        newly = 0
        for (ck, axis), (dist, alpha) in hits.items():
            if self._err(dist, alpha) >= self.gate:
                continue
            newly += self._evidence(ck, axis, +1, dist)
        for (ck, axis, dist, alpha) in opens:
            if self._err(dist, alpha) >= self.gate:
                continue
            newly += self._evidence(ck, axis, -1, dist)
        self.prune()
        self.consistency_guard()
        for c2 in list(self.m.nodes):
            self.try_mark(c2)
        self.try_mark(self.entry)
        return newly

    def _evidence(self, ck, axis, sign, dist):
        """单条观测 → belief 更新. sign=+1 墙, -1 开口. 返回 newly"""
        newly = 0
        key = self.edge_key(ck, axis)
        bel = self.beliefs.setdefault(key, {'score': 0.0, 'state': 'UNKNOWN'})
        walked = self._walked(ck, axis)
        if walked:
            if sign > 0:                          # 传感器说墙, 车走过 = 矛盾 → 诊断信号
                self.contradictions += 1
            bel['score'] = -float(T_FLIP)         # 永远 OPEN
            bel['state'] = 'OPEN'
            return newly
        mag = float(T_CONFIRM) if dist < self.confirm_near else 1.0  # 近距 1 帧直接确认
        old = bel['state']
        bel['score'] += sign * mag
        s = bel['score']
        # 迟滞: 确认容易翻转难
        if bel['state'] != 'WALL' and s >= T_CONFIRM:
            bel['state'] = 'WALL'
        elif bel['state'] != 'OPEN' and s <= -T_CONFIRM:
            bel['state'] = 'OPEN'
        elif bel['state'] == 'WALL' and s < T_CONFIRM - T_FLIP:
            bel['state'] = 'UNKNOWN' if s > -T_CONFIRM else 'OPEN'
        elif bel['state'] == 'OPEN' and s > -T_CONFIRM + T_FLIP:
            bel['state'] = 'UNKNOWN' if s < T_CONFIRM else 'WALL'
        if bel['state'] != old:
            newly += 1
            if bel['state'] == 'OPEN':
                nb = self.m.open_edge(ck, axis)
                if nb is None:
                    self.exit_open.add((ck, axis))
                    if self.exit_cell is None and ck != self.entry:
                        self.exit_cell = ck
        return newly

    # ---- 一致性守卫 ----

    def consistency_guard(self):
        """walked 边 belief 强制 OPEN; 切换确认状态后作废相关标记."""
        n = 0
        for cc, nd in self.m.nodes.items():
            for d, s in nd['edges'].items():
                if s != 'walked':
                    continue
                key = self.edge_key(cc, d)
                bel = self.beliefs.get(key)
                if bel is None or bel['state'] != 'OPEN':
                    self.beliefs[key] = {'score': -float(T_FLIP), 'state': 'OPEN'}
                    self.marks.pop(cc, None)
                    nbm = (cc[0] + DIRV[d][0], cc[1] + DIRV[d][1])
                    self.marks.pop(nbm, None)
                    n += 1
        return n

    def prune(self):
        """树环剪枝: 未知边两端点在已确认通道图已连通 → 必是墙 (仅 assume_tree)."""
        newly = 0
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
                        key = self.edge_key(ck, d)
                        self.beliefs[key] = {'score': float(T_CONFIRM), 'state': 'WALL'}
                        newly += 1
        return newly

    # ---- 格子标记器 (degree 语义: 1=dead, 2=way, ≥3=branch) ----

    def try_mark(self, c):
        if c != self.entry and c not in self.m.nodes:
            return
        if any(not self.resolved(c, d) for d in DIRV):
            return
        opens = tuple(d for d in DIRV if not self.is_wall(c, d))
        n = len(opens)
        self.marks[c] = ({'kind': 'dead', 'opens': opens} if n <= 1 else
                         {'kind': 'way', 'opens': opens} if n == 2 else
                         {'kind': 'branch', 'opens': opens})

    # ---- topology 与 navigation 分离 (GPT P0-C) ----

    def exit_given_entry(self, c, entry_side):
        """纯拓扑: 已知从 entry_side 边进入 c, 唯一的另一开口 = 出口方向.
           不看 walked/visited —— far 格未踏入也能解析 (45° cut 的前提).
           岔路/死路/场外开口 → None (不切)."""
        mk = self.marks.get(c)
        if not mk or mk['kind'] != 'way':
            return None
        outs = [d for d in mk['opens']
                if d != entry_side and (c, d) not in self.exit_open]
        return outs[0] if len(outs) == 1 else None

    def navigation_successor(self, cell, heading):
        """导航: 本格的下一步方向 (way 唯一未走开口; 重访/岔路交给搜索层)"""
        mk = self.marks.get(cell)
        if mk is None:
            return 'wait'
        if mk['kind'] == 'way':
            nd = self.m.nodes.get(cell, {}).get('edges', {})
            fwd = [d for d in mk['opens']
                   if nd.get(d) != 'walked' and (cell, d) not in self.exit_open]
            if len(fwd) == 1:
                return fwd[0]
        return None                                # branch/重访/死路 → 调用方走搜索层

    # ---- 决策 ----

    def nearest_frontier(self, cell):
        """BFS 已走边图 → 最近的 (a) 有 seen 分支 或 (b) 有未确认边的格.
           (b) 覆盖观测滞后: 该格还没被扫出 seen, 需要过去确认 —— 防 'home' 误判"""
        reach = {cell: (0, None)}
        q = [cell]
        while q:
            cc = q.pop(0)
            if cc != cell and (self.m.frontier(cc) or not self.cell_classified(cc)):
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

    def _all_resolved(self):
        """全图所有内部边+边界都确认 (全信息完成的真正判定)"""
        for i in range(self.n):
            for j in range(self.n):
                for d, dv in DIRV.items():
                    nb = (i + dv[0], j + dv[1])
                    if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n):
                        continue
                    if not self.resolved((i, j), d):
                        return False
        return True

    def plan_edge(self, cell, heading):
        """标记驱动决策: way 读拓扑/导航解析直接动; branch 才进搜索层.
           'wait' = 本格无标记; 'home' = 探索完"""
        mk = self.marks.get(cell)
        if mk is None:
            return 'wait'
        if mk['kind'] == 'way':
            d2 = self.navigation_successor(cell, heading)
            if d2 is None or d2 == 'wait':
                # way 全走过/解析失败 → 本格 frontier 或 BFS 回溯
                front = self.front(cell)
                if front:
                    d2 = self.choose(front, heading)
                else:
                    nf = self.nearest_frontier(cell)
                    if nf is None:
                        if self._all_resolved():
                            return 'home'
                        return 'wait'                # 观测滞后 → 等下一帧确认
                    d2 = nf[1]
        else:                                        # branch
            front = self.front(cell)

            def farc_of(d):
                return (cell[0] + DIRV[d][0], cell[1] + DIRV[d][1])
            front = [d for d in front
                     if self.marks.get(farc_of(d), {}).get('kind') != 'dead'
                     or self.has_block(farc_of(d))]
            if front:
                d2 = self.choose(front, heading)
            else:
                nf = self.nearest_frontier(cell)
                if nf is None:
                    if self._all_resolved():
                        return 'home'
                    return 'wait'                    # 有未确认区域但暂不可达 → 等观测
                d2 = nf[1]
        far = (cell[0] + DIRV[d2][0], cell[1] + DIRV[d2][1])
        turn_here = None if d2 == heading else ('cut' if d2 != OPP[heading] else 'rev')
        # peek far: 纯拓扑 exit_given_entry —— far 未踏入也能解析 (far_cut 真正可触发)
        end_o, v_end, d3, far_cut = 0.4, self.v_cruise, None, False
        mfar = self.marks.get(far)
        if mfar and mfar['kind'] == 'dead':
            v_end = 0.0
        elif mfar and mfar['kind'] == 'way':
            d3 = self.exit_given_entry(far, OPP[d2])
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
        if turn_here == 'cut':                       # 本格转弯: cut 来不及(入弯点在来向边)
            turn_here = 'spin90'                     # → 原地转 90°, 激励观测提前打标
        return {'d2': d2, 'far': far, 'end_o': end_o, 'v_end': v_end,
                'turn_here': turn_here, 'd3': d3, 'far_cut': far_cut}

    def v_cap(self, plan, o):
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
            'beliefs': {f'{k[0][0]},{k[0][1]},{k[1]},{k[2]}': v for k, v in self.beliefs.items()},
            'exit_open': [[list(k)] for k in self.exit_open],
            'marks': {f'{k[0]},{k[1]}': v for k, v in self.marks.items()},
            'blocks_seen': [list(b) for b in self.blocks_seen],
            'blocks_gone': [list(b) for b in self.blocks_gone],
            'got': self.got, 'exit_cell': self.exit_cell,
        })

    def from_json(self, s):
        d = json.loads(s)
        self.m = MazeMap.from_json(d['m'])
        self.beliefs = {}
        for k, v in d['beliefs'].items():
            a, b, dd, tail = k.split(',')
            key = ((int(a), int(b)), dd) if dd == 'B' else \
                  self.edge_key((int(a), int(b)), tail)
            self.beliefs[key] = v
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
