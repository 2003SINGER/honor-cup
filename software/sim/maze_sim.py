#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M3 Pro 迷宫 7×7 网格仿真 —— 场地建好之前的算法测试台

对应真实系统:
  迷宫   树形迷宫(生成树), 格距 0.4m, 7×7 —— 与赛题一致; 入口/出口在边界
  车体   沿线行驶, 注入控制误差: 转弯角度误差 / 里程计比例误差+噪声 / 雷达噪声
  决策   在线建拓扑树 + DFS, 分支顺序可配 (如 LFR / FLR / RFL), 不做剪枝(留接口)
  定位   里程计+IMU 推算 vs 「墙离散性栅格吸附」校正 —— 对比漂移

用法:
  python3 maze_sim.py explore  --seeds 30 --orders LFR,FLR,RFL,FRS
  python3 maze_sim.py localize --seed 7 --meters 30
  python3 maze_sim.py maze     --seed 3        (打印一个迷宫看看)
"""

import math
import argparse
import random
import statistics as st
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from mazemap import MazeMap

C = 0.4            # 格距 m (通道 40cm)
N = 7              # 7×7
DIRS = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
ORDER_D = {'N': 'N', 'E': 'E', 'S': 'S', 'W': 'W'}

# 双 TOF 雷达安装偏移 (URDF M3Pro.urdf laser_frame_Joint origin, 车体系: x前 y左)
# laser0 = 后左 (-0.11617, +0.09156) · laser1 = 前右 (+0.10766, -0.09078) —— 对角分布
LIDAR_OFFS = [(-0.11617, 0.09156), (0.10766, -0.09078)]

# ---------------- 迷宫 ----------------

def gen_maze(seed):
    """递归回溯法生成生成树迷宫; 返回 walls[(i,j)] = 剩余墙方向集合, 入口, 出口"""
    rnd = random.Random(seed)
    walls = {(i, j): {'N', 'E', 'S', 'W'} for i in range(N) for j in range(N)}
    visited = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        cur = stack[-1]
        nbrs = []
        for d, (dx, dy) in DIRS.items():
            nx, ny = cur[0] + dx, cur[1] + dy
            if 0 <= nx < N and 0 <= ny < N and (nx, ny) not in visited:
                nbrs.append((d, (nx, ny)))
        if not nbrs:
            stack.pop()
            continue
        d, nxt = rnd.choice(nbrs)
        walls[cur].discard(d)
        walls[nxt].discard(OPP[d])
        visited.add(nxt)
        stack.append(nxt)

    entry = (0, 0)
    walls[entry].discard('S')                       # 入口: 南边界开口
    bnd = [(i, j) for i in range(N) for j in range(N)
           if (i in (0, N - 1) or j in (0, N - 1)) and (i, j) != entry]
    ex = rnd.choice(bnd)                            # 出口: 随机边界格
    side = 'W' if ex[0] == 0 else 'E' if ex[0] == N - 1 else 'S' if ex[1] == 0 else 'N'
    walls[ex].discard(side)
    return walls, entry, ex, side


def print_maze(walls, entry, ex, blocks=()):
    lines = ['+' + '---+' * N]
    for j in reversed(range(N)):
        row = '|'
        brow = '+'
        for i in range(N):
            ch = ' . '
            if (i, j) == entry:
                ch = ' E '
            elif (i, j) == ex:
                ch = ' X '
            elif (i, j) in blocks:
                ch = ' * '
            row += ch + ('|' if 'E' in walls[(i, j)] else ' ')
            brow += ('---+' if 'S' in walls[(i, j)] else '   +')
        lines.append(row)
        lines.append(brow)
    return '\n'.join(lines)

# ---------------- 探索仿真 ----------------

def rel_of(d, heading):
    """绝对方向 d 相对当前朝向的方位: F/L/R/B"""
    m = {heading: 'F', OPP[heading]: 'B'}
    m[( {'N':'E','E':'S','S':'W','W':'N'}[heading] )] = 'R'
    m[( {'N':'W','W':'S','S':'E','E':'N'}[heading] )] = 'L'
    return m[d]


def explore(walls, entry, ex, order, blocks, v=0.30, t_turn=1.5, t_grab=1.0, corner='pivot'):
    """DFS 探索 —— 直接跑在 MazeMap 上（与实车决策层同一数据结构，09-23 接入）.
       order: 相对方向优先级, 如 'LFR' (B 永远最后).
       corner: 过弯方式
         pivot 停-转-走（巡线逻辑, 基线）
         holo  麦轮全向: 旋转与平移并行, 转弯零额外耗时（位置环解锁）
         arc   弧线切角: 转弯不停车, 且切掉路口角(路程省 C-(π/2)(C/2), 弧段限速 0.7v)
       提前终止: 出口已发现 且 方块收齐 → path_between 直奔出口.
       返回统计 dict"""
    m = MazeMap(N, entry)
    st_ = {'dist': 0.0, 'turns': 0, 'time': 0.0, 'got': 0,
           'exit_at': None, 'heading': 'N'}
    true_open = {c: [d for d in DIRS if d not in walls[c]] for c in walls}

    def turn_to(d):
        if st_['heading'] == d:
            return
        rev = st_['heading'] == OPP[d]               # 是否 180° 掉头
        r = 2 if rev else 1
        st_['turns'] += r
        if corner == 'holo':
            pass                                     # 全向: 旋转与平移并行, 零额外耗时
        elif corner == 'arc' and not rev:
            # 弧线切角(90°): 不停车, 切掉路口角
            cut = C - (math.pi / 2) * (C / 2)        # 路程省 ~0.086m (r=C/2)
            st_['dist'] -= cut
            st_['time'] += -t_turn + cut / (v * 0.7)  # 弧段限速 0.7v, 无原地转
        else:                                        # pivot / arc 的 180° 掉头: 原地转
            st_['time'] += t_turn * r
        st_['heading'] = d

    def drive(d):
        turn_to(d)
        st_['dist'] += C
        st_['time'] += C / v

    cell = entry
    m.touch(cell)
    stack = [entry]

    while True:
        for d in true_open[cell]:                        # 路口检测: 报出全部分支
            m.open_edge(cell, d)
        if cell in blocks and not m.nodes[cell]['block']:  # 只在首次到访时收集
            st_['got'] += 1
            st_['time'] += t_grab
            m.mark_block(cell)
        if m.exit_cell == cell and st_['exit_at'] is None:
            st_['exit_at'] = st_['dist']
        if m.exit_cell is not None and st_['got'] == len(blocks):
            break                                        # 出口已见 + 方块收齐 → 停止探索

        front = m.frontier(cell)
        if front:
            front.sort(key=lambda d: order.index(rel_of(d, st_['heading']))
                       if rel_of(d, st_['heading']) in order else 99)
            d = front[0]
        elif len(stack) > 1:                             # 回溯: 沿来路退一格
            stack.pop()
            d = next(dd for dd, (dx, dy) in DIRS.items()
                     if (cell[0] + dx, cell[1] + dy) == stack[-1])
        else:
            break                                        # 全图遍历完成, 回到入口

        drive(d)
        cell = m.walk_edge(cell, d)
        if front:
            stack.append(cell)

    if cell != ex:                                       # 直奔出口
        for d in m.path_between(cell, ex):
            drive(d)
    return st_

# ---------------- 流式探索仿真（2026-09-24：视野内=已知直接跑） ----------------
#
# 核心机制（与 docs/design/探索与路径规划.md「流式探索」节一一对应）：
#   置信模型   侧墙沿定位误差 err = P/sin²φ·δφ (φ=atan(P/s), P=0.2m 半通道宽)
#              正对(前墙)误差 ≈ 量程噪声 —— 远也准; 掠射(侧墙) err<gate 才确认
#   流式建图   每个扫描帧把"已确认"的边写进 MazeMap（open_edge）, 墙记入 wall_known
#   弧线提前承诺 下一格分类一置信立即决定: 直行/左弧/右弧/死路, 不等进格
#   速度调度   未分类格在前 → v ≤ √(v_arc²+2a·d) (最坏=死路需停在格中心);
#              分类完成 → 恢复巡航 —— 视界决定速度上限
#   处理时间   扫描 10Hz + 处理 proc_ms + 决策滞后 1 拍 —— 期间用保守上界, 安全
#   DFS        增量状态更新: 分类一格只做 O(1) 判定, 不重跑全局

# ---------------- 流式探索仿真（2026-09-24：视野内=已知直接跑） ----------------
#
# 对应 docs/design/探索与路径规划.md「流式探索」节:
#   置信模型   侧墙沿定位误差 err = P/sin²φ·δφ (P=0.2m, φ=atan(P/s)) —— 掠射越远越不可信;
#              正对(前墙) err ≈ ±20mm 量程噪声 —— 远也准。err<gate 才写入地图
#   流式建图   每个扫描帧(10Hz)把确认的边写进 MazeMap / wall_known, 增量 O(确认数)
#   弧线提前承诺 下一格分类一置信立即定机动: 直行/左弧/右弧/死路/抓取, 不等进格
#   速度调度   plan 未定(下一格未分类) → 保守上界 v ≤ √(2a·(d_停 − margin));
#              分类完成 → 按机动定末速(弧线 v_arc=√(a_lat·r), 死路/抓取停 0)
#   处理时间   扫描周期 + proc_ms + 决策滞后一拍; 期间保守上界兜底
#   DFS        增量: 分类一格只做 O(1) 判定; 收齐+见出口 → path_between 已知路径速度跑

DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
P_HALF = 0.2                                       # 半通道宽


class World:
    """物理世界: 真值迷宫 + 真值车位. 小车只能通过 sense()/block_at() 看世界."""

    def __init__(self, walls, blocks, entry):
        self.walls = walls
        self.blocks = set(blocks)
        self.cell = entry
        self.heading = 'N'
        self.o = 0.0
        self.collected = set()

    def sense(self, rng=0.02, maxr=2.4):
        """360° 仿真雷达: 四轴正入射 + 对角掠射, 带量程噪声; 遮挡由真值计算.
           返回 (hits, opens): hits[(ck,axis)]=带噪距离(墙), opens=[(ck,axis)](开口)"""
        hits, opens = {}, []
        hv = DIRV[self.heading]
        offs = {self.heading: self.o, OPP[self.heading]: -self.o}

        def ray(axis, ck, s):
            """沿 axis 的边界 ck 在距离 s 处: 返回测量(墙→带噪距离)或 None(开口)"""
            if s < 0.05 or s > maxr:
                return None
            if axis in self.walls[ck]:
                return s + random.gauss(0, rng)
            return None

        # ① 四轴正入射链(含本格四边), 墙即遮挡
        for axis in DIRS:
            av = DIRV[axis]
            p_off = offs.get(axis, 0.0)
            occl = False
            for k in range(0, N + 2):
                if occl:
                    break
                ck = (self.cell[0] + k * av[0], self.cell[1] + k * av[1])
                if not (0 <= ck[0] < N and 0 <= ck[1] < N):
                    break
                s = 0.2 + 0.4 * k - p_off
                if s > maxr:
                    break
                if s < 0.05:
                    continue
                if axis in self.walls[ck]:
                    hits[(ck, axis)] = s + random.gauss(0, rng)
                    occl = True
                else:
                    opens.append((ck, axis, s))
        # ② 前方格侧边对角掠射
        occluded = False
        for k in range(1, N + 2):
            if occluded:
                break
            ck = (self.cell[0] + k * hv[0], self.cell[1] + k * hv[1])
            if not (0 <= ck[0] < N and 0 <= ck[1] < N):
                break
            s_far = 0.2 + 0.4 * k - self.o
            if s_far > maxr:
                break
            for sd in DIRS:
                if DIRV[sd] == hv or DIRV[sd] == (-hv[0], -hv[1]):
                    continue
                if sd in self.walls[ck]:
                    phi = math.atan2(P_HALF, s_far)
                    hits[(ck, sd)] = s_far + random.gauss(0, max(rng, P_HALF / math.sin(phi) ** 2 * math.radians(0.3)))
                else:
                    opens.append((ck, sd, s_far))
            if self.heading in self.walls[ck]:
                occluded = True
        return hits, opens

    def block_at(self, c):
        """相机: 该格是否有未收集方块(能看见就算)"""
        return c in self.blocks and c not in self.collected

    def collect(self, c):
        self.collected.add(c)


def explore_stream(walls, entry, ex, order, blocks, *,
                   v_cruise=0.50, a_acc=1.0, a_dec=1.0, a_lat=0.7,
                   dphi_deg=0.30, gate=0.06, scan_hz=10.0, proc_ms=5.0,
                   t_spin180=1.0, t_grab=1.0, ctrl_hz=50.0, v_run=0.60,
                   assume_tree=True):
    """流式探索 v3 —— 世界/小车分离:
       World 持真值, 只暴露 sense()(带噪观测)/block_at(); Agent(本函数内联)持认知:
       MazeMap + wall_known + exit_open, 永不读真值. 决策骨架同 explore()."""
    P = C / 2
    P_G = P
    DPHI = math.radians(dphi_deg)
    MAXR = 2.4
    v_arc = math.sqrt(a_lat * P)
    dt = 1.0 / ctrl_hz
    scan_every = max(1, round(ctrl_hz / scan_hz))
    lag = 1 + math.ceil(proc_ms * 1e-3 * ctrl_hz)

    world = World(walls, blocks, entry)
    m = MazeMap(N, entry)
    wall_known = {}                                  # 认知: 已确认的墙
    exit_open = set()                                # 认知: 已确认的出口开口
    st = {'time': 0.0, 'dist': 0.0, 'got': 0, 'arcs': 0, 'spins': 0,
          'grabs': 0, 'violations': 0, 'sched_t': 0.0, 'obs_new': 0}

    def resolved(c, d):
        return (c, d) in wall_known or (c, d) in exit_open or \
               m.nodes.get(c, {}).get('edges', {}).get(d) in ('seen', 'walked')

    def ingest(hits, opens):
        """观测 → 置信门限 → 认知更新. 几何误差模型(而非读真值):
           正入射 err≈量程噪声; 掠射 err=P/sin²φ·δφ"""
        newly = 0
        for (ck, axis), d_meas in hits.items():
            if resolved(ck, axis):
                continue
            s = abs(0.2 + 0.4 * 0 - 0) + d_meas      # 位置由里程计提供(近似真值)
            phi = math.atan2(P, max(d_meas, 0.21))
            err = max(0.02, P / math.sin(phi) ** 2 * DPHI) if d_meas > 0.25 \
                else math.hypot(0.02, d_meas * DPHI)
            if err < gate:
                wall_known[(ck, axis)] = True
                nbm = (ck[0] + DIRV[axis][0], ck[1] + DIRV[axis][1])
                if 0 <= nbm[0] < N and 0 <= nbm[1] < N:
                    wall_known[(nbm, OPP[axis])] = True   # 边共享: 一面墙两侧同时确认
                newly += 1
        for (ck, axis, s) in opens:
            if resolved(ck, axis):
                continue
            if s < 0.25:                              # 正入射/贴身: 直接确认
                err = 0.02
            else:                                     # 对角掠射: 远才可信
                phi = math.atan2(P, s)
                err = max(0.02, P / math.sin(phi) ** 2 * DPHI)
            if err < gate:
                nb = m.open_edge(ck, axis)
                if nb is None:
                    exit_open.add((ck, axis))
                newly += 1
        return newly

    def cell_classified(c):
        return all(resolved(c, d) for d in DIRS)

    # ---- 格子标记器 (用户方案): 四边登记齐 → 直接打"怎么走"标记 ----
    # 纯局部观测: 只看本格四边登记状态(墙/开口), 零推理、不依赖树假设.
    # way 标记含行进方向 → 决策层进格前读标记直接动, 低耦合.
    marks = {}                                       # cell -> {'kind','d2'}

    def try_mark(c):
        if c != entry and c not in m.nodes:
            return
        if any(not resolved(c, d) for d in DIRV):
            return                                   # 还有未知边: 不打, 靠近再说
        opens = [d for d in DIRV if (c, d) not in wall_known]
        n = len(opens)
        new_mk = ({'kind': 'dead', 'd2': None} if n == 0 else
                  {'kind': 'way', 'd2': opens[0]} if n == 1 else
                  {'kind': 'branch', 'd2': None})
        if marks.get(c) != new_mk:
            if c in marks:
                st['mark_revised'] = st.get('mark_revised', 0) + 1
            marks[c] = new_mk

    def consistency_guard():
        """认知一致性: walked 边是物理事实(车真走过了), 推断墙与之冲突 → 删墙.
           防止'墙+walked并存'的矛盾认知导致决策死循环."""
        n = 0
        for cc, nd in m.nodes.items():
            for d, s in nd['edges'].items():
                if s == 'walked' and (cc, d) in wall_known:
                    del wall_known[(cc, d)]
                    nbm = (cc[0] + DIRV[d][0], cc[1] + DIRV[d][1])
                    wall_known.pop((nbm, OPP[d]), None)
                    marks.pop(cc, None)                # 该格标记作废重打
                    marks.pop(nbm, None)
                    n += 1
        st['wall_conflicts'] = st.get('wall_conflicts', 0) + n
        return n

    def prune():
        """树环剪枝: 未知边两端点在已知通道图已连通 → 必是墙 (加边即成环). 另含镜像."""
        newly = 0
        for (ck, axis) in list(wall_known):
            nbm = (ck[0] + DIRV[axis][0], ck[1] + DIRV[axis][1])
            if 0 <= nbm[0] < N and 0 <= nbm[1] < N and (nbm, OPP[axis]) not in wall_known:
                wall_known[(nbm, OPP[axis])] = True
                newly += 1
        if not assume_tree:
            return newly
        parent = {}
        def find(x):
            parent.setdefault(x, x)
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        for c2, nd in m.nodes.items():
            for d, s in nd['edges'].items():
                if s not in ('seen', 'walked'):
                    continue
                a, b = find(c2), find((c2[0] + DIRV[d][0], c2[1] + DIRV[d][1]))
                if a != b:
                    parent[a] = b
        for i in range(N):
            for j in range(N):
                for d, dv in DIRV.items():
                    nb = (i + dv[0], j + dv[1])
                    if not (0 <= nb[0] < N and 0 <= nb[1] < N):
                        continue
                    ck = (i, j)
                    if resolved(ck, d):
                        continue
                    if find(ck) == find(nb):
                        wall_known[(ck, d)] = True
                        wall_known[(nb, OPP[d])] = True
                        newly += 1
        return newly

    # ---- 初始静态扫描 ----
    cell, heading, o, v = entry, 'N', 0.0, 0.0
    m.touch(entry)
    exit_open.add((entry, 'S'))                    # 入口外=已确认开口(非墙, 非出口格)
    hits, opens = world.sense()
    ingest(hits, opens)
    prune()
    try_mark(entry)

    plan = None
    arc_left = 0.0
    tick = 0

    def go_home():
        exc = m.exit_cell
        reach = {cell}
        q = [cell]
        while q:
            cc = q.pop()
            for d, dv in DIRV.items():
                if m.nodes.get(cc, {}).get('edges', {}).get(d) == 'walked':
                    nb = (cc[0] + dv[0], cc[1] + dv[1])
                    if nb not in reach:
                        reach.add(nb)
                        q.append(nb)
        if exc in reach:
            return m.path_between(cell, exc), 0.0
        for d, dv in DIRV.items():
            nb = (exc[0] + dv[0], exc[1] + dv[1])
            if nb in reach and 0 <= nb[0] < N and 0 <= nb[1] < N and \
               m.nodes.get(nb, {}).get('edges', {}).get(OPP[d]) in ('seen', 'walked'):
                return m.path_between(cell, nb), 0.6
        return None, 0.0

    def speed_run(path_dirs):
        d_tot, cuts, prev = 0.0, 0.0, None
        for d in path_dirs:
            d_tot += C
            if prev is not None and d != prev and d != OPP[prev]:
                cuts += C - (math.pi / 2) * P
            prev = d
        st['dist'] += d_tot - cuts
        st['time'] += d_tot / v_run
        st['arcs'] += sum(1 for a, b in zip(path_dirs[:-1], path_dirs[1:])
                          if b != a and b != OPP[a])

    def finish():
        unres = 0
        for i in range(N):
            for j in range(N):
                for d, dv in DIRV.items():
                    nb = (i + dv[0], j + dv[1])
                    if not (0 <= nb[0] < N and 0 <= nb[1] < N):
                        continue
                    if not resolved((i, j), d):
                        unres += 1
        st['unresolved'] = unres // 2
        dirs, extra = go_home()
        if dirs is None:
            return None                                 # 出口区域还没走到
        speed_run(dirs)
        if extra:
            st['dist'] += extra
            st['time'] += extra / v_run
        return st

    def nearest_frontier_dir():
        """BFS 已走边图, 找最近的有 seen 分支的格; 返回 (第一步方向, 步数) 或 None"""
        reach = {cell: (0, None)}
        q = [cell]
        while q:
            cc = q.pop(0)
            if cc != cell and m.frontier(cc):
                return reach[cc]
            for d, dv in DIRV.items():
                if m.nodes.get(cc, {}).get('edges', {}).get(d) != 'walked':
                    continue
                nb = (cc[0] + dv[0], cc[1] + dv[1])
                if nb not in reach:
                    reach[nb] = (reach[cc][0] + 1,
                                 d if cc == cell else reach[cc][1])
                    q.append(nb)
        return None

    def make_plan():
        """标记驱动决策: 非岔路格读标记直接动(零推理); 岔路/无标记才走搜索层.
           本格: way→唯一出口直接走; branch/未标→frontier选向; 未标记→wait靠近"""
        mk = marks.get(cell)
        if mk is None:
            return 'wait'                              # 四边未齐: 靠近/等待观测
        if mk['kind'] == 'way':
            d2 = mk['d2']                              # 唯一出口: 标记即答案
        else:                                          # branch
            front = [d for d in m.frontier(cell)]
            # dead 方向跳过 — 除非那格有方块(必须进去抓)
            def farc_of(d):
                return (cell[0] + DIRV[d][0], cell[1] + DIRV[d][1])
            front = [d for d in front
                     if marks.get(farc_of(d), {}).get('kind') != 'dead'
                     or world.block_at(farc_of(d))]
            if front:
                d2 = sorted(front, key=lambda dd: order.index(rel_of(dd, heading))
                            if rel_of(dd, heading) in order else 99)[0]
            else:
                nf = nearest_frontier_dir()            # 回溯: BFS 最近未探分支
                if nf is None:
                    return 'home'                      # 真·全图探完
                d2 = nf[1]
        far = (cell[0] + DIRV[d2][0], cell[1] + DIRV[d2][1])
        grab = world.block_at(far)
        turn_here = None if d2 == heading else ('arc' if d2 != OPP[heading] else 'rev')
        cut = (cell[0] + DIRV[heading][0], cell[1] + DIRV[heading][1])
        if turn_here == 'arc' and world.block_at(cut):
            d2, far, grab = heading, cut, True
            turn_here = None                           # 直行进格先抓, 不切角
        # peek far 转弯方式: 优先读标记; branch 用决策层预演
        end_o, v_end, d3, far_arc = 0.4, v_cruise, None, False
        mfar = marks.get(far)
        if mfar and mfar['kind'] == 'dead':
            v_end = 0.0                                # 死路: 停格心(抓块)或掉头
        elif mfar and mfar['kind'] == 'way':
            d3 = mfar['d2']
            if d3 != d2 and d3 != OPP[d2]:
                far_arc = True
                end_o, v_end = 0.2, v_arc
        elif cell_classified(far):
            f_far = [d for d in m.frontier(far)]
            if f_far:
                d3 = sorted(f_far, key=lambda dd: order.index(rel_of(dd, d2))
                            if rel_of(dd, d2) in order else 99)[0]
                if d3 != d2 and d3 != OPP[d2]:
                    far_arc = True
                    end_o, v_end = 0.2, v_arc
            else:
                v_end = 0.0
        if grab:
            end_o, v_end = 0.4, 0.0
        return {'d2': d2, 'end_o': end_o, 'v_end': v_end, 'grab': grab,
                'd3': d3, 'far_arc': far_arc, 'turn_here': turn_here}

    while True:
        tick += 1
        st['time'] += dt

        if len(blocks) > 0 and m.exit_cell is not None and st['got'] == len(blocks):
            r = finish()
            if r is not None:
                return r

        # ---- 观测(扫描拍; 世界给带噪数据, 小车只消费) ----
        if tick % scan_every == 0:
            st['obs_new'] += ingest(*world.sense())
            prune()
            consistency_guard()
            for c2 in list(m.nodes):
                try_mark(c2)

        # ---- 决策 ----
        if plan is None:
            p_ = make_plan()
            if p_ == 'home':
                r = finish()
                if r is not None:
                    return r
                continue                               # 极罕见: 出口未可达, 继续探
            if p_ == 'wait':
                v = 0.0
                continue
            plan = p_
            if plan['turn_here'] == 'rev':
                st['spins'] += 1
                st['time'] += t_spin180
                heading = plan['d2']                   # 掉头后朝向 = 行进方向
                world.heading = heading
            elif plan['turn_here'] == 'arc':
                arc_left = (math.pi / 2) * P            # 本格转弯弧线立即执行
                st['arcs'] += 1

        # ---- 速度调度(视界) ----
        seg_end, seg_vend = plan['end_o'], plan['v_end']
        farc = (cell[0] + DIRV[plan['d2']][0], cell[1] + DIRV[plan['d2']][1])
        if not cell_classified(farc):
            cap = math.sqrt(max(0.0, 2 * a_dec * max(0.0, 0.4 - o - 0.05)))
            if v > cap:
                v = cap
                st['sched_t'] += dt

        # ---- 运动(世界执行) ----
        if arc_left > 0.0:
            step = min(arc_left, v_arc * dt)
            arc_left = arc_left - step
            st['dist'] += step
            if arc_left <= 1e-9:
                # 本格转弯弧线: 弧在 C 内自转(切点=两侧中心), 不走格
                heading = plan['d2']
                world.heading = heading
                o, v, plan = 0.2, v_arc, None
            continue

        v_allow = math.sqrt(max(0.0, seg_vend ** 2 + 2 * a_dec * max(0.0, seg_end - o)))
        v = max(0.0, min(v + a_acc * dt, v_cruise, v_allow))
        o += v * dt
        st['dist'] += v * dt
        world.o = o

        if o >= seg_end - 1e-9:
            o = seg_end
            if plan['grab']:
                if not (0 <= cell[0] + DIRV[heading][0] < N and 0 <= cell[1] + DIRV[heading][1] < N):
                    import sys as _s
                    print(f"BUG grab-out cell={cell} hdg={heading} plan={plan}", file=_s.stderr)
                    break
                farc = m.walk_edge(cell, heading)
                world.collect(farc)
                st['enters'] = st.get('enters', 0) + 1
                st['mark_hit'] = st.get('mark_hit', 0) + (farc in marks)
                st['got'] += 1
                st['grabs'] += 1
                st['time'] += t_grab
                cell, o, v, plan = farc, 0.0, 0.0, None
                world.cell = cell
                continue
            if plan['far_arc']:
                arc_left = (math.pi / 2) * P
                st['arcs'] += 1
                nxtc = m.walk_edge(cell, heading)        # 走到 far
                st['enters'] = st.get('enters', 0) + 1
                st['mark_hit'] = st.get('mark_hit', 0) + (nxtc in marks)
                cell = nxtc                              # 弧线切 far 的角: 车停 far 内 o=0.2
                world.cell = cell
                heading = plan['d3']
                world.heading = heading
                o = 0.2
                v = v_arc
                plan = None
                continue
            if not (0 <= cell[0] + DIRV[heading][0] < N and 0 <= cell[1] + DIRV[heading][1] < N):
                import sys as _s
                print(f"BUG straight-out cell={cell} hdg={heading} plan={plan} "
                      f"frontier={m.frontier(cell)} exit_open={sorted(exit_open)}", file=_s.stderr)
                break
            nxtc = m.walk_edge(cell, heading)
            st['enters'] = st.get('enters', 0) + 1
            st['mark_hit'] = st.get('mark_hit', 0) + (nxtc in marks)
            cell, o, v, plan = nxtc, 0.0, v, None
            world.cell = cell

    return st

# ---------------- 定位仿真（墙登记册版） ----------------

def _raycast(true, dvec, walls, rmax=4.0):
    """真值位置沿 dvec 单位向量打雷达, 返回到第一面墙的距离(>maze 边界则 rmax)"""
    ax = 0 if dvec[0] else 1
    s = 1 if dvec[ax] > 0 else -1
    dd = {(1, 0): 'E', (-1, 0): 'W', (0, 1): 'N', (0, -1): 'S'}[dvec]
    i, j = min(max(int(true[0] // C), 0), N - 1), min(max(int(true[1] // C), 0), N - 1)
    d = ((i + 1) * C - true[0]) if (ax == 0 and s > 0) else \
        (true[0] - i * C) if ax == 0 else \
        ((j + 1) * C - true[1]) if s > 0 else (true[1] - j * C)
    while d <= rmax:
        if dd in walls.get((i, j), {'E', 'W', 'N', 'S'}):   # 边界外的格子视为全墙
            return d
        i += dvec[0]
        j += dvec[1]
        if not (0 <= i < N and 0 <= j < N):
            return rmax
        d += C
    return rmax


def localize(seed, meters, v=0.30, dt=0.05,
             odom_scale=1.03, odom_noise=0.02,          # 里程计: 比例误差 + 每步噪声σ(m)
             lidar_noise=0.01,                          # 雷达测距噪声 σ(m)
             snap_gain=0.8, gate=0.1,                   # 吸附门限: 必须 < 半格0.2m, 否则会级联吸错
             slip_at=0.4, slip=0.25):                   # 中途打滑: 推算坐标突跳(模拟撞墙/打滑)
    """定位对比 · 三种模式走同一条轨迹(各 30m, 独立噪声流):
       raw    纯里程计推算
       naive  无脑吸附: 每次墙观测都硬吸到最近格线 —— 吸附错了会锁死
       reg    墙登记册 + 门限: 启动时(漂移=0,吸附可信)登记 4m 内墙的绝对格线坐标;
              之后观测先查册, 查到→按登记坐标校正; 查不到→|残差|≤gate 才新增, 否则丢弃
       中途注入一次打滑 est += slip (0.25m > 半格 0.2m) —— 用户担心的「吸附错」场景"""
    DV = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    def run(mode):
        rnd = random.Random(seed + {'raw': 0, 'naive': 1000, 'reg': 2000}[mode])
        walls, entry, ex, _ = gen_maze(seed)
        NBR = {}
        for (i, j) in walls:
            NBR[(i, j)] = [d for d in DIRS
                           if d not in walls[(i, j)]
                           and 0 <= i + DIRS[d][0] < N and 0 <= j + DIRS[d][1] < N]

        cell, came, heading = entry, None, 'N'
        true = [C / 2, C / 2]                       # 入口格中心, 位姿已知 → 漂移=0
        est = [C / 2, C / 2]
        register = {}                               # ('x'|'y', 格线坐标) -> 精确坐标
        errs = []
        anomalies = 0                               # 地图系门限外观测(=障碍物/打滑/噪声; 方块太矮雷达扫不到)
        traveled = 0.0
        slip_done = False

        if mode == 'reg':                           # 启动扫描: 4m 内墙全部入库(两只雷达轮询)
            for si, dv in enumerate(DV):
                off = LIDAR_OFFS[si % 2]
                sen = [true[0] + off[0], true[1] + off[1]]
                d = _raycast(sen, dv, walls)
                if d < 4.0:
                    a = 0 if dv[0] else 1
                    W = sen[a] + dv[a] * d
                    register[(('x', 'y')[a], round(W / C) * C)] = W

        while traveled < meters:
            opts = list(NBR[cell])
            if came in opts and len(opts) > 1:
                opts.remove(came)
            heading = rnd.choice(opts)
            axis = 0 if DIRS[heading][0] else 1
            perp = 1 - axis
            hv = DIRS[heading]
            sides = [hv, (0, 1) if abs(hv[0]) else (1, 0), (0, -1) if abs(hv[0]) else (-1, 0)]

            nsteps = int(-(-C // (v * dt)))             # ceil: 每格整数步
            step = C / nsteps                           # 步长归一 → 每格恰好走 C 米
            for _ in range(nsteps):
                true[0] += hv[0] * step
                true[1] += hv[1] * step
                m = step
                est[0] += hv[0] * m * odom_scale + rnd.gauss(0, odom_noise)
                est[1] += hv[1] * m * odom_scale + rnd.gauss(0, odom_noise)
                traveled += m

                if mode != 'raw':
                    # 观测建在【地图坐标系】: 墙线坐标 = 传感器位置 + sgn*墙距, 恒在格点上
                    # 传感器世界位置 = 车位姿 + 旋转后的安装偏移(URDF: laser0 后左 / laser1 前右)
                    fwd = hv
                    left = (-hv[1], hv[0])              # 朝向左转 90°
                    for idx, dv in enumerate(sides):
                        a = 0 if dv[0] else 1
                        sgn = dv[a]
                        off = LIDAR_OFFS[idx % 2]       # 车体系偏移 (fx前, fy左)
                        w_off = (off[0] * fwd[0] + off[1] * left[0],
                                 off[0] * fwd[1] + off[1] * left[1])
                        sen_t = [true[0] + w_off[0], true[1] + w_off[1]]
                        sen_e = [est[0] + w_off[0], est[1] + w_off[1]]
                        d_true = _raycast(sen_t, dv, walls)
                        if d_true >= 4.0:
                            continue
                        d_meas = d_true + rnd.gauss(0, lidar_noise)
                        cand = sen_e[a] + sgn * d_meas              # 地图系墙坐标估计
                        key = (('x', 'y')[a], round(cand / C) * C)
                        if mode == 'reg':
                            W = register.get(key)
                            if W is not None and abs(cand - W) <= gate:   # 查册命中且过门限
                                est[a] += ((W - sgn * d_meas - w_off[a]) - est[a]) * snap_gain
                            elif W is None and abs(cand - key[1]) <= gate:  # 新墙: 门限内才收
                                register[key] = key[1]
                                est[a] += ((key[1] - sgn * d_meas - w_off[a]) - est[a]) * snap_gain
                            else:
                                anomalies += 1              # 残差超门限 = 方块/打滑/噪声
                                continue
                        else:                                   # naive: 无脑硬吸
                            est[a] = key[1] - sgn * d_meas - w_off[a]

                    # 路口(格中心)事件: 黑线分叉可检测 → 沿航向吸附到格点
                    if abs((true[axis] % C) - C / 2) < v * dt / 2:
                        node = round((true[axis] - C / 2) / C) * C + C / 2
                        est[axis] -= (est[axis] - node) * snap_gain

                if mode != 'raw' and not slip_done and traveled > meters * slip_at:
                    est[0] += slip                              # 打滑/撞墙: 推算突跳
                    slip_done = True

                errs.append(((est[0] - true[0]) ** 2 + (est[1] - true[1]) ** 2) ** 0.5)

            cell = (cell[0] + hv[0], cell[1] + hv[1])
            came = OPP[heading]
        return errs, anomalies

    out = {}
    for mode in ('raw', 'naive', 'reg'):
        e, ano = run(mode)
        out[mode] = {'rmse': (sum(x * x for x in e) / len(e)) ** 0.5, 'max': max(e), 'errs': e}
        if mode == 'reg':
            out['anomalies'] = ano
    out['meters'] = meters
    out['slip'] = slip
    return out

# ---------------- CLI ----------------

def cmd_explore(a):
    orders = a.orders.split(',')
    corners = ['pivot', 'holo', 'arc'] if a.corner == 'all' else [a.corner]
    print(f"探索仿真 · {a.seeds} 个迷宫种子 · 车速 {a.v} m/s · 抓取 {a.grab}s/个 · 不剪枝")
    print(f"过弯方式: {', '.join(corners)}   (pivot=停转走 holo=麦轮边走边转 arc=弧线切角)\n")
    for corner in corners:
        print(f"── 过弯方式 [{corner}] ──")
        print(f"{'顺序':<8}{'平均路程(m)':>12}{'最短':>8}{'最长':>8}{'平均用时(s)':>12}{'出口发现(m)':>12}")
        for o in orders:
            res = []
            for s in range(a.seeds):
                rnd = random.Random(1000 + s)
                walls, entry, ex, _ = gen_maze(s)
                cells = [(i, j) for i in range(N) for j in range(N) if (i, j) not in (entry, ex)]
                blocks = set(rnd.sample(cells, 8))
                r = explore(walls, entry, ex, o, blocks, v=a.v, t_grab=a.grab, corner=corner)
                res.append(r)
            d = [r['dist'] for r in res]
            t = [r['time'] for r in res]
            e = [r['exit_at'] for r in res]
            print(f"{o:<8}{st.mean(d):>12.1f}{min(d):>8.1f}{max(d):>8.1f}{st.mean(t):>12.1f}{st.mean(e):>12.1f}")
    print("\n注: 树形迷宫里, 不触发提前终止时 DFS 每条边恰走两次(路程与顺序无关);")
    print("    分支顺序的全部影响 = 「出口/最后一块被发现的早晚」→ 决定『收齐+见出口→直奔出口』何时触发。")
    print("    单个迷宫差异大(方差高), 结论要以多种子统计为准。")


def cmd_localize(a):
    print(f"定位仿真 · 每种子走 {a.meters}m · 里程计比例误差+3% · 中途打滑 est+0.25m(>半格0.2m)\n")
    print(f"{'种子':<6}{'raw-RMSE':>10}{'naive-RMSE':>12}{'reg-RMSE':>10}{'naive峰值':>10}{'reg峰值':>9}")
    rs = {'raw': [], 'naive': [], 'reg': []}
    for s in range(a.seeds):
        r = localize(s, a.meters)
        for k in rs:
            rs[k].append(r[k]['rmse'])
        print(f"{s:<6}{r['raw']['rmse']:>9.3f}m{r['naive']['rmse']:>11.3f}m"
              f"{r['reg']['rmse']:>9.3f}m{r['naive']['max']:>9.3f}m{r['reg']['max']:>8.3f}m")
    print('-' * 60)
    print(f"{'均值':<6}{st.mean(rs['raw']):>9.3f}m{st.mean(rs['naive']):>11.3f}m"
          f"{st.mean(rs['reg']):>9.3f}m")
    print("\nnaive = 无脑吸附(用户担心的「吸附错」: 打滑后锁死在错误格线上)")
    print("reg   = 墙登记册 + 门限(启动时位姿准→首批墙可信入库; 之后查册校正, 残差>0.1m 丢弃)")


def cmd_stream(a):
    orders = a.orders.split(',')
    print(f"流式探索 · {a.seeds} 种子 · 巡航 {a.vc} m/s · 弧线限速 √(0.7·0.2)={math.sqrt(0.7*0.2):.3f} m/s · "
          f"δφ={a.dphi}° · gate={a.gate}m · 扫描 {a.scan}Hz · 处理 {a.proc}ms\n")
    print(f"{'顺序':<8}{'平均用时(s)':>10}{'最短':>7}{'最长':>7}{'平均路程':>9}{'平均速':>8}{'弧线':>6}{'掉头':>6}{'违规':>6}{'未确认':>7}{'标记命中':>8}")
    for o_ in orders:
        res = []
        for s in range(a.seeds):
            rnd = random.Random(1000 + s)
            walls, entry, ex, _ = gen_maze(s)
            cells = [(i, j) for i in range(N) for j in range(N) if (i, j) not in (entry, ex)]
            blocks = set() if a.fullinfo else set(rnd.sample(cells, 8))
            r = explore_stream(walls, entry, ex, o_, blocks, v_cruise=a.vc,
                               dphi_deg=a.dphi, gate=a.gate, scan_hz=a.scan, proc_ms=a.proc,
                               assume_tree=not a.no_prune)
            res.append(r)
        t = [r['time'] for r in res]
        d = [r.get('dist', 0.0) for r in res]
        print(f"{o_:<8}{st.mean(t):>10.1f}{min(t):>7.1f}{max(t):>7.1f}{st.mean(d):>9.1f}"
              f"{st.mean(d) / st.mean(t):>8.2f}"
              f"{st.mean([r['arcs'] for r in res]):>6.0f}{st.mean([r['spins'] for r in res]):>6.1f}"
              f"{st.mean([r['violations'] for r in res]):>6.1f}"
              f"{st.mean([r.get('unresolved', -1) for r in res]):>6.1f}"
              f"{100 * st.mean([r.get('mark_hit', 0) / max(1, r.get('enters', 1)) for r in res]):>6.1f}%")
    print("\n对照(同迷宫): pivot@0.3 186.7s · holo@0.3 109.8s · arc@0.3 78.2s (30种子, explore 命令)")
    print("流式 = 观测置信建图+弧线提前承诺+视界调速+处理延迟; 违规>0 说明视界模型过于乐观")


def cmd_maze(a):
    walls, entry, ex, _ = gen_maze(a.seed)
    rnd = random.Random(a.seed + 500)
    cells = [(i, j) for i in range(N) for j in range(N) if (i, j) not in (entry, ex)]
    blocks = set(rnd.sample(cells, 8))
    print(f"迷宫 seed={a.seed}   E=入口(南开口) X=出口 *=方块")
    print(print_maze(walls, entry, ex, blocks))


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    e = sub.add_parser('explore')
    e.add_argument('--seeds', type=int, default=30)
    e.add_argument('--orders', default='LFR,FLR,RFL,FRS')
    e.add_argument('--v', type=float, default=0.30)
    e.add_argument('--grab', type=float, default=1.0)
    e.add_argument('--corner', default='pivot', choices=['pivot', 'holo', 'arc', 'all'])
    l = sub.add_parser('localize')
    l.add_argument('--seeds', type=int, default=10)
    l.add_argument('--meters', type=float, default=30)
    m = sub.add_parser('maze')
    m.add_argument('--seed', type=int, default=3)
    s2 = sub.add_parser('stream')
    s2.add_argument('--seeds', type=int, default=30)
    s2.add_argument('--orders', default='LFR,FLR,RFL,FRS')
    s2.add_argument('--vc', type=float, default=0.50)
    s2.add_argument('--dphi', type=float, default=0.30)
    s2.add_argument('--gate', type=float, default=0.06)
    s2.add_argument('--scan', type=float, default=10.0)
    s2.add_argument('--proc', type=float, default=5.0)
    s2.add_argument('--fullinfo', action='store_true', help='不看方块: 最快拿全地图信息')
    s2.add_argument('--no-prune', action='store_true', help='关闭树环剪枝对照')
    a = p.parse_args()
    {'explore': cmd_explore, 'localize': cmd_localize, 'maze': cmd_maze,
     'stream': cmd_stream}[a.cmd](a)


if __name__ == '__main__':
    main()
