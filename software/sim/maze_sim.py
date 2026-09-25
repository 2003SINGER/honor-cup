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

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                'ros2', 'm3pro_nav'))
from m3pro_nav.mazemap import MazeMap        # 铁律: 单一真相源, 只 import m3pro_nav 包
from m3pro_nav.stream_nav import StreamNav
from m3pro_nav.tracker import HolonomicTracker

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
            st_['time'] += cut / (v * 0.7)            # 弧段限速 0.7v, 无原地转 (bug修复: 原多减一次 t_turn)
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
    """物理世界: 真值迷宫 + 真值车位 + 雷达/相机物理模型.
       雷达: 返回 (cell,dir,dist,alpha) —— alpha=射线与墙法线夹角(真实几何+角噪声),
             类别(墙/口)由真值遮挡计算 —— 语义关联 (cell,dir) 由仿真直接给出,
             实车上由节点层几何反算 (仿真边界, 见 README).
       相机: 方块仅在视野内可见 (前方 cam_range 内 ±45° 锥), 非上帝视角."""

    def __init__(self, walls, blocks, entry):
        self.walls = walls
        self.blocks = set(blocks)
        self.cell = entry
        self.heading = 'N'
        self.o = 0.0
        self.collected = set()
        # 真值墙线段 (碰撞检查用)
        self.wall_segs = []
        for (i, j), ds in walls.items():
            for d in ds:
                x0, y0 = i * C, j * C
                if d == 'N':
                    self.wall_segs.append(((x0, y0 + C), (x0 + C, y0 + C)))
                elif d == 'S':
                    self.wall_segs.append(((x0, y0), (x0 + C, y0)))
                elif d == 'E':
                    self.wall_segs.append(((x0 + C, y0), (x0 + C, y0 + C)))
                elif d == 'W':
                    self.wall_segs.append(((x0, y0), (x0, y0 + C)))
        self.wall_segs = list({tuple(sorted(s)) for s in self.wall_segs})

    def sense(self, rng=0.02, maxr=2.4, dphi_deg=0.30):
        """360° 一帧: 四轴正入射(α≈0) + 前方侧边对角(α=atan(s,P) 掠射).
           返回 hits{(c,d):(dist,alpha)}, opens[(c,d,dist,alpha)]"""
        dphi = math.radians(dphi_deg)
        hits, opens = {}, []
        hv = DIRV[self.heading]
        offs = {self.heading: self.o, OPP[self.heading]: -self.o}
        for axis in DIRS:                             # ① 四轴正入射链
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
                alpha = abs(random.gauss(0, dphi))    # 正入射 + 角噪声
                if axis in self.walls[ck]:
                    hits[(ck, axis)] = (s + random.gauss(0, rng), alpha)
                    occl = True
                else:
                    opens.append((ck, axis, s + random.gauss(0, rng), alpha))
        occluded = False                                # ② 前方侧边对角掠射
        for k in range(1, N + 2):
            if occluded:
                break
            ck = (self.cell[0] + k * hv[0], self.cell[1] + k * hv[1])
            if not (0 <= ck[0] < N and 0 <= ck[1] < N):
                break
            s_far = 0.2 + 0.4 * k - self.o
            if s_far > maxr:
                break
            alpha0 = math.atan2(s_far, P_HALF)          # 射线 vs 侧墙法线
            for sd in DIRS:
                if DIRV[sd] == hv or DIRV[sd] == (-hv[0], -hv[1]):
                    continue
                alpha = alpha0 + random.gauss(0, dphi)
                if sd in self.walls[ck]:
                    hits[(ck, sd)] = (s_far + random.gauss(0, rng), alpha)
                else:
                    opens.append((ck, sd, s_far + random.gauss(0, rng), alpha))
            if self.heading in self.walls[ck]:
                occluded = True
        return hits, opens

    def camera_blocks(self, cam_range=1.5):
        """相机输出: 视野内(前方 cam_range, ±45°锥)的含块格 —— 非上帝视角"""
        seen = []
        hv = DIRV[self.heading]
        for bc in self.blocks - self.collected:
            dx, dy = bc[0] - self.cell[0], bc[1] - self.cell[1]
            along = dx * hv[0] + dy * hv[1]             # 前向格数
            if along < 0:
                continue
            side = abs(dx * hv[1] - dy * hv[0])         # 横向格数
            if along * along + side * side == 0:
                seen.append(bc)
                continue
            if math.hypot(along, side) * C > cam_range:
                continue
            if side > along + 1e-9:                     # >45° 半角
                continue
            seen.append(bc)
        return seen

    def block_at(self, c):
        return c in self.blocks and c not in self.collected

    def collect(self, c):
        self.collected.add(c)


# ---------------- 连续位姿与碰撞检查 ----------------

HALF_W = 0.1075                                   # 车宽 21.5cm 之半 (赛题约束)
HALF_L = 0.145                                    # 车长 ~29cm 之半


def _seg_aabb(ax, ay, bx, by, hl, hw):
    """Liang-Barsky: 线段 (a→b) vs AABB [-hl,hl]×[-hw,hw] 精确相交"""
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax + hl), (dx, hl - ax), (-dy, ay + hw), (dy, hw - ay)):
        if p == 0.0:
            if q < 0.0:
                return False
        else:
            r = q / p
            if p < 0.0:
                if r > t1:
                    return False
                if r > t0:
                    t0 = r
            else:
                if r < t0:
                    return False
                if r < t1:
                    t1 = r
    return True


def collision(px, py, th, wall_segs, margin=0.005):
    """车矩形(OBB) vs 墙线段精确求交: 墙段变换到车体系 → segment-AABB.
       margin = footprint inflation (m), 参数不写死"""
    ca, sa = math.cos(th), math.sin(th)
    hl, hw = HALF_L + margin, HALF_W + margin
    for (x1, y1), (x2, y2) in wall_segs:
        dx1, dy1 = x1 - px, y1 - py
        dx2, dy2 = x2 - px, y2 - py
        ax, ay = dx1 * ca + dy1 * sa, -dx1 * sa + dy1 * ca   # 旋转到车体系
        bx, by = dx2 * ca + dy2 * sa, -dx2 * sa + dy2 * ca
        if _seg_aabb(ax, ay, bx, by, hl, hw):
            return True
    return False


def explore_stream(walls, entry, ex, order, blocks, *,
                   v_cruise=0.70, a_acc=1.0, a_dec=1.0, a_lat=0.7,
                   dphi_deg=0.30, gate=0.06, scan_hz=10.0, proc_ms=5.0,
                   t_spin180=1.0, t_turn90=0.5, t_grab=1.0, ctrl_hz=50.0, v_run=0.60,
                   assume_tree=True, cam_range=1.5,
                   lat_k=3.0, lat_sigma=0.03):
    """流式探索 —— 世界(World) + 认知(StreamNav, 唯一决策核心) + 运动/碰撞.

    诚实性: ①观测带真实入射角α+距离噪声, 置信门限在认知侧;
            ②proc_ms 通过 pending 队列真实延迟;
            ③violations 由连续位姿的足迹-墙线碰撞检查产生;
            ④方块仅相机视野内可见;
            ⑤仿真边界: 语义关联(cell,dir)假设完美(实车由节点层几何反算)."""
    P = C / 2
    v_arc = math.sqrt(a_lat * P)
    dt = 1.0 / ctrl_hz
    scan_every = max(1, round(ctrl_hz / scan_hz))
    lag = 1 + math.ceil(proc_ms * 1e-3 * ctrl_hz)
    TH = {'N': math.pi / 2, 'E': 0.0, 'S': -math.pi / 2, 'W': math.pi}

    world = World(walls, blocks, entry)
    nav = StreamNav(entry, order=order, n=N, v_cruise=v_cruise, a_acc=a_acc,
                    a_dec=a_dec, a_lat=a_lat, dphi_deg=dphi_deg, gate=gate,
                    assume_tree=assume_tree)
    wall_segs = world.wall_segs

    st = {'time': 0.0, 'dist': 0.0, 'got': 0, 'arcs': 0, 'spins': 0,
          'grabs': 0, 'violations': 0, 'sched_t': 0.0, 'obs_new': 0,
          'mark_hit': 0, 'enters': 0}

    # ---- 离散簿记 + 连续位姿 ----
    cell, heading, o, v = entry, 'N', 0.0, 0.0
    world.cell, world.heading, world.o = cell, heading, o
    nav.m.touch(cell)
    e_lat, th_err = 0.0, 0.0                        # 横向误差 / 航向误差 (OU)
    px, py = cell[0] * C + P, cell[1] * C + P       # 连续中心
    th = TH['N']
    cut = None                                       # (p_in, th_in, p_out, th_out, tgt_cell, tgt_hdg, s)
    pending = []                                     # 观测帧延迟队列
    tick = 0

    def near_segs(x, y, r=0.8):
        out = []
        for seg in wall_segs:
            sx0, sx1 = min(seg[0][0], seg[1][0]), max(seg[0][0], seg[1][0])
            sy0, sy1 = min(seg[0][1], seg[1][1]), max(seg[0][1], seg[1][1])
            if sx1 < x - r or sx0 > x + r or sy1 < y - r or sy0 > y + r:
                continue                                # AND 包围盒: 两轴都远才丢
            out.append(seg)
        return out

    def step_pose(o_now):
        """直线连续位姿: 位置 = 走廊中心线(o 参数化) + 横向误差偏移 (无积分漂移)"""
        nonlocal px, py, th, e_lat, th_err
        e_lat += (-lat_k * e_lat) * dt + lat_sigma * math.sqrt(dt) * random.gauss(0, 1)
        th_err += (-4.0 * th_err) * dt + 0.01 * math.sqrt(dt) * random.gauss(0, 1)
        dv = DIRV[heading]
        nx, ny = -dv[1], dv[0]
        bx, by = (cell[0] + 0.5) * C, (cell[1] + 0.5) * C
        px = bx + dv[0] * o_now + nx * e_lat
        py = by + dv[1] * o_now + ny * e_lat
        th = TH[heading] + th_err
        if collision(px, py, th, near_segs(px, py)):
            st['violations'] += 1
            st['viol_line'] = st.get('viol_line', 0) + 1
            e_lat *= 0.3                             # 撞后回中(粗糙恢复)





    def go_home():
        exc = nav.exit_cell
        if exc is None:                              # 出口还没发现: 探索被兜底中断
            return None, 0.0
        reach = {cell}
        q = [cell]
        while q:
            cc = q.pop(0)
            for d, dv in DIRV.items():
                if nav.m.nodes.get(cc, {}).get('edges', {}).get(d) == 'walked':
                    nb = (cc[0] + dv[0], cc[1] + dv[1])
                    if nb not in reach:
                        reach.add(nb)
                        q.append(nb)
        if exc in reach:
            return nav.m.path_between(cell, exc), 0.0
        for d, dv in DIRV.items():
            nb = (exc[0] + dv[0], exc[1] + dv[1])
            if nb in reach and 0 <= nb[0] < N and 0 <= nb[1] < N and \
               nav.m.nodes.get(nb, {}).get('edges', {}).get(OPP[d]) in ('seen', 'walked'):
                return nav.m.path_between(cell, nb), 0.6
        return None, 0.0

    def speed_run(path_dirs):
        """已知路径速度跑 + 逐边碰撞采样"""
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
        # 碰撞采样: 沿路径 5cm 步长 (直线段 + 原地转向角)
        xx, yy = px, py
        ee = e_lat
        for d in path_dirs:
            dv = DIRV[d]
            nx, ny = -dv[1], dv[0]
            for _ in range(int(C / 0.05)):
                ee += (-lat_k * ee) * 0.1 + lat_sigma * math.sqrt(0.1) * random.gauss(0, 1)
                xx += dv[0] * 0.05
                yy += dv[1] * 0.05
                sx, sy = xx + nx * ee, yy + ny * ee
                if collision(sx, sy, TH[d], near_segs(sx, sy)):
                    st['violations'] += 1
                    ee *= 0.3

    def finish():
        unres = 0
        for i in range(N):
            for j in range(N):
                for d, dv in DIRV.items():
                    nb = (i + dv[0], j + dv[1])
                    if not (0 <= nb[0] < N and 0 <= nb[1] < N):
                        continue
                    if not nav.resolved((i, j), d):
                        unres += 1
        st['unresolved'] = unres // 2
        dirs, extra = go_home()
        if dirs is None:
            return st
        speed_run(dirs)
        if extra:
            st['dist'] += extra
            st['time'] += extra / v_run
        return st

    # ---- 初始观测 ----
    hits, opens = world.sense()
    nav.observe(hits, opens)
    for bc in world.camera_blocks(cam_range):
        nav.set_block_seen(bc, True)

    plan = None
    arc_left_plan = None

    while True:
        tick += 1
        st['time'] += dt
        if tick > 250000:                            # 全局看门狗: 仿真必终止
            st['aborted'] = True
            return finish()

        # ---- 早停: 收齐 + 出口已知 → 速度跑回家 ----
        if len(blocks) > 0 and nav.got >= len(blocks) and nav.exit_cell is not None:
            dirs, extra = go_home()
            if dirs is not None:
                speed_run(dirs)
                if extra:
                    st['dist'] += extra
                    st['time'] += extra / v_run
                return st

        # ---- 观测 (proc_ms 真实延迟: 帧 tick+lag 才进认知) ----
        if tick % scan_every == 0:
            pending.append((tick + lag, world.sense(),
                            world.camera_blocks(cam_range)))
        due = [p for p in pending if p[0] <= tick]
        pending = [p for p in pending if p[0] > tick]
        for _, frame, cbs in due:
            st['obs_new'] += nav.observe(*frame)
            for bc in cbs:
                nav.set_block_seen(bc, True)

        # ---- 决策 (StreamNav 唯一决策核心) ----
        if plan is None or plan in ('wait', 'home'):
            p_ = nav.plan_edge(cell, heading)
            if p_ == 'home':
                return finish()
            if p_ == 'wait':
                # 蠕行靠近 (d_conf 调度): 原地等观测几何不变永远等不到,
                # 必须靠近到置信范围让远边过 α 门限. 上限 0.35 不越未确认边界.
                v = min(0.05, math.sqrt(max(0.0, 2 * a_dec * max(0.0, 0.35 - o))))
                o += v * dt
                st['dist'] += v * dt
                step_pose(o)
                world.o = o
                continue
            plan = p_
            if plan['turn_here'] == 'rev':
                st['spins'] += 1
                st['time'] += t_spin180
                e_lat *= 0.1                             # 原地转前对中
                heading = plan['d2']
                world.heading = heading
            elif plan['turn_here'] == 'spin90':
                st['spins90'] = st.get('spins90', 0) + 1
                st['time'] += t_turn90
                heading = plan['d2']
                world.heading = heading
                # 重置为新方向直行 plan: 旧 plan 的 far/d3/far_cut 属于旧走廊, 已失效
                ndv = DIRV[heading]
                plan = {'d2': heading, 'far': (cell[0] + ndv[0], cell[1] + ndv[1]),
                        'end_o': 0.4, 'v_end': v_cruise,
                        'turn_here': None, 'd3': None, 'far_cut': False}

        # ---- 速度调度 (视界) ----
        seg_end, seg_vend = plan['end_o'], plan['v_end']
        far = plan['far']
        if nav.marks.get(far) is None and not nav.cell_classified(far):
            cap = math.sqrt(max(0.0, 2 * a_dec * max(0.0, 0.4 - o - 0.05)))
            if v > cap:
                v = cap
                st['sched_t'] += dt

        # ---- 运动 ----
        if cut is not None:                          # 45° 斜切进行中
            p_in, th_in, p_out, th_out, tgt_cell, tgt_hdg, s = cut
            ds = min(s, v * dt)
            s -= ds
            f = 1.0 - s / 0.2121 if s > 1e-9 else 1.0
            px = p_in[0] + (p_out[0] - p_in[0]) * f
            py = p_in[1] + (p_out[1] - p_in[1]) * f
            th = th_in + (th_out - th_in) * f
            e_lat += (-lat_k * e_lat) * dt + lat_sigma * math.sqrt(dt) * random.gauss(0, 1)
            if collision(px, py, th, near_segs(px, py)):
                st['violations'] += 1
                st['viol_cut'] = st.get('viol_cut', 0) + 1
                e_lat *= 0.3
            st['dist'] += ds
            if s <= 1e-9:
                cut = None
                cell, heading, o, v = tgt_cell, tgt_hdg, 0.15, v
                world.cell, world.heading = cell, heading
                st['enters'] += 1
                st['mark_hit'] += cell in nav.marks
            else:
                cut = (p_in, th_in, p_out, th_out, tgt_cell, tgt_hdg, s)
            continue

        v_allow = math.sqrt(max(0.0, seg_vend ** 2 + 2 * a_dec * max(0.0, seg_end - o)))
        v = max(0.0, min(v + a_acc * dt, v_cruise, v_allow))
        ds = v * dt
        o += ds
        st['dist'] += ds
        step_pose(o)
        world.o = o

        if o >= seg_end - 1e-9:
            o = seg_end
            if nav.has_block(far):
                _nb = (cell[0] + DIRV[heading][0], cell[1] + DIRV[heading][1])
                if not (0 <= _nb[0] < N and 0 <= _nb[1] < N):
                    st['violations'] += 50                 # 边角兜底(已知问题见 TODO)
                    return finish()
                farc = nav.m.walk_edge(cell, heading)
                world.collect(farc)
                nav.set_block_collected(farc)
                nav.got += 1
                st['got'] += 1
                st['grabs'] += 1
                st['time'] += t_grab
                st['enters'] += 1
                st['mark_hit'] += farc in nav.marks
                cell, o, v, plan = farc, 0.0, 0.0, None
                world.cell = cell
                continue
            if plan['far_cut'] and plan['far'] != cell and \
                    0 <= cell[0] + DIRV[heading][0] < N and 0 <= cell[1] + DIRV[heading][1] < N:
                # 45° 斜切: 入弯点 = 走廊交点(farc中心)前 0.15, 出弯点 = 交点后沿 d3 0.15
                # 斜线长 0.2121m, 距内角 0.177m (车侧缘余量 6.9cm)
                e_lat *= 0.1                             # 切弯前对中
                farc = nav.m.walk_edge(cell, heading)    # 走到 far (登记)
                st['enters'] += 1
                st['mark_hit'] += farc in nav.marks
                d3v = DIRV[plan['d3']]
                d2v = DIRV[heading]
                fc = ((farc[0] + 0.5) * C, (farc[1] + 0.5) * C)
                p_out = (fc[0] + d3v[0] * 0.15, fc[1] + d3v[1] * 0.15)
                tgt_cell = nav.m.walk_edge(farc, plan['d3'])
                cut = ((px, py), th, p_out, TH[plan['d3']], tgt_cell, plan['d3'], 0.2121)
                st['arcs'] += 1
                heading = plan['d3']
                world.heading = heading
                cell = farc
                world.cell = cell
                o = 0.25                             # 簿记: cut 完成后置 0.15
                plan = None
                continue
            _nb = (cell[0] + DIRV[heading][0], cell[1] + DIRV[heading][1])
            if not (0 <= _nb[0] < N and 0 <= _nb[1] < N):
                # 边角鲁棒性兜底(已知问题, 见 TODO): 认知漂移到外墙 → 记违规并回家
                st['violations'] += 50
                return finish()
            if plan.get('far_cut') and heading != plan['d2']:
                # far_cut 不可行(边界/簿记异常) → 降级原地转 90° 后沿 d2 走
                st['spins90'] = st.get('spins90', 0) + 1
                st['time'] += t_turn90
                e_lat *= 0.1
                heading = plan['d2']
                world.heading = heading
                plan = None
                continue
            nxtc = nav.m.walk_edge(cell, heading)
            st['enters'] += 1
            st['mark_hit'] += nxtc in nav.marks
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
