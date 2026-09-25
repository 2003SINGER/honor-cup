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

    def __init__(self, walls, blocks, entry=None):
        # R3: World 只拥有静态环境真相, 无任何机器人状态 (位姿由 runtime 传入)
        self.walls = walls
        self.blocks = set(blocks)
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

    def collect(self, c):
        self.collected.add(c)


# ---------------- 连续位姿与碰撞检查 ----------------

class World:
    """物理世界: 真值迷宫 + 真值车位 + 雷达/相机物理模型.
       雷达: 返回 (cell,dir,dist,alpha) —— alpha=射线与墙法线夹角(真实几何+角噪声),
             类别(墙/口)由真值遮挡计算 —— 语义关联 (cell,dir) 由仿真直接给出,
             实车上由节点层几何反算 (仿真边界, 见 README).
       相机: 方块仅在视野内可见 (前方 cam_range 内 ±45° 锥), 非上帝视角."""

    def __init__(self, walls, blocks, entry=None):
        # R3: World 只拥有静态环境真相, 无任何机器人状态 (位姿由 runtime 传入)
        self.walls = walls
        self.blocks = set(blocks)
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
    print(f"{'顺序':<8}{'平均用时(s)':>10}{'最短':>7}{'最长':>7}{'平均路程':>9}{'平均速':>8}{'弧线':>6}{'等待':>6}{'违规':>6}{'未确认':>7}")
    for o_ in orders:
        res = []
        for s in range(a.seeds):
            rnd = random.Random(1000 + s)
            walls, entry, ex, _ = gen_maze(s)
            cells = [(i, j) for i in range(N) for j in range(N) if (i, j) not in (entry, ex)]
            blocks = set() if a.fullinfo else set(rnd.sample(cells, 8))
            r = _v2_explore(walls, entry, ex, o_, blocks, v_cruise=a.vc,
                               dphi_deg=a.dphi, gate=a.gate, scan_hz=a.scan, proc_ms=a.proc)
            res.append(r)
        t = [r['time'] for r in res]
        d = [r.get('dist', 0.0) for r in res]
        print(f"{o_:<8}{st.mean(t):>10.1f}{min(t):>7.1f}{max(t):>7.1f}{st.mean(d):>9.1f}"
              f"{st.mean(d) / st.mean(t):>8.2f}"
              f"{st.mean([r['arcs'] for r in res]):>6.0f}"
              f"{st.mean([r['wait_ticks'] for r in res]):>6.0f}"
              f"{st.mean([r['violations'] for r in res]):>6.1f}"
              f"{st.mean([r.get('unresolved', -1) for r in res]):>6.1f}")
    print("\n对照(同迷宫): pivot@0.3 186.7s · holo@0.3 109.8s · arc@0.3 78.2s (30种子, explore 命令)")
    print("流式 = 观测置信建图+弧线提前承诺+视界调速+处理延迟; 违规>0 说明视界模型过于乐观")


def cmd_maze(a):
    walls, entry, ex, _ = gen_maze(a.seed)
    rnd = random.Random(a.seed + 500)
    cells = [(i, j) for i in range(N) for j in range(N) if (i, j) not in (entry, ex)]
    blocks = set(rnd.sample(cells, 8))
    print(f"迷宫 seed={a.seed}   E=入口(南开口) X=出口 *=方块")
    print(print_maze(walls, entry, ex, blocks))


def _v2_explore(*args, **kw):
    from runtime_v2 import explore
    return explore(*args, **kw)


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
