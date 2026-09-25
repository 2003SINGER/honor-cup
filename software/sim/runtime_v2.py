#!/usr/bin/env python3
"""runtime_v2 —— R3 事件驱动 SemanticSim 运行时 (Clean-Room, 规范 §11 tick 顺序).

单一物理真相: MotionExecutor 持有 Pose2D; sensor/collision/event-detector 全部
以参数接收同一 pose. 旧的三套位姿账本全部废除, 离散状态只由几何跨越事件产生.

每 tick 固定顺序:
  1. executor.step(dt)               prev_pose → new_pose
  2. collision(world, new_pose)
  3. event_detector(prev, new)       → CrossedEdge 事件
  4. dispatch: on_crossed / on_entered (TraversalMap / CellVisit / 岔路状态机)
  5. sensor(world, new_pose)         → 观测 (scan_hz + proc_ms 延迟队列)
  6. nav.observe()
  7. refresh_branch(当前格)          → 迟分类岔路状态建立
  8. executor.idle → compile_chain / compile_home → 新 primitive 链

禁止出现: 预登记式过边、第二套位姿账本、兼容层决策——见 tests 结构验收."""

import math
import random
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.pose import Pose2D, C, DIRV, OPP, DIRS, TH, nearest_axis
from m3pro_nav.stream_nav import StreamNav
from m3pro_nav.motion_planner import MotionPlanner
from m3pro_nav.motion_executor import MotionExecutor
from m3pro_nav.event_detector import GridEventDetector

OPEN_TRUTH = 'OPEN'
N = 7
P_HALF = 0.2
HALF_W = 0.1075
HALF_L = 0.145


# ---------------- 碰撞 (Liang-Barsky 线段 vs 车体 OBB, margin 参数化) ----------------

def _seg_aabb(ax, ay, bx, by, hl, hw):
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
    ca, sa = math.cos(th), math.sin(th)
    hl, hw = HALF_L + margin, HALF_W + margin
    for (x1, y1), (x2, y2) in wall_segs:
        dx1, dy1 = x1 - px, y1 - py
        dx2, dy2 = x2 - px, y2 - py
        ax, ay = dx1 * ca + dy1 * sa, -dx1 * sa + dy1 * ca
        bx, by = dx2 * ca + dy2 * sa, -dx2 * sa + dy2 * ca
        if _seg_aabb(ax, ay, bx, by, hl, hw):
            return True
    return False


def near_segs(wall_segs, x, y, r=0.8):
    out = []
    for seg in wall_segs:
        sx0, sx1 = min(seg[0][0], seg[1][0]), max(seg[0][0], seg[1][0])
        sy0, sy1 = min(seg[0][1], seg[1][1]), max(seg[0][1], seg[1][1])
        if sx1 < x - r or sx0 > x + r or sy1 < y - r or sy0 > y + r:
            continue
        out.append(seg)
    return out


# ---------------- World (静态环境唯一 owner; 无任何机器人状态) ----------------

class World:
    """只拥有: walls / blocks / collected / 墙线段几何. 禁止持有机器人位姿."""

    def __init__(self, walls, blocks):
        self.walls = walls
        self.blocks = set(blocks)
        self.collected = set()
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

    def collect(self, cell):
        self.collected.add(cell)


# ---------------- Sensor (以 pose 参数消费; 内部投影为派生快照) ----------------

def project_pose(pose: Pose2D):
    """连续位姿 → 离散投影快照 (纯派生, 非状态 owner)"""
    heading = nearest_axis(pose.yaw)
    i = min(max(int(pose.x // C), 0), N - 1)
    j = min(max(int(pose.y // C), 0), N - 1)
    dv = DIRV[heading]
    bx, by = (i + 0.5) * C, (j + 0.5) * C
    o = (pose.x - bx) * dv[0] + (pose.y - by) * dv[1]
    return (i, j), heading, max(-0.19, min(0.19, o))


def sense_from(world, pose, rng=0.02, maxr=2.4, dphi_deg=0.30):
    """雷达一帧 (Tier A SemanticSim: 语义关联假设完美, 实车由 EdgeObserver 反算)"""
    cell, heading, o = project_pose(pose)
    dphi = math.radians(dphi_deg)
    hits, opens = {}, []
    hv = DIRV[heading]
    offs = {heading: o, OPP[heading]: -o}
    for axis in DIRS:
        av = DIRV[axis]
        p_off = offs.get(axis, 0.0)
        occl = False
        for k in range(0, N + 2):
            if occl:
                break
            ck = (cell[0] + k * av[0], cell[1] + k * av[1])
            if not (0 <= ck[0] < N and 0 <= ck[1] < N):
                break
            s = P_HALF + C * k - p_off
            if s > maxr:
                break
            if s < 0.02:                          # 近场墙(贴脸)也必须可确认, 否则 wait 死锁
                continue
            alpha = abs(random.gauss(0, dphi))
            if axis in world.walls[ck]:
                hits[(ck, axis)] = (s + random.gauss(0, rng), alpha)
                occl = True
            else:
                opens.append((ck, axis, s + random.gauss(0, rng), alpha))
    occluded = False
    for k in range(1, N + 2):
        if occluded:
            break
        ck = (cell[0] + k * hv[0], cell[1] + k * hv[1])
        if not (0 <= ck[0] < N and 0 <= ck[1] < N):
            break
        s_far = P_HALF + C * k - o
        if s_far > maxr:
            break
        alpha0 = math.atan2(s_far, P_HALF)
        for sd in DIRS:
            if DIRV[sd] == hv or DIRV[sd] == (-hv[0], -hv[1]):
                continue
            alpha = alpha0 + random.gauss(0, dphi)
            if sd in world.walls[ck]:
                hits[(ck, sd)] = (s_far + random.gauss(0, rng), alpha)
            else:
                opens.append((ck, sd, s_far + random.gauss(0, rng), alpha))
        if heading in world.walls[ck]:
            occluded = True
    return hits, opens


def camera_from(world, pose, cam_range=1.5):
    """相机: 视野内(前方 cam_range, ±45°锥)含块格 —— 非上帝视角"""
    cell, heading, _ = project_pose(pose)
    hv = DIRV[heading]
    seen = []
    for bc in world.blocks - world.collected:
        dx, dy = bc[0] - cell[0], bc[1] - cell[1]
        along = dx * hv[0] + dy * hv[1]
        if along < 0:
            continue
        side = abs(dx * hv[1] - dy * hv[0])
        if along * along + side * side == 0:
            seen.append(bc)
            continue
        if math.hypot(along, side) * C > cam_range:
            continue
        if side > along + 1e-9:
            continue
        seen.append(bc)
    return seen


# ---------------- 运行时 ----------------

def explore(walls, entry, ex, order, blocks, *,
            v_cruise=0.70, a_acc=1.0, a_dec=1.0,
            dphi_deg=0.30, gate=0.06, scan_hz=10.0, proc_ms=5.0,
            t_grab=1.0, ctrl_hz=50.0,
            cam_range=1.5):
    """事件驱动流式探索. 返回统计 dict (诚实性: aborted 标记 = FAILED, 非成功)."""
    dt = 1.0 / ctrl_hz
    scan_every = max(1, round(ctrl_hz / scan_hz))
    lag = 1 + math.ceil(proc_ms * 1e-3 * ctrl_hz)

    world = World(walls, blocks)
    nav = StreamNav(entry, order=order, n=N, dphi_deg=dphi_deg, gate=gate)
    planner = MotionPlanner(v_cruise=v_cruise, a_acc=a_acc, a_dec=a_dec)
    executor = MotionExecutor(Pose2D((entry[0] + 0.5) * C, (entry[1] + 0.5) * C,
                                     TH['N']), a_acc=a_acc, a_dec=a_dec)
    detector = GridEventDetector(n=N)

    st = {'time': 0.0, 'dist': 0.0, 'got': 0, 'arcs': 0,
          'grabs': 0, 'violations': 0, 'enters': 0, 'wait_ticks': 0}

    # 起始: 机器人从场外经入口边进入 entry 格 (入口约定: 底边 S 开口, 朝向 N)
    nav.on_entered(entry, 'S', ts=0.0)

    def _entry_side_of(pose):
        v = nav.visits.get(_cell_of(pose))
        return v.latest_entered_from if v else 'S'

    def finish():
        """收尾: 未确认统计 + 真值对账 + 返航路线装载."""
        unres = 0
        wrong = 0
        for i in range(N):
            for j in range(N):
                for d, dv in DIRV.items():
                    nb = (i + dv[0], j + dv[1])
                    if not (0 <= nb[0] < N and 0 <= nb[1] < N):
                        continue
                    if not nav.resolved((i, j), d):
                        unres += 1
                        continue
                    truth = OPEN_TRUTH if d not in walls[(i, j)] else 'WALL'
                    if nav.edges.state((i, j), d, nav.traversal) != truth:
                        wrong += 1               # 有答案但答错 (canonical 对账)
        st['unresolved'] = unres // 2
        st['wrong_edges'] = wrong
        p = executor.pose
        hr = nav.home_route(_cell_of(p))
        if hr is None:
            st['aborted'] = True              # 无出口候选: FAILED
            return st
        seg, exc, edir = hr
        executor.set_plan(planner.compile_home(nav, p, _cell_of(p),
                                               _entry_side_of(p), seg, edir))
        st['_finishing'] = True
        return None

    def _cell_of(pose):
        return (min(max(int(pose.x // C), 0), N - 1),
                min(max(int(pose.y // C), 0), N - 1))

    # 初始观测
    nav.observe(*sense_from(world, executor.pose, dphi_deg=dphi_deg))
    for bc in camera_from(world, executor.pose, cam_range):
        nav.set_block_seen(bc, True)

    pending = []
    tick = 0

    while True:
        tick += 1
        st['time'] += dt
        if tick > 250000:                     # 看门狗: FAILED 语义, 非成功
            st['aborted'] = True
            finish()
            return st

        # 1. 运动 (位姿唯一 owner)
        prev_pose = executor.pose.copy()
        new_pose = executor.step(dt) if not executor.idle else executor.pose.copy()

        # 2. 碰撞 (同一 pose)
        if collision(new_pose.x, new_pose.y, new_pose.yaw,
                     near_segs(world.wall_segs, new_pose.x, new_pose.y)):
            st['violations'] += 1

        # 3-4. 几何跨越事件 → 离散状态 (唯一通道)
        for ev in detector.detect(prev_pose, new_pose, timestamp=st['time']):
            if 0 <= ev.to_cell[0] < N and 0 <= ev.to_cell[1] < N:
                nav.on_crossed(ev.from_cell, ev.direction, ts=ev.timestamp)
                nav.on_entered(ev.to_cell, OPP[ev.direction], ts=ev.timestamp)
                st['enters'] += 1
            # 场外 to_cell: 仅返航冲出口时发生, 无需登记

        st['dist'] += math.hypot(new_pose.x - prev_pose.x, new_pose.y - prev_pose.y)

        # 返航执行完毕 → 结果
        if st.get('_finishing') and executor.idle:
            return st

        # 5-6. 观测 (延迟队列; 同一 pose)
        if tick % scan_every == 0:
            pending.append((tick + lag, sense_from(world, new_pose, dphi_deg=dphi_deg),
                            camera_from(world, new_pose, cam_range)))
        due = [p for p in pending if p[0] <= tick]
        pending = [p for p in pending if p[0] > tick]
        for _, frame, cbs in due:
            nav.observe(*frame)
            for bc in cbs:
                nav.set_block_seen(bc, True)

        # 7. 迟分类 (当前格 CellMark 完成 → 岔路局部状态机建立)
        cur_cell = _cell_of(new_pose)
        nav.refresh_branch(cur_cell)

        # 早停 (仅在决策点): 任务完成 且 出口候选【可达】→ 回家;
        # 不可达 → 继续探索 (believed-exit 尚未连通, 继续走, 连通后自然回家)
        # 任务完成 = blocks 模式收齐方块 | fullinfo 模式全图每条边已解析
        done = (len(blocks) > 0 and nav.got >= len(blocks)) or \
               (len(blocks) == 0 and nav.all_resolved())
        if executor.idle and done:
            if nav.home_route(cur_cell) is None:
                pass                                 # 出口未连通: 继续探索
            else:
                r = finish()
                if r is not None:
                    return r

        # 8. 空闲 → 规划 (CellAction → 模板编译; 未知格边界中点 STOP)
        if executor.idle:
            # 抓取检查: 到达目标格且有方块
            visit = nav.visits.get(cur_cell)
            entry_side = visit.latest_entered_from if visit else 'S'
            if nav.has_block(cur_cell):
                world.collect(cur_cell)
                nav.set_block_collected(cur_cell)
                nav.got += 1
                st['got'] += 1
                st['grabs'] += 1
                st['time'] += t_grab
            prims = planner.compile_chain(nav, executor.pose, cur_cell, entry_side)
            if prims and prims[0].kind == 'STOP':
                st['wait_ticks'] += 1
            for pm in prims:
                if pm.kind == 'ARC':
                    st['arcs'] += 1
            executor.set_plan(prims)

    return st
