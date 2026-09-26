#!/usr/bin/env python3
"""runtime_v2 —— 事件驱动 SemanticSim 运行时 (拓扑游标版, 规范 §11 tick 顺序).

单一物理真相: MotionExecutor 持有 Pose2D; sensor/collision/event-detector 全部
以参数接收同一 pose. 连续 Pose 只服务: 传感/碰撞/运动/跨越事件/定位.
规划拓扑只来自地图图递推 —— runtime 持 planning_cursor (prev_cell, cell),
绝不从 Pose 反推"现在是哪格、从哪格来".

每 tick 固定顺序:
  1. executor.step(dt)               prev_pose → new_pose
  2. collision(world, new_pose)
  3. event_detector(prev, new)       → CrossedEdge 事件
  4. dispatch: on_crossed (walked commit) /
                on_entered (branch commit) / 入格收取 (时间暂停, 位姿不变)
  5. sensor: sense_from (雷达) + block_observe_from (直线连通方块观测)
  6. nav.observe / nav.observe_blocks
  7. refresh_branch (迟分类 commit, 依据 visit 真父)
  8. 新观测可从队尾 STOP 的拓扑游标延长 Action Horizon；空闲时新编链

禁止: 从 Pose 反推规划拓扑 / NUDGE 蹭入 / 新方块清运动链 —— 见 tests 结构验收."""

import math
import random
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.pose import Pose2D, C, DIRV, OPP, DIRS, TH, nearest_axis
from m3pro_nav.stream_nav import StreamNav
from m3pro_nav.motion_planner import MotionPlanner
from m3pro_nav.action_horizon import ActionHorizon
from m3pro_nav.motion_executor import MotionExecutor
from m3pro_nav.event_detector import GridEventDetector
from m3pro_nav.task_pruning import prove_empty_dead_branch

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
    """连续位姿 → 离散投影快照 (纯派生, 非状态 owner; 仅传感用)"""
    heading = nearest_axis(pose.yaw)
    i = min(max(int(pose.x // C), 0), N - 1)
    j = min(max(int(pose.y // C), 0), N - 1)
    dv = DIRV[heading]
    bx, by = (i + 0.5) * C, (j + 0.5) * C
    o = (pose.x - bx) * dv[0] + (pose.y - by) * dv[1]
    return (i, j), heading, max(-0.19, min(0.19, o))


_TOL_LINE = 1e-6


def unambiguous_cell(pose: Pose2D):
    """位姿恰在格线上 → None (floor 分配歧义, GPT 定案: 不得失忆也不得瞎猜);
    严格在格内 → 格坐标. 供 runtime 维护 last_unambiguous + cursor 提示."""
    i = int(pose.x // C)
    j = int(pose.y // C)
    fx = pose.x - i * C
    fy = pose.y - j * C
    if fx < _TOL_LINE or C - fx < _TOL_LINE or fy < _TOL_LINE or C - fy < _TOL_LINE:
        return None
    if not (0 <= i < N and 0 <= j < N):
        return None
    return (i, j)


def sense_from(world, pose, rng=0.02, maxr=2.4, dphi_deg=0.30, cell_hint=None):
    """雷达一帧 (Tier A SemanticSim: 语义关联假设完美, 实车由 EdgeObserver 反算).
    cell_hint: 车恰停在格线上时的观测帧原点 (计划进入格), 消除 floor 歧义."""
    cell, heading, o = project_pose(pose)
    if cell_hint is not None:
        cell = cell_hint
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


def block_observe_from(world, pose, cam_range=1.5, cell_hint=None):
    """方块观测 —— 直线连通可直达格 (GPT 定稿 C1-C5):
    只有从车所在格沿网格直线、沿途每条边真实 OPEN、距离 ≤ cam_range 的格
    才进入观测集; 拐弯/隔墙不可见. 返回 {cell: 'EMPTY'|'BLOCK'} (负信息一等公民)."""
    i = min(max(int(pose.x // C), 0), N - 1)
    j = min(max(int(pose.y // C), 0), N - 1)
    cell = (i, j) if cell_hint is None else cell_hint
    uncollected = world.blocks - world.collected
    obs = {cell: 'BLOCK' if cell in uncollected else 'EMPTY'}
    for d, dv in DIRV.items():
        c = cell
        dist = 0.0
        while d not in world.walls[c]:
            c = (c[0] + dv[0], c[1] + dv[1])
            if not (0 <= c[0] < N and 0 <= c[1] < N):
                break
            dist += C
            if dist > cam_range:
                break
            obs[c] = 'BLOCK' if c in uncollected else 'EMPTY'
    return obs


# ---------------- 运行时 ----------------

def explore(walls, entry, order, blocks, *, required_blocks,
            v_cruise=0.70, a_acc=1.0, a_dec=1.0,
            dphi_deg=0.30, gate=0.06, scan_hz=10.0, proc_ms=5.0,
            t_grab=1.0, ctrl_hz=50.0,
            cam_range=1.5):
    """事件驱动流式探索. 返回统计 dict (诚实性: aborted 标记 = FAILED, 非成功)."""
    if required_blocks < 0:
        raise ValueError('required_blocks must be nonnegative')
    dt = 1.0 / ctrl_hz
    scan_every = max(1, round(ctrl_hz / scan_hz))
    lag = 1 + math.ceil(proc_ms * 1e-3 * ctrl_hz)
    task_mode = required_blocks > 0

    world = World(walls, blocks)
    nav = StreamNav(entry, order=order, n=N, dphi_deg=dphi_deg, gate=gate,
                    task_mode=task_mode)
    planner = MotionPlanner(v_cruise=v_cruise)
    # A 7x7 tree needs at most 2*(N*N-1) edge traversals for a complete DFS.
    # Keep the bounded horizon long enough to end at a real unknown/root STOP,
    # rather than leaving a moving, max_steps-truncated queue tail.
    horizon = ActionHorizon(planner, max_steps=2 * N * N)
    executor = MotionExecutor(Pose2D((entry[0] + 0.5) * C, (entry[1] + 0.5) * C,
                                     TH['N']), a_acc=a_acc, a_dec=a_dec)
    detector = GridEventDetector(n=N)

    st = {'time': 0.0, 'dist': 0.0, 'got': 0, 'arcs': 0, 'grabs': 0,
          'violations': 0, 'enters': 0, 'wait_ticks': 0,
          'topology_mismatch': 0, 'false_prune': 0}
    false_prune_cases = set()

    # 规划游标 (拓扑真相; 车位姿只服务物理层)
    cursor = (None, entry)
    planned_cells = [entry]          # 计划将进入的格序列 (一致性校验用)
    plan_state = {}                  # 已排队动作对应的虚拟 DFS continuation
    last_def_cell = entry            # last_unambiguous_cell (GPT 定案: 不失忆)
    nav.on_entered(entry, None, ts=0.0)   # 根 commit (入口边界 = 来向)

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
        # Derive final proof coverage without mutating navigation during a
        # preview. Count only unvisited cells on a proved empty dead branch.
        proved = set()
        if task_mode:
            for parent in nav.visits.cells:
                for child in nav.open_neighbors(parent):
                    proved.update(prove_empty_dead_branch(nav, parent, child))
        saved = proved - set(nav.visits.cells)
        st['pruned_cells_saved'] = len(saved)
        st['pruned_distance_saved'] = round(2 * C * len(saved), 6)
        hr = nav.home_route(cursor[1])
        if hr is None:
            st['aborted'] = True              # 无出口候选: FAILED
            return st
        path, edir = hr
        executor.set_plan(planner.compile_home(executor.pose, cursor, path, edir))
        st['_finishing'] = True
        return None

    # 初始观测
    nav.observe(*sense_from(world, executor.pose, dphi_deg=dphi_deg))
    nav.observe_blocks(block_observe_from(world, executor.pose, cam_range))

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

        # 3-4. 几何跨越事件 → 离散状态 (唯一通道; 只 commit/verify)
        for ev in detector.detect(prev_pose, new_pose, timestamp=st['time']):
            if 0 <= ev.to_cell[0] < N and 0 <= ev.to_cell[1] < N:
                nav.on_crossed(ev.from_cell, ev.direction, ts=ev.timestamp)
                nav.on_entered(ev.to_cell, ev.from_cell, ts=ev.timestamp)
                st['enters'] += 1
                # 一致性校验: 实际进入格必须符合计划序列 (返航段跳过: 计划已切换)
                if not st.get('_finishing'):
                    if len(planned_cells) >= 2 and ev.to_cell == planned_cells[1]:
                        planned_cells.pop(0)
                    elif ev.to_cell != planned_cells[0]:
                        st['topology_mismatch'] += 1
                # 入格收取 (GPT 定稿): 真实进入含方块格 → 时间暂停, 位姿不变
                if nav.has_block(ev.to_cell):
                    world.collect(ev.to_cell)
                    nav.collect_block(ev.to_cell)
                    st['got'] += 1
                    st['grabs'] += 1
                    st['time'] += t_grab
            # 场外 to_cell: 仅返航冲出口时发生, 无需登记

        st['dist'] += math.hypot(new_pose.x - prev_pose.x, new_pose.y - prev_pose.y)

        # 返航执行完毕 → 结果
        if st.get('_finishing') and executor.idle:
            return st

        # 5-6. 观测 (延迟队列; 同一 pose). 车恰在格线上时需消解 floor 歧义:
        #  - 停在 STOP 边界中点 (等待态)       → 按该 STOP 的格观测
        #  - 链中途接缝 tick (仅 1 tick, 车在动) → last_unambiguous (前一格帧,
        #    关联到的都是真实墙, 无害; 严禁用远处队尾 cursor 帧)
        uc = unambiguous_cell(new_pose)
        if uc is not None:
            last_def_cell = uc
            cell_hint = uc
        else:
            wait = (executor.queue[0] if executor.queue and
                    executor.queue[0].kind == 'STOP' else None)
            if wait is not None:
                cell_hint = wait.meta['wait']
            elif executor.idle:
                cell_hint = cursor[1]
            else:
                cell_hint = last_def_cell
        if tick % scan_every == 0:
            pending.append((tick + lag,
                            sense_from(world, new_pose, dphi_deg=dphi_deg,
                                       cell_hint=cell_hint),
                            block_observe_from(world, new_pose, cam_range,
                                               cell_hint=cell_hint)))
        due = [p for p in pending if p[0] <= tick]
        pending = [p for p in pending if p[0] > tick]
        for _, frame, bobs in due:
            nav.observe(*frame)
            nav.observe_blocks(bobs)

        # 7. 迟分类 commit (依据 visit 真父; 绝不重复登记)
        for cell in list(nav.visits.cells):
            nav.refresh_branch(cell)

        # Tier A truth audit at each decision boundary, while the block may
        # still be uncollected. A later map update cannot erase a bad proof.
        if executor.idle and task_mode:
            for parent in nav.visits.cells:
                for child in nav.open_neighbors(parent):
                    corridor = prove_empty_dead_branch(nav, parent, child)
                    false_prune_cases.update(set(corridor) &
                                             (world.blocks - world.collected))
            st['false_prune'] = len(false_prune_cases)

        # 早停 (仅在决策点): 任务完成 且 出口候选可达 → 回家
        # blocks: 收齐 | 活动前沿清空 (剩余支路全被证明为空死枝)
        # fullinfo: 全图每条边已解析
        if task_mode:
            done = nav.got >= required_blocks or not nav.active_frontier()
        else:
            done = nav.all_resolved()
        if executor.idle and done:
            if nav.home_route(cursor[1]) is None:
                pass                                 # 出口未连通: 继续探索
            else:
                r = finish()
                if r is not None:
                    return r

        # 8. 新信息在车仍运动时就能延长队尾的等待边界。编译从旧队尾的
        #    cursor/pose/虚拟 DFS overlay 继续；executor 只追加，不重编已执行前缀。
        if due and not done and not executor.idle and executor.queue[-1].kind == 'STOP':
            tail_pose = executor.queue[-1].start_pose.copy()
            suffix, terminal, seq, next_state = horizon.compile_with_state(
                nav, tail_pose, cursor, plan_state)
            if suffix and suffix[0].kind != 'STOP':
                executor.extend_plan(suffix)
                cursor = terminal
                planned_cells.extend(seq[1:])
                plan_state = next_state
                st['arcs'] += sum(pm.kind == 'ARC' for pm in suffix)

        # 空闲 → 图递推编链 (cursor 续航, 不从 Pose 反推)
        if executor.idle:
            prims, terminal, seq, plan_state = horizon.compile_with_state(
                nav, executor.pose, cursor)
            cursor = terminal
            planned_cells = seq
            for pm in prims:
                if pm.kind == 'ARC':
                    st['arcs'] += 1
            if prims and prims[0].kind == 'STOP':
                st['wait_ticks'] += 1
            executor.set_plan(prims)

    return st
