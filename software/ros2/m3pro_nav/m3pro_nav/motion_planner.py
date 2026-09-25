#!/usr/bin/env python3
"""MotionPlanner —— 三格固定模板编译 (图接口, 2026-09-25 定稿).

公开接口只有格子三元组:

    template(prev_cell, cell, next_cell)
      din  = cell - prev
      dout = next - cell
      dout ==  din  → STRAIGHT   (入口边中点 → 出口边中点)
      dout == -din  → REVERSE    (倒车穿回 prev, 不是 TURN180)
      cross(din,dout) > 0 → LEFT_ARC   R = C/2 = 0.2m 四分之一圆
      cross(din,dout) < 0 → RIGHT_ARC

entry_side/exit_side 只是函数内部为算边中点临时派生的量, 绝不向上暴露.
MotionPlanner 不知道方块 / 剪枝 / DFS —— 它只把图递推给出的 (prev,cell,next)
逐个编译成固定模板.

Action Horizon = 纯图递推:

    while True:
        nxt = nav.resolve_next(prev, cell)
        if nxt is None: 在边界中点 STOP 等地图成熟 (cursor 不变, 零蹭入)
        prev, cell = cell, nxt

已知格动作链不读取 Pose / Visit / 事件. 车还没进入 B 就可以编 A→B→C ——
这正是"小车到格子之前就知道这个格子怎么走"."""

import math
from .pose import Pose2D, C, DIRV
from .motion_primitive import MotionPrimitive

R_ARC = C / 2          # 固定转弯半径: 唯一全局模板参数
V_ARC = 0.45           # 弧内速度 (全局模板参数, 保守)
V_CRUISE = 0.70
V_CONNECT = 0.30       # 连接段低速
GRAB_V = 0.15          # 死路倒车速度


class MotionPlanner:
    def __init__(self, v_cruise=V_CRUISE, v_arc=V_ARC, a_acc=1.0, a_dec=1.0,
                 exit_len=0.6, horizon=64):
        self.v_cruise = v_cruise
        self.v_arc = v_arc
        self.a_acc = a_acc
        self.a_dec = a_dec
        self.exit_len = exit_len
        self.horizon = horizon

    # ---- 单步: (prev, cell, nxt) → 模板 ----

    def _cell_step(self, prev, cell, nxt, cur):
        """编译一格. 返回 (prims, end_point, end_cursor)."""
        din = (cell[0] - prev[0], cell[1] - prev[1]) if prev is not None else (0, 1)
        dout = (nxt[0] - cell[0], nxt[1] - cell[1])
        center = ((cell[0] + 0.5) * C, (cell[1] + 0.5) * C)
        m_in = (center[0] - din[0] * 0.2, center[1] - din[1] * 0.2)
        m_out = (center[0] + dout[0] * 0.2, center[1] + dout[1] * 0.2)
        prims = []

        def seg(p0, p1, v_max, v_end):
            return MotionPrimitive(
                kind='STRAIGHT', start_pose=Pose2D(p0[0], p0[1], 0.0),
                p0=p0, p1=p1, yaw0=0.0, yaw1=0.0,
                length=math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
                v_max=v_max, v_end=v_end)

        # 连接当前位姿 → 本格入口边中点 (模板几何锚点)
        if abs(cur.x - m_in[0]) > 1e-9 or abs(cur.y - m_in[1]) > 1e-9:
            prims.append(seg((cur.x, cur.y), m_in, V_CONNECT,
                             min(self.v_arc, V_CONNECT)))

        if dout == din:
            # STRAIGHT: 后处理统一衔接 (ARC 前降为 v_arc)
            prims.append(seg(m_in, m_out, self.v_cruise, self.v_cruise))
            return prims, m_out, (cell, nxt)

        if dout == (-din[0], -din[1]):
            # REVERSE: 倒车穿过来边回到 prev 格中心 (事件在 prev 触发 commit)
            pc = (center[0] - din[0] * 0.6, center[1] - din[1] * 0.6)
            prims.append(seg(m_in, center, GRAB_V, 0.0))
            prims.append(seg(center, m_in, GRAB_V, GRAB_V))
            prims.append(seg(m_in, pc, GRAB_V, GRAB_V))
            return prims, pc, (cell, prev)

        # 四分之一圆弧: 圆心 = 内角点 (m_in + dout*0.2)
        corner = (m_in[0] + dout[0] * 0.2, m_in[1] + dout[1] * 0.2)
        cross = din[0] * dout[1] - din[1] * dout[0]
        delta = math.pi / 2 if cross > 0 else -math.pi / 2
        a0 = math.atan2(-dout[1], -dout[0])
        prims.append(MotionPrimitive(
            kind='ARC', start_pose=Pose2D(m_in[0], m_in[1], 0.0),
            p0=corner, yaw0=a0, yaw1=delta,
            length=abs(delta) * R_ARC, v_max=self.v_arc, v_end=self.v_arc,
            meta={'r': R_ARC}))
        return prims, m_out, (cell, nxt)

    # ---- 探索动作链: 纯图递推 ----

    def compile_chain(self, nav, pose, cursor):
        """从 cursor (prev_cell, cell) 沿图递推编链.
        返回 (prims, terminal_cursor, plan_seq):
        plan_seq = 本链预期真实进入的格序列 (执行一致性校验用).
        plan_state = 计划内 overlay: 单链多次经过同一 branch 时累积虚拟 done,
        并冻结 preview 入向 (与未来事件 commit 的排序一致)."""
        prims = []
        cur = pose.copy()
        prev, cell = cursor
        seq = [cell]
        plan_state = {}
        if prev is None:
            # 根: 入口方向 bootstrap (从入口边界北上)
            plan_state[cell] = {'done': set(), 'incoming': (0, 1)}

        def is_branch(c):
            mk = nav.mark(c)
            return mk is not None and mk['kind'] == 'BRANCH'

        for _ in range(self.horizon):
            # 计划内 bookkeeping: 从 branch 进入 child / 从 child 返回 branch
            # 都累积为该 branch 的计划内 done (正式 done 由执行期事件 commit)
            if prev is not None and is_branch(prev):
                pst = plan_state.setdefault(prev, {'done': set(), 'incoming': None})
                if pst['incoming'] is None:
                    pst['incoming'] = (cell[0] - prev[0], cell[1] - prev[1])
                parent_dv = pst['incoming']
                if (cell[0] - prev[0], cell[1] - prev[1]) != \
                        (-parent_dv[0], -parent_dv[1]):
                    pst['done'].add(cell)        # child 被本计划取用
            nxt = nav.resolve_next(prev, cell, plan_state)
            if nxt is None:
                if prims and prims[-1].kind == 'STRAIGHT':
                    prims[-1].v_end = 0.0            # 刹车段: 到边界中点停稳
                prims.append(MotionPrimitive(
                    kind='STOP', start_pose=Pose2D(cur.x, cur.y, cur.yaw),
                    p0=(cur.x, cur.y), yaw0=cur.yaw, duration=0.2,
                    meta={'wait': cell}))
                return prims, (prev, cell), seq     # cursor 不变: 等地图成熟再续
            if is_branch(cell):
                pst = plan_state.setdefault(cell, {'done': set(), 'incoming': None})
                if pst['incoming'] is None and prev is not None:
                    pst['incoming'] = (cell[0] - prev[0], cell[1] - prev[1])
            step, end_pt, end_cursor = self._cell_step(prev, cell, nxt, cur)
            prims += step
            cur = Pose2D(end_pt[0], end_pt[1], cur.yaw)
            seq.append(nxt)
            prev, cell = end_cursor
        # 衔接后处理: ARC 前的直线段 v_end = v_arc (速度模长连续)
        for i in range(len(prims) - 1):
            if prims[i + 1].kind == 'ARC' and prims[i].kind == 'STRAIGHT':
                prims[i].v_end = self.v_arc
        return prims, (prev, cell), seq

    # ---- 返航链: 走过图 BFS 格路径 → 同一套三格模板 + 出场段 ----

    def compile_home(self, nav, pose, path_cells, exit_dir):
        """path_cells[0] = 当前格, path[-1] = 出口格. 出场方向 = exit_dir."""
        prims = []
        cur = pose.copy()
        path = list(path_cells)

        def seg(p0, p1, v_max, v_end):
            return MotionPrimitive(
                kind='STRAIGHT', start_pose=Pose2D(p0[0], p0[1], 0.0),
                p0=p0, p1=p1, yaw0=0.0, yaw1=0.0,
                length=math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
                v_max=v_max, v_end=v_end)

        center = ((path[-1][0] + 0.5) * C, (path[-1][1] + 0.5) * C)
        m_out = (center[0] + DIRV[exit_dir][0] * 0.2,
                 center[1] + DIRV[exit_dir][1] * 0.2)
        if len(path) == 1:
            # 车已在出口格: 经格中心两段轴对齐走到出场点 (禁止斜线切角)
            if abs(cur.x - center[0]) > 1e-9 or abs(cur.y - center[1]) > 1e-9:
                prims.append(seg((cur.x, cur.y), center, V_CONNECT, 0.0))
            prims.append(seg(center, m_out, V_CONNECT, min(self.v_arc, V_CONNECT)))
        else:
            for i in range(1, len(path)):
                prev = path[i - 1]
                cell = path[i]
                if i + 1 < len(path):
                    nxt = path[i + 1]
                else:
                    nxt = (cell[0] + DIRV[exit_dir][0], cell[1] + DIRV[exit_dir][1])
                step, end_pt, _ = self._cell_step(prev, cell, nxt, cur)
                prims += step
                cur = Pose2D(end_pt[0], end_pt[1], cur.yaw)
        # 出场段: 沿出口边方向冲出
        dv = DIRV[exit_dir]
        prims.append(MotionPrimitive(
            kind='STRAIGHT', start_pose=Pose2D(m_out[0], m_out[1], cur.yaw),
            p0=m_out, p1=(m_out[0] + dv[0] * self.exit_len,
                          m_out[1] + dv[1] * self.exit_len),
            yaw0=cur.yaw, yaw1=cur.yaw,
            length=self.exit_len, v_max=0.6, v_end=0.6,
            meta={'route_exit': True}))
        # 衔接后处理: ARC 前的直线段 v_end = v_arc (速度模长连续)
        for i in range(len(prims) - 1):
            if prims[i + 1].kind == 'ARC' and prims[i].kind == 'STRAIGHT':
                prims[i].v_end = self.v_arc
        return prims
