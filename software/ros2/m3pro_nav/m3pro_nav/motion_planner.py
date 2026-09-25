#!/usr/bin/env python3
"""MotionPlanner v2 —— CellAction → 固定轨迹模板 (用户原始设计, GPT 八审定稿).

整个比赛只用一套模板, 禁止按弯形/seed/围墙几何单独调整:
  STRAIGHT          entry 边中点 → exit 边中点 (exit = entry 对边)
  LEFT/RIGHT 1/4圆  R = C/2 = 0.2m, 圆心 = 格子内角点,
                    连接 entry 边中点 → 相邻 exit 边中点, 与直线天然相切
  REVERSE           沿来路反向平移 (死路/边界折返), body yaw 不变
  STOP              v=0 等扫描 (CellMark 未完成时的唯一合法行为)

车身朝向 body_yaw 全程固定; 转弯 = 麦轮平移速度矢量沿圆弧切向连续旋转。
弧内 |v| = v_arc 恒定, 进出弧速度模长连续 (直线段 v_end=v_arc 衔接)。

已知格动作链 (Known Horizon): 从当前位姿沿格子逐格编译模板, 直到
下一格 CellMark 未完成 → 在边界中点 STOP 等扫描, 完成后下一周期自动续。
"""

import math
from .pose import Pose2D, C, DIRV, OPP
from .motion_primitive import MotionPrimitive

R_ARC = C / 2          # 固定转弯半径: 唯一全局模板参数
V_ARC = 0.45           # 弧内速度 (全局模板参数, 保守)
V_CRUISE = 0.70
V_CONNECT = 0.30       # 连接段/抓取段低速
GRAB_V = 0.15
NUDGE = 0.10           # 等扫描前向格内蹭入距离 (脱离格线, 触发离线跨越事件)


class MotionPlanner:
    def __init__(self, v_cruise=V_CRUISE, v_arc=V_ARC, a_acc=1.0, a_dec=1.0,
                 v_grab=GRAB_V, exit_len=0.6, horizon=64):
        self.v_cruise = v_cruise
        self.v_arc = v_arc
        self.a_acc = a_acc
        self.a_dec = a_dec
        self.v_grab = v_grab
        self.exit_len = exit_len
        self.horizon = horizon

    # ---- 单格模板: (entry 边中点 → exit 边中点), 返回 primitive 列表 ----

    def _cell_template(self, cell, entry, exit_side, grab, v_in):
        """几何模板. 返回 (prims, end_point). 调用方保证车已在 m_in."""
        cx = (cell[0] + 0.5) * C
        cy = (cell[1] + 0.5) * C
        m_in = (cx + DIRV[entry][0] * 0.2, cy + DIRV[entry][1] * 0.2)
        m_out = (cx + DIRV[exit_side][0] * 0.2, cy + DIRV[exit_side][1] * 0.2)
        prims = []

        def seg(p0, p1, v_max, v_end):
            return MotionPrimitive(
                kind='STRAIGHT', start_pose=Pose2D(p0[0], p0[1], 0.0),
                p0=p0, p1=p1, yaw0=0.0, yaw1=0.0,
                length=math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
                v_max=v_max, v_end=v_end)

        if grab:
            # 方块格: 入中点 → 格中心停车 (v_end=0, 抓取后重规划)
            prims.append(seg(m_in, (cx, cy), v_in, 0.0))
            return prims, (cx, cy)

        if exit_side == OPP[entry]:
            # 直行: entry 边中点 → exit 边中点 (后处理统一衔接: ARC 前降为 v_arc)
            prims.append(seg(m_in, m_out, self.v_cruise, self.v_cruise))
            return prims, m_out

        if exit_side == entry:
            # REVERSE: 死路折返 —— 倒车穿过父边回到【父格中心】:
            # m_in → 中心 → m_in → 父格中心 (跨越事件在父格触发, 状态机推进)
            pc = (cx + DIRV[entry][0] * 0.4, cy + DIRV[entry][1] * 0.4)
            prims.append(seg(m_in, (cx, cy), self.v_grab, 0.0))
            prims.append(seg((cx, cy), m_in, self.v_grab, self.v_grab))
            prims.append(seg(m_in, pc, self.v_grab, self.v_grab))
            return prims, pc

        # 四分之一圆弧: 圆心 = 内角点 (m_in + exit 方向 0.2)
        corner = (m_in[0] + DIRV[exit_side][0] * 0.2,
                  m_in[1] + DIRV[exit_side][1] * 0.2)
        a0 = math.atan2(m_in[1] - corner[1], m_in[0] - corner[0])
        a1 = math.atan2(m_out[1] - corner[1], m_out[0] - corner[0])
        delta = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi   # ±90°
        prims.append(MotionPrimitive(
            kind='ARC', start_pose=Pose2D(m_in[0], m_in[1], 0.0),
            p0=corner, yaw0=a0, yaw1=delta,
            length=abs(delta) * R_ARC, v_max=self.v_arc, v_end=self.v_arc,
            meta={'r': R_ARC, 'turn': exit_side}))
        return prims, m_out

    # ---- 探索动作链: 沿已知格逐格编译, 未知格在边界中点 STOP 等扫描 ----

    def compile_chain(self, nav, pose, cell, entry_side):
        prims = []
        cur = pose.copy()
        c, e = cell, entry_side
        for _ in range(self.horizon):
            mark = nav.mark(c)
            ex = nav.resolve_exit(c, e) if mark else None
            m_in = ((c[0] + 0.5) * C + DIRV[e][0] * 0.2,
                    (c[1] + 0.5) * C + DIRV[e][1] * 0.2)
            if ex is None:
                # CellMark 未完成: STOP 等扫描 (规范 §10/§11, 异常边界情况).
                # 若车恰停在格线中点 (模板接缝的常态): 必须再向格内蹭 NUDGE
                # 后停车 —— 否则车不离线 → 离线跨越事件不触发 → EnteredCell
                # 缺失 → entry_side 拿不到 → 永久 wait (P0 死锁, 2026-09-25).
                # 蹭的方向 = 行进方向 OPP[e] (已知, 无需 entry_side 之外的真相).
                on_line = abs(cur.x - m_in[0]) < 1e-6 and abs(cur.y - m_in[1]) < 1e-6
                if on_line:
                    dv = DIRV[OPP[e]]
                    p1 = (cur.x + dv[0] * NUDGE, cur.y + dv[1] * NUDGE)
                    prims.append(MotionPrimitive(
                        kind='STRAIGHT', start_pose=Pose2D(cur.x, cur.y, cur.yaw),
                        p0=(cur.x, cur.y), p1=p1, yaw0=cur.yaw, yaw1=cur.yaw,
                        length=NUDGE, v_max=V_CONNECT, v_end=0.0,
                        meta={'nudge_into': c}))
                    cur = Pose2D(p1[0], p1[1], cur.yaw)
                prims.append(MotionPrimitive(
                    kind='STOP', start_pose=Pose2D(cur.x, cur.y, cur.yaw),
                    p0=(cur.x, cur.y), yaw0=cur.yaw, duration=0.2,
                    meta={'wait': c}))
                break
            if nav.is_boundary(c, ex):
                if nav.is_boundary(c, e):
                    break        # 进出皆边界 (入口格探索耗尽): 停车, 早停/返航接管
                ex = e           # 探索永不冲出场: 边界出口 → 原路折返
            grab = nav.has_block(c)
            cell_prims, end_pt = self._cell_template(c, e, ex, grab, self.v_cruise)
            # 连接当前位姿 → 本格入口边中点 (模板几何锚点, 独立于 primitive 形式)
            if abs(cur.x - m_in[0]) > 1e-9 or abs(cur.y - m_in[1]) > 1e-9:
                prims.append(MotionPrimitive(
                    kind='STRAIGHT', start_pose=Pose2D(cur.x, cur.y, cur.yaw),
                    p0=(cur.x, cur.y), p1=m_in, yaw0=cur.yaw, yaw1=cur.yaw,
                    length=math.hypot(m_in[0] - cur.x, m_in[1] - cur.y),
                    v_max=V_CONNECT, v_end=min(self.v_arc, V_CONNECT)))
            prims += cell_prims
            cur = Pose2D(end_pt[0], end_pt[1], cur.yaw)
            if grab:
                break                                 # 抓取停车 → 重规划
            if ex == e:
                break            # REVERSE 已穿过父边 → 事件推进状态机 → 重规划
            e = OPP[ex]
            c = (c[0] + DIRV[ex][0], c[1] + DIRV[ex][1])
        # 衔接后处理: ARC 前的直线段 v_end = v_arc (速度模长连续)
        for i in range(len(prims) - 1):
            if prims[i + 1].kind == 'ARC' and prims[i].kind == 'STRAIGHT':
                prims[i].v_end = self.v_arc
        return prims

    # ---- 返航链: BFS 方向序列 → 同一套模板 + 出场段 ----

    def compile_home(self, nav, pose, cell, entry_side, seg_dirs, exit_dir):
        prims = []
        cur = pose.copy()
        c, e = cell, entry_side
        for d in seg_dirs:
            cell_prims, end_pt = self._cell_template(c, e, d, False, self.v_cruise)
            m_in = ((c[0] + 0.5) * C + DIRV[e][0] * 0.2,
                    (c[1] + 0.5) * C + DIRV[e][1] * 0.2)
            if abs(cur.x - m_in[0]) > 1e-9 or abs(cur.y - m_in[1]) > 1e-9:
                prims.append(MotionPrimitive(
                    kind='STRAIGHT', start_pose=Pose2D(cur.x, cur.y, cur.yaw),
                    p0=(cur.x, cur.y), p1=m_in, yaw0=cur.yaw, yaw1=cur.yaw,
                    length=math.hypot(m_in[0] - cur.x, m_in[1] - cur.y),
                    v_max=V_CONNECT, v_end=min(self.v_arc, V_CONNECT)))
            prims += cell_prims
            cur = Pose2D(end_pt[0], end_pt[1], cur.yaw)
            e = OPP[d]
            c = (c[0] + DIRV[d][0], c[1] + DIRV[d][1])
        # 出口格自身也走同一套模板 (entry→exit_dir); 禁止直线切角 ——
        # 相邻边时弦线会扫过内角墙 (2026-09-25 seed7 碰撞根因)
        cell_prims, end_pt = self._cell_template(c, e, exit_dir, False, self.v_cruise)
        m_in = ((c[0] + 0.5) * C + DIRV[e][0] * 0.2,
                (c[1] + 0.5) * C + DIRV[e][1] * 0.2)
        if abs(cur.x - m_in[0]) > 1e-9 or abs(cur.y - m_in[1]) > 1e-9:
            prims.append(MotionPrimitive(
                kind='STRAIGHT', start_pose=Pose2D(cur.x, cur.y, cur.yaw),
                p0=(cur.x, cur.y), p1=m_in, yaw0=cur.yaw, yaw1=cur.yaw,
                length=math.hypot(m_in[0] - cur.x, m_in[1] - cur.y),
                v_max=V_CONNECT, v_end=min(self.v_arc, V_CONNECT)))
        prims += cell_prims
        cur = Pose2D(end_pt[0], end_pt[1], cur.yaw)
        # 出场段: 沿出口边方向冲出
        dv = DIRV[exit_dir]
        prims.append(MotionPrimitive(
            kind='STRAIGHT', start_pose=Pose2D(cur.x, cur.y, cur.yaw),
            p0=(cur.x, cur.y), p1=(cur.x + dv[0] * self.exit_len,
                                   cur.y + dv[1] * self.exit_len),
            yaw0=cur.yaw, yaw1=cur.yaw,
            length=self.exit_len, v_max=0.6, v_end=0.6,
            meta={'route_exit': True}))
        for i in range(len(prims) - 1):
            if prims[i + 1].kind == 'ARC' and prims[i].kind == 'STRAIGHT':
                prims[i].v_end = self.v_arc
        return prims
