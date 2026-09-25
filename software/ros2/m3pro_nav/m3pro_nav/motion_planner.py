#!/usr/bin/env python3
"""MotionPlanner —— MoveIntent + 实时 Pose → MotionPrimitive 队列 (R3 规范 §9).

commit point / 速度 / cut 几何 / 停车位置的唯一 owner. StreamNav 不允许知道这些.
铁律:
  continuation 只来自 intent.preferred_continuation (纯拓扑 peek) ——
  planner 绝不自行发明 continuation (旧 adapter 重选 f_far 是 seed3 越界根因, 已废除);
  intent.preferred_continuation is None → 永不 CUT90.
迟到信息: plan 只发生在节点/当前位姿; 错过 cut entry (已过 commit line) → 不准硬切,
退化为 STRAIGHT_TO_NODE + SPIN90 —— 由几何距离判定, 确定性."""

import math
from .pose import Pose2D, C, DIRV, OPP, TH, nearest_axis
from .motion_primitive import MotionPrimitive

C_ = C
P_HALF = 0.2
CUT_OFF = 0.10          # 切弯切点: 距转角格中心 0.10 —— 弦距内角 0.212 > 车对角 0.1855+margin
CUT_IN = 0.10           # (0.15 时弦距内角仅 0.177, 刮内角 8mm —— 碰撞检查实测)


class MotionPlanner:
    def __init__(self, v_cruise=0.70, a_acc=1.0, a_dec=1.0,
                 t_spin90=0.5, t_spin180=1.0, v_run=0.60,
                 v_creep=0.05, creep_max=0.04):   # 车头(0.145)不得越边界: 0.2-0.145-0.015
        self.v_cruise = v_cruise
        self.a_acc = a_acc
        self.a_dec = a_dec
        self.t_spin90 = t_spin90
        self.t_spin180 = t_spin180
        self.v_run = v_run
        self.v_creep = v_creep
        self.creep_max = creep_max

    # ---- 主规划: 在节点/当前位姿做一次决策 → primitive 链 ----

    def plan(self, pose: Pose2D, intent, far_kind=None):
        """intent: MoveIntent | 'wait'. far_kind: 远格 CellMark kind (调速用)."""
        if intent == 'wait':
            return self.plan_creep(pose)
        prims = []
        cur = pose.copy()
        d2 = intent.next_edge
        heading = nearest_axis(cur.yaw)
        # 原地旋转前必须回中 (车对角 0.18, 偏心旋转会扫到侧墙)
        dv0 = DIRV[heading] if heading else (0.0, 1.0)
        if intent not in ('wait', 'home') and intent.target_cell:
            fc = ((intent.target_cell[0] + 0.5) * C_ - DIRV[d2][0] * C_,
                  (intent.target_cell[1] + 0.5) * C_ - DIRV[d2][1] * C_)  # 当前格中心
        else:
            fc = (math.floor(cur.x / C_) * C_ + 0.5 * C_,
                  math.floor(cur.y / C_) * C_ + 0.5 * C_)
        off = abs(cur.x - fc[0]) + abs(cur.y - fc[1])
        if d2 != heading:
            if off > 0.02:                        # 偏心 → 先直线回中 (全向轮可横退)
                prims.append(MotionPrimitive(
                    kind='STRAIGHT', start_pose=cur.copy(), p0=(cur.x, cur.y),
                    p1=fc, yaw0=cur.yaw, yaw1=cur.yaw,
                    length=off, v_max=0.2, v_end=0.0, meta={'recenter': True}))
                cur = Pose2D(fc[0], fc[1], cur.yaw)
            dur = self.t_spin180 if d2 == OPP[heading] else self.t_spin90
            prims.append(MotionPrimitive(
                kind='SPIN', start_pose=cur.copy(), p0=(cur.x, cur.y),
                yaw0=cur.yaw, yaw1=TH[d2], duration=dur,
                preconditions=('at_node',), meta={'spin': d2}))
            cur = Pose2D(cur.x, cur.y, TH[d2])
        dv = DIRV[d2]
        far_c = ((intent.target_cell[0] + 0.5) * C_, (intent.target_cell[1] + 0.5) * C_)
        cont = intent.preferred_continuation
        if cont is not None and not intent.requires_stop:
            # CUT90 链: 直行到入弯点 (已跨 A/B) → 切弯 (全程在 B 内) → 出弯沿 cont
            cut_in = (far_c[0] - dv[0] * CUT_IN, far_c[1] - dv[1] * CUT_IN)
            s_in = (cut_in[0] - cur.x) * dv[0] + (cut_in[1] - cur.y) * dv[1]
            prims.append(MotionPrimitive(
                kind='STRAIGHT', start_pose=cur.copy(), p0=(cur.x, cur.y),
                p1=cut_in, yaw0=TH[d2], yaw1=TH[d2],
                length=max(1e-6, s_in), v_max=self.v_cruise, v_end=self.v_cruise,
                preconditions=('mark_far_complete',), commit_point=s_in,
                meta={'approach': d2}))
            cut_out = (far_c[0] + DIRV[cont][0] * CUT_OFF,
                       far_c[1] + DIRV[cont][1] * CUT_OFF)
            entry_pose = Pose2D(cut_in[0], cut_in[1], TH[d2])
            prims.append(MotionPrimitive(
                kind='CUT90', start_pose=entry_pose, p0=cut_in, p1=cut_out,
                yaw0=TH[d2], yaw1=TH[cont],
                length=math.hypot(cut_out[0] - cut_in[0], cut_out[1] - cut_in[1]),
                v_max=self.v_cruise, v_end=self.v_cruise,
                preconditions=('mark_far_complete', 'before_commit_line'),
                commit_point=0.0, meta={'cut': (d2, cont)}))
            return prims
        # 远格未分类 → 到远格中心停车重规划 (未知区域不许全速穿越; KnownHorizon R4 优化)
        v_end = 0.0 if (intent.requires_stop or far_kind in ('DEAD', None)) else self.v_cruise
        s_end = (far_c[0] - cur.x) * dv[0] + (far_c[1] - cur.y) * dv[1]
        prims.append(MotionPrimitive(
            kind='STRAIGHT', start_pose=cur.copy(), p0=(cur.x, cur.y),
            p1=far_c, yaw0=TH[d2], yaw1=TH[d2],
            length=max(1e-6, s_end), v_max=self.v_cruise, v_end=v_end,
            preconditions=('edge_open',),
            meta={'grab': intent.target_cell if intent.requires_stop else None}))
        return prims

    # ---- CREEP_OBSERVE: 正式 primitive, 无无限蠕行 ----

    def plan_creep(self, pose, heading=None, node_center=None):
        """朝前方蠕行观察, 最远 creep_max (不越边界). 到线仍 UNKNOWN → STOP."""
        h = heading or nearest_axis(pose.yaw)
        dv = DIRV[h]
        if node_center is None:
            node_center = (math.floor(pose.x / C_) * C_ + 0.5 * C_,
                           math.floor(pose.y / C_) * C_ + 0.5 * C_)
        o0 = (pose.x - node_center[0]) * dv[0] + (pose.y - node_center[1]) * dv[1]
        s_end = max(0.0, self.creep_max - o0)
        if s_end < 1e-6:
            return [MotionPrimitive(
                kind='STOP', start_pose=pose.copy(), p0=(pose.x, pose.y),
                yaw0=pose.yaw, duration=0.2,
                preconditions=('mark_incomplete',),
                cancel_deadline=self.creep_max)]
        target = (pose.x + dv[0] * s_end, pose.y + dv[1] * s_end)
        return [MotionPrimitive(
            kind='CREEP', start_pose=pose.copy(), p0=(pose.x, pose.y),
            p1=target, yaw0=pose.yaw, yaw1=pose.yaw,
            length=s_end, v_max=self.v_creep, v_end=0.0,
            preconditions=('mark_incomplete',),
            cancel_deadline=self.creep_max,   # latest_safe_stop: 不越边界 (0.2)]
            meta={'heading': h})]

    # ---- 返航路线: 已知图逐边 SPIN+STRAIGHT (与探索同一运动模型) ----

    def plan_route(self, pose, seg_dirs, exit_len=0.6, exit_dir=None):
        prims = []
        cur = pose.copy()
        for d in seg_dirs:
            h = nearest_axis(cur.yaw)
            if d != h:
                dur = self.t_spin180 if d == OPP[h] else self.t_spin90
                prims.append(MotionPrimitive(
                    kind='SPIN', start_pose=cur.copy(), p0=(cur.x, cur.y),
                    yaw0=cur.yaw, yaw1=TH[d], duration=dur, meta={'spin': d}))
                cur = Pose2D(cur.x, cur.y, TH[d])
            dv = DIRV[d]
            nxt = (cur.x + dv[0] * C_, cur.y + dv[1] * C_)
            prims.append(MotionPrimitive(
                kind='STRAIGHT', start_pose=cur.copy(), p0=(cur.x, cur.y),
                p1=nxt, yaw0=TH[d], yaw1=TH[d],
                length=C_, v_max=self.v_run, v_end=self.v_run, meta={'route': True}))
            cur = Pose2D(nxt[0], nxt[1], TH[d])
        ed = exit_dir or (seg_dirs[-1] if seg_dirs else 'N')
        hd = nearest_axis(cur.yaw)
        if ed != hd:
            dur = self.t_spin180 if ed == OPP[hd] else self.t_spin90
            prims.append(MotionPrimitive(
                kind='SPIN', start_pose=cur.copy(), p0=(cur.x, cur.y),
                yaw0=cur.yaw, yaw1=TH[ed], duration=dur, meta={'spin': ed}))
            cur = Pose2D(cur.x, cur.y, TH[ed])
        dv = DIRV[ed]
        prims.append(MotionPrimitive(
            kind='STRAIGHT', start_pose=cur.copy(), p0=(cur.x, cur.y),
            p1=(cur.x + dv[0] * exit_len, cur.y + dv[1] * exit_len),
            yaw0=cur.yaw, yaw1=cur.yaw,
            length=exit_len, v_max=self.v_run, v_end=self.v_run,
            meta={'route_exit': True}))
        return prims

