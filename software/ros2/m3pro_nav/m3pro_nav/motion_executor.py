#!/usr/bin/env python3
"""MotionExecutor —— 唯一物理状态 owner (R3 规范 §3/§7).

只拥有: Pose2D / 当前速度 / primitive 队列 / 内部时钟.
其他模块只能读 immutable pose 快照 (step() 返回新帧).
积分只作用于连续位姿; 离散事件由 GridEventDetector 从相邻两帧位姿几何判定."""

import math
from .pose import Pose2D, norm_angle
from .motion_primitive import MotionPrimitive

_EPS = 1e-9


class MotionExecutor:
    def __init__(self, start: Pose2D, a_acc=1.0, a_dec=1.0):
        self.pose = start.copy()
        self.v = 0.0
        self.a_acc = a_acc
        self.a_dec = a_dec
        self.queue = []                    # list[MotionPrimitive] (链式, 首元 start=当前位姿)
        self.time = 0.0

    @property
    def idle(self):
        return not self.queue

    def set_plan(self, prims):
        """装载新计划. 首个 primitive 必须从当前位姿出发 (位姿连续性硬约束)."""
        if prims:
            p0 = prims[0].start_pose
            assert abs(p0.x - self.pose.x) < 1e-6 and abs(p0.y - self.pose.y) < 1e-6, \
                "primitive 链起点与当前位姿不连续"
        self.queue = list(prims)

    def clear(self):
        self.queue = []

    def step(self, dt) -> Pose2D:
        """推进 dt; 返回新位姿快照. 依次消费 primitive 队列."""
        t_left = dt
        while t_left > _EPS and self.queue:
            prim = self.queue[0]
            used = self._advance(prim, t_left)
            t_left -= used
            if prim.done:
                self.queue.pop(0)
                if prim.kind in ('SPIN', 'STOP'):
                    self.v = 0.0
                else:
                    self.v = prim.v_end
        self.time += dt
        return self.pose.copy()

    def _advance(self, prim, dt) -> float:
        """推进 primitive 一拍, 返回实际消耗的时间."""
        if prim.kind in ('STRAIGHT', 'CREEP', 'CUT90'):
            return self._advance_translate(prim, dt)
        if prim.kind == 'SPIN':
            return self._advance_spin(prim, dt)
        if prim.kind == 'STOP':
            used = min(dt, prim.duration - prim.progress)
            prim.progress += used
            if prim.duration - prim.progress <= _EPS:
                prim.done = True
            return used
        raise ValueError(f'unknown primitive kind: {prim.kind}')

    def _advance_translate(self, prim, dt) -> float:
        remaining = prim.length - prim.progress
        if prim.kind == 'CUT90':
            v_t = prim.v_max                      # 切弯定速 (几何余量按设计保证)
        elif prim.kind == 'CREEP':
            v_t = prim.v_max                      # 蠕行定速
        else:
            v_allow = math.sqrt(max(0.0, prim.v_end ** 2 +
                                    2 * self.a_dec * max(0.0, remaining)))
            v_t = min(prim.v_max, v_allow)
        self.v = max(0.0, min(self.v + self.a_acc * dt, v_t))
        ds = self.v * dt
        # 不越过段末: 截断并回算实际用时
        if ds >= remaining - _EPS:
            used = remaining / max(self.v, _EPS) if self.v > _EPS else dt
            prim.progress = prim.length
            prim.done = True
            f = 1.0
        else:
            used = dt
            prim.progress += ds
            f = prim.progress / prim.length if prim.length > _EPS else 1.0
            if prim.length <= _EPS:
                prim.done = True
        self.pose.x = prim.p0[0] + (prim.p1[0] - prim.p0[0]) * f
        self.pose.y = prim.p0[1] + (prim.p1[1] - prim.p0[1]) * f
        if prim.kind == 'CUT90':
            self.pose.yaw = norm_angle(prim.yaw0 + (prim.yaw1 - prim.yaw0) * f)
        else:
            self.pose.yaw = prim.yaw0
        return used

    def _advance_spin(self, prim, dt) -> float:
        used = min(dt, prim.duration - prim.progress)
        prim.progress += used
        f = prim.progress / prim.duration if prim.duration > _EPS else 1.0
        self.pose.yaw = norm_angle(prim.yaw0 + (prim.yaw1 - prim.yaw0) * f)
        self.pose.x, self.pose.y = prim.p0
        if prim.duration - prim.progress <= _EPS:
            prim.done = True
        return used
