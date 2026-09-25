#!/usr/bin/env python3
"""MotionExecutor v2 —— 麦轮平移模型 (用户原始设计).

核心前提: 车身朝向 body_yaw 全程固定 (姿态保持环负责), 导航只改变
世界系平移速度矢量 (vx, vy)。转弯 = 沿固定 R=0.2 四分之一圆弧平移,
圆弧与前后直线在入口/出口边中点天然相切, 弧内 |v| = v_arc 恒定,
进出弧速度模长连续。

primitive 只修改连续 Pose 的位置, 绝不修改 yaw / 认知状态。
离散事件由 GridEventDetector 从相邻两帧位姿几何判定。"""

import math
from .pose import Pose2D
from .motion_primitive import MotionPrimitive

_EPS = 1e-9


class MotionExecutor:
    def __init__(self, start: Pose2D, a_acc=1.0, a_dec=1.0):
        self.pose = start.copy()
        self.v = 0.0
        self.a_acc = a_acc
        self.a_dec = a_dec
        self.queue = []
        self.time = 0.0

    @property
    def idle(self):
        return not self.queue

    def set_plan(self, prims):
        if prims:
            p0 = prims[0].start_pose
            assert abs(p0.x - self.pose.x) < 1e-6 and abs(p0.y - self.pose.y) < 1e-6, \
                "primitive 链起点与当前位姿不连续"
        self.queue = list(prims)

    def clear(self):
        self.queue = []

    def step(self, dt) -> Pose2D:
        t_left = dt
        while t_left > _EPS and self.queue:
            prim = self.queue[0]
            used = self._advance(prim, t_left)
            t_left -= used
            if prim.done:
                self.queue.pop(0)
                self.v = prim.v_end
        self.time += dt
        return self.pose.copy()

    def _advance(self, prim, dt) -> float:
        if prim.kind in ('STRAIGHT', 'REVERSE'):
            return self._advance_seg(prim, dt)
        if prim.kind == 'ARC':
            return self._advance_arc(prim, dt)
        if prim.kind == 'STOP':
            used = min(dt, max(0.0, prim.duration - prim.progress))
            prim.progress += used
            if prim.duration - prim.progress <= _EPS:
                prim.done = True
            self.v = 0.0
            return used
        raise ValueError(f'unknown primitive kind: {prim.kind}')

    def _advance_seg(self, prim, dt) -> float:
        remaining = prim.length - prim.progress
        v_allow = math.sqrt(max(0.0, prim.v_end ** 2 +
                                2 * self.a_dec * max(0.0, remaining)))
        v_t = min(prim.v_max, v_allow)
        self.v = max(0.0, min(self.v + self.a_acc * dt, v_t))
        ds = self.v * dt
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
        # body yaw 不由 primitive 修改 (姿态保持环职责, executor 只积分位置)
        return used

    def _advance_arc(self, prim, dt) -> float:
        """四分之一圆弧平移: yaw0 字段存起始极角, yaw1 孫带符号角跨度,
        p0 字段存圆心, meta['r'] 存半径. 速度矢量 = 切向, 模长恒 v_arc."""
        remaining = prim.length - prim.progress
        self.v = max(0.0, min(self.v + self.a_acc * dt, prim.v_max))
        ds = self.v * dt
        if ds >= remaining - _EPS:
            used = remaining / max(self.v, _EPS) if self.v > _EPS else dt
            prim.progress = prim.length
            prim.done = True
        else:
            used = dt
            prim.progress += ds
        ang = prim.yaw0 + prim.yaw1 * (prim.progress / prim.length)
        r = prim.meta['r']
        self.pose.x = prim.p0[0] + r * math.cos(ang)
        self.pose.y = prim.p0[1] + r * math.sin(ang)
        self.pose.yaw = prim.start_pose.yaw            # 车身朝向不变
        return used
