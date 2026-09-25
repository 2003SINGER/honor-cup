#!/usr/bin/env python3
"""MotionExecutor v2 —— 麦轮平移模型 (用户原始设计).

核心前提: 车身朝向 body_yaw 全程固定 (姿态保持环负责), 导航只改变
世界系平移速度矢量 (vx, vy)。转弯 = 沿固定 R=0.2 四分之一圆弧平移,
圆弧与前后直线在入口/出口边中点天然相切。弧长进度按加速度限制调速，
终端速度由 primitive.v_end 决定；接直线时保持弧速，接 STOP 时沿弧减速至零。

primitive 只修改连续 Pose 的位置, 绝不修改 yaw / 认知状态。
离散事件由 GridEventDetector 从相邻两帧位姿几何判定。"""

import math
from .pose import Pose2D
from .motion_primitive import MotionPrimitive

_EPS = 1e-9
_DIST_EPS = 1e-6
_SEAM_STOP_DISTANCE = 1e-6


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
                if len(self.queue) > 1 and self._is_linear_reversal(prim, self.queue[1]):
                    seam_speed = math.sqrt(2 * self.a_dec * _SEAM_STOP_DISTANCE)
                    if self.v > seam_speed:
                        raise RuntimeError(
                            f"180-degree reversal reached at nonzero speed {self.v:.6f} m/s")
                    self.v = 0.0
                self.queue.pop(0)
        self.time += dt
        return self.pose.copy()

    def _advance(self, prim, dt) -> float:
        if prim.kind in ('STRAIGHT', 'REVERSE'):
            return self._advance_seg(prim, dt)
        if prim.kind == 'ARC':
            return self._advance_arc(prim, dt)
        if prim.kind == 'STOP':
            seam_speed = math.sqrt(2 * self.a_dec * _SEAM_STOP_DISTANCE)
            if self.v > seam_speed:
                raise RuntimeError(
                    f"STOP reached with nonzero speed {self.v:.6f} m/s")
            # Residuals whose stopping distance is at most one micron are
            # below the motion model's endpoint resolution.
            self.v = 0.0
            used = min(dt, max(0.0, prim.duration - prim.progress))
            prim.progress += used
            if prim.duration - prim.progress <= _EPS:
                prim.done = True
            self.v = 0.0
            return used
        raise ValueError(f'unknown primitive kind: {prim.kind}')

    def _profile_step(self, prim, dt, remaining):
        """Return slew-limited (acceleration, next speed, distance) for one tick.

        The endpoint speed cap solves the stopping inequality after accounting
        for distance travelled at the average speed during this tick.
        """
        v0 = max(0.0, self.v)
        if self.a_dec > _EPS:
            b = self.a_dec * dt
            c = (self.a_dec * dt * v0 - prim.v_end ** 2 -
                 2 * self.a_dec * remaining)
            discriminant = max(0.0, b * b - 4 * c)
            v_allow = max(0.0, (-b + math.sqrt(discriminant)) / 2)
        else:
            v_allow = prim.v_max
        v_target = min(prim.v_max, v_allow)
        if v_target < v0:
            accel = max(-self.a_dec, (v_target - v0) / dt)
        else:
            accel = min(self.a_acc, (v_target - v0) / dt)
        v1 = max(0.0, v0 + accel * dt)
        if accel < 0.0:
            v1 = max(v1, v_target)
        else:
            v1 = min(v1, v_target)
        ds = max(0.0, v0 * dt + 0.5 * accel * dt * dt)
        return accel, v1, ds

    def _terminal_profile(self, prim, remaining, v0, dt):
        """Return the endpoint acceleration/time if v_end is reachable.

        A tiny distance shortfall is accepted only within eight ULPs of the
        accumulated primitive distance. This accounts for floating-point
        summation in progress without masking physically infeasible braking.
        """
        if remaining <= _EPS or v0 + prim.v_end <= _EPS:
            return None
        accel = (prim.v_end ** 2 - v0 ** 2) / (2 * remaining)
        distance_tol = 8 * math.ulp(max(abs(prim.length), abs(prim.progress), remaining))
        if accel < -self.a_dec:
            required = (v0 ** 2 - prim.v_end ** 2) / (2 * self.a_dec) if self.a_dec > _EPS else math.inf
            if required - remaining > distance_tol:
                return None
            accel = -self.a_dec
        elif accel > self.a_acc:
            required = (prim.v_end ** 2 - v0 ** 2) / (2 * self.a_acc) if self.a_acc > _EPS else math.inf
            if required - remaining > distance_tol:
                return None
            accel = self.a_acc
        used = 2 * remaining / (v0 + prim.v_end)
        if used > dt + 1e-9:
            return None
        return accel, used

    @staticmethod
    def _is_linear_reversal(first, second):
        if first.kind not in ('STRAIGHT', 'REVERSE') or second.kind not in ('STRAIGHT', 'REVERSE'):
            return False
        if first.p0 is None or first.p1 is None or second.p0 is None or second.p1 is None:
            return False
        ax, ay = first.p1[0] - first.p0[0], first.p1[1] - first.p0[1]
        bx, by = second.p1[0] - second.p0[0], second.p1[1] - second.p0[1]
        a_norm = math.hypot(ax, ay)
        b_norm = math.hypot(bx, by)
        if a_norm <= _EPS or b_norm <= _EPS:
            return False
        return ax * bx + ay * by < 0 and abs(ax * by - ay * bx) <= _EPS * a_norm * b_norm

    def _advance_seg(self, prim, dt) -> float:
        remaining = max(0.0, prim.length - prim.progress)
        speed_delta = prim.v_end - self.v
        terminal_accel = self.a_acc if speed_delta > 0 else self.a_dec
        if (remaining <= _DIST_EPS and
                abs(speed_delta) <= terminal_accel * dt + 1e-9):
            used = abs(speed_delta) / max(terminal_accel, _EPS)
            prim.progress = prim.length
            prim.done = True
            self.v = prim.v_end
            self.pose.x, self.pose.y = prim.p1
            return min(dt, used)
        v0 = self.v
        accel, v1, ds = self._profile_step(prim, dt, remaining)
        if ds >= remaining:
            # The tick's profile may only apply for part of dt before the
            # endpoint. Re-solve acceleration over that partial interval so
            # endpoint speed is physically reachable at the exact endpoint.
            terminal = self._terminal_profile(prim, remaining, v0, dt)
            if terminal is None:
                if abs(accel) <= _EPS:
                    used = remaining / max(v0, _EPS)
                else:
                    disc = max(0.0, v0 * v0 + 2 * accel * remaining)
                    used = (-v0 + math.sqrt(disc)) / accel
                used = min(dt, max(0.0, used))
                v1 = math.sqrt(max(0.0, v0 * v0 + 2 * accel * remaining))
            else:
                accel, used = terminal
                v1 = prim.v_end
            prim.progress = prim.length
            prim.done = True
            self.v = v1
            f = 1.0
        else:
            used = dt
            prim.progress += ds
            self.v = v1
            f = prim.progress / prim.length if prim.length > _EPS else 1.0
            if prim.length <= _EPS:
                prim.done = True
        self.pose.x = prim.p0[0] + (prim.p1[0] - prim.p0[0]) * f
        self.pose.y = prim.p0[1] + (prim.p1[1] - prim.p0[1]) * f
        # body yaw 不由 primitive 修改 (姿态保持环职责, executor 只积分位置)
        return used

    def _advance_arc(self, prim, dt) -> float:
        """四分之一圆弧平移: yaw0 字段存起始极角, yaw1 存带符号角跨度,
        p0 字段存圆心, meta['r'] 存半径. 切向速度按弧长加减速, 不改变弧几何."""
        remaining = max(0.0, prim.length - prim.progress)
        v0 = self.v
        accel, v1, ds = self._profile_step(prim, dt, remaining)
        # Do not use a distance epsilon to declare the endpoint reached:
        # at very low speed it can represent a meaningful stopping distance
        # (seed 153 exposed a ~1.7e-8 m gap with ~1.9e-4 m/s still on the arc).
        if ds >= remaining:
            terminal = self._terminal_profile(prim, remaining, v0, dt)
            if terminal is None:
                if abs(accel) <= _EPS:
                    used = remaining / max(v0, _EPS)
                else:
                    disc = max(0.0, v0 * v0 + 2 * accel * remaining)
                    used = (-v0 + math.sqrt(disc)) / accel
                used = min(dt, max(0.0, used))
                v1 = math.sqrt(max(0.0, v0 * v0 + 2 * accel * remaining))
            else:
                accel, used = terminal
                v1 = prim.v_end
            prim.progress = prim.length
            prim.done = True
            self.v = v1
        else:
            used = dt
            prim.progress += ds
            self.v = v1
        ang = prim.yaw0 + prim.yaw1 * (prim.progress / prim.length)
        r = prim.meta['r']
        self.pose.x = prim.p0[0] + r * math.cos(ang)
        self.pose.y = prim.p0[1] + r * math.sin(ang)
        # body yaw 不由 primitive 修改 (姿态保持环职责, executor 只积分位置)
        return used
