#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HolonomicTracker —— 全向底盘轨迹跟踪控制器（位置环，纯 Python 无 ROS 依赖）

角色：Phase 3 位置环的控制器。输入 = MazeMap 给出的路径（地图系）+ 当前位姿
（雷达校准里程计，reg 模式 RMSE 4.2cm），输出 = 速度指令（车体系）。
麦轮全向特性：vx(前) vy(左) ω 三者独立 → 旋转与平移并行，过弯零停车。

实现要点（踩过的坑）：
- 目标点用【弧长参数化 + 线段投影】取，进度 s 单调不减 —— 顶点版在过冲时
  目标会翻到身后导致乒乓振荡
- 麦轮可边走边转：位置环与航向环同时输出，ω 占用轮速容量按等效臂折算
- 轮速总需求 |vx|+|vy|+|wz|·L 超过下位机钳位(0.7m/s)时按比例缩
"""

import math


class HolonomicTracker:
    def __init__(self, kp_pos=2.0, kp_yaw=3.0,
                 v_max=0.5, w_max=2.0, lookahead=0.18, arrive=0.08):
        self.kp_pos = kp_pos        # 位置误差 → 速度增益 (1/s)
        self.kp_yaw = kp_yaw        # 航向误差 → 角速度增益 (1/s)
        self.v_max = v_max          # 巡航速度上限 (m/s)
        self.w_max = w_max          # 角速度上限 (rad/s)，官方底盘 4.2
        self.lookahead = lookahead  # 前视距离 (m)
        self.arrive = arrive        # 到达判定半径 (m)
        self._path_fp = None                    # 路径内容指纹 (len+首尾+中点), 非 id()
        self._s = 0.0               # 路径进度（弧长，单调不减）
        self._seg = []              # 各段累计弧长
        self._total = 0.0

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _prepare(self, path):
        """新路径: 计算累计弧长; 进度归零"""
        self._seg = []
        total = 0.0
        for (ax, ay), (bx, by) in zip(path[:-1], path[1:]):
            total += math.hypot(bx - ax, by - ay)
            self._seg.append(total)
        self._total = total
        self._s = 0.0

    def _project(self, path, x, y):
        """把车投影到路径上(逐段), 只接受进度 s 之后的投影, 返回 (s, 投影点距离)"""
        s_prev = 0.0
        best_s, best_d = self._s, float('inf')
        for k, ((ax, ay), (bx, by)) in enumerate(zip(path[:-1], path[1:])):
            seg = self._seg[k] - s_prev
            if seg < 1e-9:
                s_prev = self._seg[k]
                continue
            dx, dy = bx - ax, by - ay
            u = ((x - ax) * dx + (y - ay) * dy) / (seg * seg)
            u = max(0.0, min(1.0, u))
            px, py = ax + u * dx, ay + u * dy
            d = math.hypot(x - px, y - py)
            s = s_prev + u * seg
            if s >= self._s - 1e-9 and d < best_d:
                best_s, best_d = s, d
            s_prev = self._seg[k]
        # 路径末端之后的延伸也算候选（过冲时）
        if self._total >= self._s - 1e-9:
            ex, ey = path[-1]
            d = math.hypot(x - ex, y - ey)
            if d < best_d:
                best_s, best_d = self._total, d
        self._s = max(self._s, best_s)          # 进度单调不减
        return best_d

    def _point_at(self, path, s):
        """路径上弧长 s 处的坐标"""
        if s <= 0:
            return path[0]
        if s >= self._total:
            return path[-1]
        s_prev = 0.0
        for k, ((ax, ay), (bx, by)) in enumerate(zip(path[:-1], path[1:])):
            seg = self._seg[k] - s_prev
            if s_prev < s <= self._seg[k] + 1e-9:
                u = (s - s_prev) / seg
                return (ax + u * (bx - ax), ay + u * (by - ay))
            s_prev = self._seg[k]
        return path[-1]

    def update(self, pose, path):
        """pose = (x, y, θ) 世界系, θ 弧度; path = [(x,y), ...] 路径（含终点）.
           路径按内容指纹识别: 原地变异 list 元素也会正确重建弧长缓存.
           返回 (vx, vy, wz, done)：vx=车体前方, vy=车体左方, wz=角速度"""
        x, y, th = pose
        if not path:                                 # 空路径: 安全停, 绝不异常
            return 0.0, 0.0, 0.0, True
        fp = tuple(path)                             # 内容指纹: 任何点变异都会重建缓存
        if fp != self._path_fp:
            self._path_fp = fp
            self._prepare(path)
        if len(path) < 2:
            d = math.hypot(path[-1][0] - x, path[-1][1] - y) if path else 0.0
            return 0.0, 0.0, 0.0, d < self.arrive


        # ---- 进度投影（s 单调不减）----
        proj_d = self._project(path, x, y)

        # ---- 到达判定 ----
        end_d = math.hypot(path[-1][0] - x, path[-1][1] - y)
        if self._s >= self._total - 1e-9 and proj_d < self.arrive and end_d < self.arrive:
            return 0.0, 0.0, 0.0, True

        # ---- 前视目标点（弧长 s + lookahead）----
        tgt = self._point_at(path, self._s + self.lookahead)

        # ---- 位置误差 → 期望速度矢量（世界系）----
        ex, ey = tgt[0] - x, tgt[1] - y
        err = math.hypot(ex, ey)
        speed = min(self.kp_pos * err, self.v_max)
        if err > 1e-6:
            wx, wy = speed * ex / err, speed * ey / err
        else:
            wx, wy = 0.0, 0.0

        # ---- 世界系 → 车体系 ----
        cos_t, sin_t = math.cos(th), math.sin(th)
        vx = cos_t * wx + sin_t * wy
        vy = -sin_t * wx + cos_t * wy

        # ---- 航向控制：朝向运动方向（麦轮边走边转）----
        hd = math.atan2(ey, ex) if err > 1e-6 else th
        wz = max(-self.w_max, min(self.w_max, self.kp_yaw * self._wrap(hd - th)))

        # ---- 轮速容量分配 ----
        total = abs(vx) + abs(vy) + abs(wz) * 0.07
        cap = 0.7                                # 下位机 MECANUM_MAX_SPEED_X
        if total > cap:
            k = cap / total
            vx, vy, wz = vx * k, vy * k, wz * k
        return vx, vy, wz, False


# ---------------- 自检：带噪声跟踪 L 形路径（含 90° 弯） ----------------

if __name__ == '__main__':
    import random
    rnd = random.Random(1)
    tr = HolonomicTracker(v_max=0.35)

    path = [(0.2, 0.2), (0.2, 0.6), (0.2, 1.0), (0.6, 1.0), (1.0, 1.0)]
    pose = [0.2, 0.2, math.pi / 2]
    true = [0.2, 0.2]
    th_true = math.pi / 2
    off = [0.0, 0.0]                          # 定位偏移: OU 有界过程(雷达校准后 ~2cm)
    off_th = 0.0
    dt = 0.05
    t = 0.0
    done = False

    dense = []
    for (ax, ay), (bx, by) in zip(path[:-1], path[1:]):
        n = int(math.hypot(bx - ax, by - ay) / 0.02) + 1
        for k in range(n + 1):
            dense.append((ax + (bx - ax) * k / n, ay + (by - ay) * k / n))

    def path_dev(px, py):
        return min(math.hypot(px - dx_, py - dy_) for dx_, dy_ in dense)

    max_dev = 0.0
    while not done and t < 60:
        vx, vy, wz, done = tr.update(tuple(pose), path)
        wx = math.cos(th_true) * vx - math.sin(th_true) * vy
        wy = math.sin(th_true) * vx + math.cos(th_true) * vy
        true[0] += wx * dt + rnd.gauss(0, 0.002)
        true[1] += wy * dt + rnd.gauss(0, 0.002)
        th_true += wz * dt + rnd.gauss(0, 0.002)
        off[0] = 0.98 * off[0] + rnd.gauss(0, 0.004)
        off[1] = 0.98 * off[1] + rnd.gauss(0, 0.004)
        off_th = 0.98 * off_th + rnd.gauss(0, 0.01)
        pose = [true[0] + off[0], true[1] + off[1], th_true + off_th]
        max_dev = max(max_dev, path_dev(true[0], true[1]))
        t += dt

    end_err = math.hypot(true[0] - path[-1][0], true[1] - path[-1][1])
    print(f"[HolonomicTracker 自检] 用时 {t:.1f}s  终点误差 {end_err*100:.1f}cm  "
          f"路径最大横向偏差 {max_dev*100:.1f}cm  "
          f"{'✅ 收敛' if done and end_err < 0.10 and max_dev < 0.15 else '❌'}")
