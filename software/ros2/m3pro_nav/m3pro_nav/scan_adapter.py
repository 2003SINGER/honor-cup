#!/usr/bin/env python3
"""ScanAdapter —— sensor_msgs/LaserScan → RayObservation[] (纯 Python, 无 ROS).

铁律 (GPT 感知工作台定稿):
  - 保留 beam/ray 语义: 每 ray 同时携带 origin→hit 的 free 路径信息,
    不降维成无语义点云 (free path → OPEN evidence, hit → WALL evidence);
  - NaN / Inf / 越量程绝不进入有效 hit, 每条无效 ray 必须带原因;
  - 保留 time_increment / scan_time, 为后续高速运动 deskew 留接口
    (本轮静态实验不做 deskew);
  - 本模块不 import rclpy —— ROS 消息对象只需具备 LaserScan 字段形状
    (在线节点与离线 bag 重放喂同一形状)。"""

from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class RayObservation:
    """单束雷达 ray (laser frame, 尚未投影到世界系)."""
    index: int
    angle: float                 # laser frame 内的角度 (rad)
    range: float                 # 有效 hit 距离; 无效 ray 为 nan
    valid: bool
    invalid_reason: str | None   # INVALID_RANGE / OUT_OF_RANGE / None


@dataclass(frozen=True)
class ScanFrame:
    """一帧扫描 + 全部元数据 (stamp/时间结构为 deskew 预留)."""
    frame_id: str
    stamp: float                 # header stamp (秒)
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    time_increment: float        # 相邻 beam 时间差 (deskew 预留)
    scan_time: float             # 两帧间隔 (deskew 预留)
    rays: tuple                  # RayObservation[]

    def valid_rays(self):
        return [r for r in self.rays if r.valid]


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def parse_scan(msg) -> ScanFrame:
    """解析 LaserScan 形状对象 → ScanFrame. 数值字段非法按 nan 处理.

    msg 需具备: header.frame_id, header.stamp.sec/.nanosec,
    angle_min/angle_max/angle_increment, range_min/range_max,
    time_increment/scan_time, ranges[]"""
    frame_id = str(getattr(msg.header, 'frame_id', '') or '')
    sec = _num(getattr(msg.header.stamp, 'sec', 0))
    nanosec = _num(getattr(msg.header.stamp, 'nanosec', 0))
    stamp = sec + nanosec * 1e-9
    angle_min = _num(msg.angle_min)
    angle_increment = _num(msg.angle_increment)
    range_min = _num(msg.range_min)
    range_max = _num(msg.range_max)
    time_increment = _num(getattr(msg, 'time_increment', math.nan))
    scan_time = _num(getattr(msg, 'scan_time', math.nan))

    rays = []
    for i, raw in enumerate(msg.ranges):
        r = _num(raw)
        if not math.isfinite(r):
            rays.append(RayObservation(i, angle_min + i * angle_increment,
                                       math.nan, False, 'INVALID_RANGE'))
        elif r < range_min or r > range_max:
            rays.append(RayObservation(i, angle_min + i * angle_increment,
                                       r, False, 'OUT_OF_RANGE'))
        else:
            rays.append(RayObservation(i, angle_min + i * angle_increment,
                                       r, True, None))
    return ScanFrame(frame_id, stamp, angle_min, angle_increment,
                     range_min, range_max, time_increment, scan_time,
                     tuple(rays))
