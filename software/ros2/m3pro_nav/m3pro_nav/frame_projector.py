#!/usr/bin/env python3
"""FrameProjector —— laser → base → odom → maze 坐标链 (纯 Python, 无 ROS).

复用 RigidFrameTransform, 不另造第二套二维坐标数学:
  T_base_laser   雷达外参: 有 TF 用 TF, 无 TF 用显式 YAML; 都没有 → ABSTAIN
  T_odom_base    每帧由实测 odom pose 给出 (base 原点在 odom 系的位置)
  T_maze_odom    ManualMazeAnchor 启动时建立一次, 之后车被推动也只跟随
                 odom —— 绝不每帧把车吸回人工指定的格中心/正方向

无外参时 project() 返回 None (调用方对全部 ray 标 NO_TRANSFORM),
禁止假设雷达在机器人中心。"""

from dataclasses import dataclass
import math

from .frame_transform import RigidFrameTransform
from .pose import Pose2D, C, TH, norm_angle

FIELD_N = 7                     # 7×7 正式场地
FIELD_SIZE = FIELD_N * C        # 2.8 m


@dataclass(frozen=True)
class LaserExtrinsic:
    """laser → base 外参; 来源显式记录 (TF / YAML / 缺失)."""
    pose: Pose2D | None         # 雷达在 base_link 下的位姿
    source: str                 # 'tf' / 'yaml' / 'missing'

    @classmethod
    def from_tf(cls, x, y, yaw):
        return cls(Pose2D(float(x), float(y), float(yaw)), 'tf')

    @classmethod
    def from_yaml(cls, x, y, yaw):
        return cls(Pose2D(float(x), float(y), float(yaw)), 'yaml')

    @classmethod
    def missing(cls):
        return cls(None, 'missing')

    @property
    def available(self):
        return self.pose is not None


def maze_pose_from_cell(cell, heading, offset=(0.0, 0.0, 0.0)):
    """人工摆车输入 → maze frame 位姿. cell=(cx,cy), heading=N/E/S/W,
    offset = 可选 (x, y, yaw) 微调 (米/弧度)."""
    cx, cy = int(cell[0]), int(cell[1])
    if not (0 <= cx < FIELD_N and 0 <= cy < FIELD_N):
        raise ValueError(f'cell {cell} outside 7x7 field')
    if heading not in TH:
        raise ValueError(f'heading must be N/E/S/W, got {heading!r}')
    return Pose2D((cx + 0.5) * C + offset[0],
                  (cy + 0.5) * C + offset[1],
                  norm_angle(TH[heading] + offset[2]))


class ManualMazeAnchor:
    """一次性 odom→maze 锚定.

    启动瞬间读实测 odom pose, 与人工给的 maze pose 建立唯一锚
    T_maze_odom; 之后 current_pose(odom_pose) 全部由该锚变换得到,
    车被推 3cm 也如实反映, 不重新吸附格中心."""

    def __init__(self, cell, heading, odom_pose, offset=(0.0, 0.0, 0.0)):
        maze = maze_pose_from_cell(cell, heading, offset)
        self.cell = (int(cell[0]), int(cell[1]))
        self.heading = heading
        self.maze_anchor = maze
        self.odom_anchor = odom_pose.copy()
        self.transform = RigidFrameTransform(odom_pose, maze)

    def maze_pose(self, odom_pose) -> Pose2D:
        """当前 odom pose → maze pose (跟随 odom, 永不重吸附)."""
        return self.transform.transform_pose(odom_pose)


class FrameProjector:
    """把 laser frame 的 ray 投影到 maze frame (每帧给定实测 odom pose)."""

    def __init__(self, extrinsic: LaserExtrinsic):
        if not isinstance(extrinsic, LaserExtrinsic):
            raise ValueError('a LaserExtrinsic is required')
        self.extrinsic = extrinsic

    def project(self, frame, odom_pose, anchor: ManualMazeAnchor):
        """ScanFrame (laser frame) → WorldRay[] (maze frame).

        无外参 → 返回 None (NO_TRANSFORM, 全体 ABSTAIN);
        逐 ray 输出 origin / 单位方向 / hit 点 (maze frame)."""
        if not self.extrinsic.available:
            return None
        if anchor is None:
            return None
        ext = self.extrinsic.pose
        # 合成旋转角: laser→base→odom→maze (全部绕 z 的 2D 刚体)
        odom_yaw = odom_pose.yaw
        rot_maze_odom = anchor.transform.rotation
        total_yaw = ext.yaw + odom_yaw + rot_maze_odom
        c, s = math.cos(total_yaw), math.sin(total_yaw)
        # ray 原点 (laser frame (0,0)) 依序平移: laser→base→odom→maze
        bx, by = ext.x, ext.y
        ox = odom_pose.x + math.cos(odom_yaw) * bx - math.sin(odom_yaw) * by
        oy = odom_pose.y + math.sin(odom_yaw) * bx + math.cos(odom_yaw) * by
        src = anchor.transform.source
        origin = anchor.transform.transform_pose(Pose2D(ox, oy, 0.0))

        rays = []
        for ray in frame.rays:
            if not ray.valid:
                rays.append(WorldRay(ray.index, origin.x, origin.y,
                                     c, s, math.nan, math.nan, math.nan,
                                     ray.invalid_reason))
                continue
            # laser frame 内的 hit 点 + 逐 ray 方向 (旋转到 maze frame)
            ca, sa = math.cos(ray.angle), math.sin(ray.angle)
            dx, dy = c * ca - s * sa, s * ca + c * sa
            hx = origin.x + dx * ray.range
            hy = origin.y + dy * ray.range
            rays.append(WorldRay(ray.index, origin.x, origin.y, dx, dy,
                                 hx, hy, ray.range, None))
        return tuple(rays)


@dataclass(frozen=True)
class WorldRay:
    """maze frame 中的 ray: origin → (origin + dir*range) = hit."""
    index: int
    ox: float
    oy: float
    dir_x: float                 # 单位方向 (余弦分量)
    dir_y: float
    hx: float                    # hit 点; 无效 ray 为 nan
    hy: float
    range: float
    invalid_reason: str | None

    @property
    def valid(self):
        return self.invalid_reason is None
