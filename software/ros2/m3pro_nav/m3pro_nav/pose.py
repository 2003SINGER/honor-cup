#!/usr/bin/env python3
"""Pose2D/Twist2D values shared by motion and observation adapters.

Each execution backend has one Pose owner: MotionExecutor in Tier A semantic
regression, or one ChassisPlant in the deterministic control gate. Sensors,
collision checks, and event detection consume snapshots instead of keeping a
second physical pose ledger.
"""

from dataclasses import dataclass
import math

C = 0.4
DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
DIRS = ('N', 'E', 'S', 'W')
TH = {'N': math.pi / 2, 'E': 0.0, 'S': -math.pi / 2, 'W': math.pi}


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

    def copy(self):
        return Pose2D(self.x, self.y, self.yaw)


@dataclass
class Twist2D:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def nearest_axis(yaw):
    """yaw → 最近的主轴方向 ('N'/'E'/'S'/'W'); 切弯中间态取确定性最近轴"""
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    cand = min(DIRS, key=lambda d: abs(_ang_diff(yaw, TH[d])))
    return cand


def _ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))
