#!/usr/bin/env python3
"""Five small deterministic manoeuvres through the actual control interfaces.

This is a motion-only gate.  It does not run a random maze, infer walls, or
claim calibrated hardware performance.  The lower motor response remains an
explicit uncalibrated assumption until encoder step data is collected.
"""

from dataclasses import asdict, dataclass
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.control_chain import ControlChain
from m3pro_nav.motion_planner import MotionPlanner, validate_geometry
from m3pro_nav.motion_primitive import MotionPrimitive
from m3pro_nav.plant import IdealPlant, LowerLoopPlant, WHEEL_CIRCUMFERENCE_M
from m3pro_nav.pose import Pose2D, C, norm_angle
from runtime_v2 import HALF_L, HALF_W, collision


YAW = math.pi / 2
DT = 0.02  # Upper loop, configured separately from firmware's 0.01 s tick.


@dataclass(frozen=True)
class MotionCase:
    name: str
    primitives: tuple
    start: Pose2D
    route: tuple


@dataclass(frozen=True)
class MotionMetrics:
    name: str
    backend: str
    rms_position_error_m: float
    max_cross_track_error_m: float
    max_yaw_error_rad: float
    max_wheel_rpm: float
    max_wheel_accel_rpm_s: float
    rms_velocity_tracking_error_m_s: float
    min_wall_clearance_m: float
    final_position_error_m: float
    collisions: int


def _line(a, b, vmax=0.7, vend=0.0):
    return MotionPrimitive('STRAIGHT', Pose2D(*a, YAW), p0=a, p1=b,
                           length=math.dist(a, b), v_max=vmax, v_end=vend)


def _stop(point):
    return MotionPrimitive('STOP', Pose2D(*point, YAW), p0=point,
                           duration=0.2)


def _turn_case(direction):
    planner = MotionPlanner()
    nxt = (1, 2) if direction == 'left' else (3, 2)
    outgoing = (0.4, 1.0) if direction == 'left' else (1.6, 1.0)
    arc, end, _ = planner.template((2, 1), (2, 2), nxt,
                                    Pose2D(1.0, 0.8, YAW))
    chain = (_line((1.0, 0.6), (1.0, 0.8), vend=planner.v_arc),
             *arc, _line(end, outgoing, vend=0.0), _stop(outgoing))
    validate_geometry(chain)
    route = ((2, 1), (2, 2), nxt, (0, 2) if direction == 'left' else (4, 2))
    return MotionCase(f'straight-{direction}-straight', chain,
                      Pose2D(1.0, 0.6, YAW), route)


def _combined_case():
    planner = MotionPlanner()
    first, point, _ = planner.template((2, 1), (2, 2), (1, 2),
                                       Pose2D(1.0, 0.8, YAW))
    middle, point, _ = planner.template((2, 2), (1, 2), (0, 2),
                                        Pose2D(*point, YAW))
    second, point, _ = planner.template((1, 2), (0, 2), (0, 3),
                                        Pose2D(*point, YAW))
    second[-1].v_end = 0.15  # Dead-end approach speed ceiling.
    retreat, point, _ = planner.template((0, 2), (0, 3), (0, 2),
                                         Pose2D(*point, YAW))
    retreat[-1].v_end = 0.0
    chain = (_line((1.0, 0.6), (1.0, 0.8), vend=planner.v_arc),
             *first, *middle, *second, *retreat, _stop(point))
    validate_geometry(chain)
    return MotionCase('straight-left-straight-right-reverse', chain,
                      Pose2D(1.0, 0.6, YAW),
                      ((2, 1), (2, 2), (1, 2), (0, 2), (0, 3)))


def cases():
    straight_end = (1.0, 1.4)
    lateral_end = (1.8, 1.0)
    return (
        MotionCase('straight',
                   (_line((1.0, 0.6), straight_end), _stop(straight_end)),
                   Pose2D(1.0, 0.6, YAW), ((2, 1), (2, 2), (2, 3))),
        MotionCase('lateral',
                   (_line((1.0, 1.0), lateral_end), _stop(lateral_end)),
                   Pose2D(1.0, 1.0, YAW), ((2, 2), (3, 2), (4, 2))),
        _turn_case('left'), _turn_case('right'), _combined_case())


def _wall_segments(route, n=7):
    route_edges = {frozenset((a, b)) for a, b in zip(route, route[1:])}
    walls = set()
    dirs = ((0, 1), (1, 0), (0, -1), (-1, 0))
    for cell in route:
        x, y = cell
        for dx, dy in dirs:
            nb = (x + dx, y + dy)
            if frozenset((cell, nb)) in route_edges:
                continue
            if dx == 0:
                ordinate = (y + 1) * C if dy > 0 else y * C
                a, b = (x * C, ordinate), ((x + 1) * C, ordinate)
            else:
                abscissa = (x + 1) * C if dx > 0 else x * C
                a, b = (abscissa, y * C), (abscissa, (y + 1) * C)
            walls.add(tuple(sorted((a, b))))
    return tuple(walls)


def _point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) /
                          (dx * dx + dy * dy)))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _clearance(pose, walls):
    if collision(pose.x, pose.y, pose.yaw, walls, margin=0.0):
        return 0.0
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    corners = tuple((pose.x + c * lx - s * ly, pose.y + s * lx + c * ly)
                    for lx, ly in ((-HALF_L, -HALF_W), (-HALF_L, HALF_W),
                                   (HALF_L, -HALF_W), (HALF_L, HALF_W)))
    best = math.inf
    for a, b in walls:
        for corner in corners:
            best = min(best, _point_segment_distance(corner, a, b))
        for p in (a, b):
            lx = c * (p[0] - pose.x) + s * (p[1] - pose.y)
            ly = -s * (p[0] - pose.x) + c * (p[1] - pose.y)
            best = min(best, math.hypot(max(abs(lx) - HALF_L, 0.0),
                                        max(abs(ly) - HALF_W, 0.0)))
    return best


def run_case(case, backend='lower'):
    plant = (LowerLoopPlant(case.start) if backend == 'lower' else
             IdealPlant(case.start))
    follower = ControlChain(case.primitives, plant, yaw_ref=YAW,
                            upper_period=DT)
    walls = _wall_segments(case.route)
    error_sq = vel_error_sq = 0.0
    max_cross = max_yaw = max_rpm = max_rpm_accel = 0.0
    min_clearance = math.inf
    collisions = count = 0
    previous_rpm = None
    # One second beyond the reference is enough to measure settling; the
    # reference holds its final position with zero feedforward after STOP.
    ticks = math.ceil((follower.profile.duration + 1.0) / DT)
    for _ in range(ticks):
        snap = follower.step(DT)
        pose = snap.plant_step.pose
        ref = snap.reference
        actual = snap.plant_step.twist
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        vx = c * actual.vx - s * actual.vy
        vy = s * actual.vx + c * actual.vy
        ex, ey = pose.x - ref.x, pose.y - ref.y
        error_sq += ex * ex + ey * ey
        vel_error_sq += (vx - ref.vx_world) ** 2 + (vy - ref.vy_world) ** 2
        max_cross = max(max_cross, abs(-ref.tangent_y * ex + ref.tangent_x * ey))
        max_yaw = max(max_yaw, abs(norm_angle(pose.yaw - YAW)))
        rpm = tuple(abs(mm_s / 1000.0 / WHEEL_CIRCUMFERENCE_M * 60.0)
                    for mm_s in snap.plant_step.wheel_actual_mm_s)
        max_rpm = max(max_rpm, *rpm)
        if previous_rpm is not None:
            max_rpm_accel = max(max_rpm_accel,
                                *(abs(a - b) / DT for a, b in zip(rpm, previous_rpm)))
        previous_rpm = rpm
        min_clearance = min(min_clearance, _clearance(pose, walls))
        collisions += collision(pose.x, pose.y, pose.yaw, walls)
        count += 1
    final = follower.target()
    return MotionMetrics(case.name, backend, math.sqrt(error_sq / count), max_cross,
                         max_yaw, max_rpm, max_rpm_accel,
                         math.sqrt(vel_error_sq / count), min_clearance,
                         math.hypot(plant.pose.x - final.x, plant.pose.y - final.y),
                         collisions)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('ideal', 'lower'), default='lower')
    args = parser.parse_args()
    print(json.dumps([asdict(run_case(case, args.backend)) for case in cases()],
                     ensure_ascii=False, indent=2))
