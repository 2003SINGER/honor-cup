"""Read-only online replay of the causal frame-grid snap estimator.

This object owns a diagnostic pose recurrence. It never changes the navigation
anchor, odometry, EdgeMap, or motor command. The caller must time-align each
scan with wheel odometry and run it outside the control callback.
"""

from __future__ import annotations

import math
import time

from .frame_grid_snap import Segment, extract_segments, solve_frame_correction
from .pose import Pose2D, norm_angle


def _compose(a: Pose2D, b: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * b.x - s * b.y,
                  a.y + s * b.x + c * b.y,
                  norm_angle(a.yaw + b.yaw))


def _inverse(a: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(-c * a.x - s * a.y, s * a.x - c * a.y,
                  norm_angle(-a.yaw))


def _as_list(pose: Pose2D) -> list[float]:
    return [pose.x, pose.y, pose.yaw]


class FrameGridShadow:
    """Causal diagnostic recurrence with the same fit/solver as offline replay."""

    def __init__(self, *, max_gap_s: float = 0.8):
        if not math.isfinite(max_gap_s) or max_gap_s <= 0:
            raise ValueError('max_gap_s must be finite and positive')
        self.max_gap_s = max_gap_s
        self.previous_odom: Pose2D | None = None
        self.previous_corrected: Pose2D | None = None
        self.previous_stamp: float | None = None

    def process(self, frame, odom_pose: Pose2D, extrinsic, anchor) -> dict:
        start = time.perf_counter()
        if not math.isfinite(frame.stamp):
            return {'accepted': False, 'reason': 'INVALID_SCAN_STAMP'}
        if extrinsic is None or not extrinsic.available or anchor is None:
            return {'accepted': False, 'reason': 'NO_TRANSFORM'}
        if self.previous_stamp is not None and frame.stamp <= self.previous_stamp:
            return {'accepted': False, 'reason': 'NONMONOTONIC_SCAN_STAMP'}

        reset = (self.previous_stamp is None or
                 frame.stamp - self.previous_stamp > self.max_gap_s)
        if reset:
            predicted = anchor.maze_pose(odom_pose)
        else:
            delta = _compose(_inverse(self.previous_odom), odom_pose)
            predicted = _compose(self.previous_corrected, delta)

        ext = extrinsic.pose
        ce, se = math.cos(ext.yaw), math.sin(ext.yaw)
        local_points = []
        for ray in frame.rays:
            if not ray.valid:
                continue
            a = ray.angle
            lx, ly = ray.range * math.cos(a), ray.range * math.sin(a)
            local_points.append((ext.x + ce * lx - se * ly,
                                 ext.y + se * lx + ce * ly))
        local_segments = extract_segments(local_points)
        cp, sp = math.cos(predicted.yaw), math.sin(predicted.yaw)

        def transform(p):
            return (predicted.x + cp * p[0] - sp * p[1],
                    predicted.y + sp * p[0] + cp * p[1])

        segments = [Segment(transform(s.a), transform(s.b), s.inliers,
                            s.rms, tuple(transform(p) for p in s.support))
                    for s in local_segments]
        if reset:
            corrected = predicted
            accepted, reason, mode = False, 'FIRST_FRAME_ANCHOR', 'REJECTED'
            dx = dy = dyaw = 0.0
            inliers = 0
        else:
            result = solve_frame_correction(segments, predicted)
            corrected = result.corrected_pose if result.accepted else predicted
            accepted, reason, mode = result.accepted, result.reason, result.mode
            dx, dy, dyaw = result.dx, result.dy, result.dyaw
            inliers = result.inlier_wall_count
        self.previous_odom = odom_pose.copy()
        self.previous_corrected = corrected.copy()
        self.previous_stamp = frame.stamp
        return {'accepted': accepted, 'reason': reason, 'mode': mode,
                'predicted_pose': _as_list(predicted),
                'corrected_pose': _as_list(corrected),
                'dx': dx, 'dy': dy, 'dyaw': dyaw,
                'fitted_wall_count': len(segments),
                'inlier_wall_count': inliers,
                'elapsed_ms': (time.perf_counter() - start) * 1000.0}
