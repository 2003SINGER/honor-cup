"""Diagnostic-only, robot-local prior for *new WALL promotion*.

This module is intentionally not imported by pose correction.  It turns the
measured near-side-wall visibility into a soft geometric prior and exposes a
separate turn factor for wall promotion.  Missing optional measurements are
neutral; vehicle heading is never mistaken for per-beam incidence.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class TrustPrior:
    diagnostic_only: bool
    wall_side: str
    robot_local_forward_m: float | None
    range_weight: float
    side_visibility_weight: float
    roi_weight: float
    mad_weight: float
    incidence_weight: float
    turn_promotion_weight: float
    geometry_weight: float
    promotion_weight: float
    pose_correction_weight: float = 1.0


def _edge_geometry(edge, cell_size):
    if edge[0] == 'B':
        _, cell, direction = edge
    else:
        cell, direction = edge
    x, y = cell
    if direction == 'E': return 'V', (x + 1) * cell_size, y * cell_size
    if direction == 'W': return 'V', x * cell_size, y * cell_size
    if direction == 'N': return 'H', (y + 1) * cell_size, x * cell_size
    if direction == 'S': return 'H', y * cell_size, x * cell_size
    raise ValueError(f'unknown edge direction: {direction!r}')


def _roi_support(hit):
    roi = hit.get('roi')
    if isinstance(roi, dict):
        # Prefer a narrow measured band; fall back only to a band actually
        # emitted by the producer.  Never synthesize ROI support from full-fit
        # support_points/span_m.
        for key in ('5', '4', '3', '2'):
            item = roi.get(key)
            if isinstance(item, dict):
                n = item.get('longest_contiguous_support_points',
                             item.get('support_points'))
                span = item.get('longest_contiguous_span_m', item.get('span_m'))
                if n is not None and span is not None:
                    return float(n), float(span)
    return None, None


def _continuous_quality(count, span, mad):
    if count is None or span is None:
        roi = 0.0
    else:
        point_score = max(0.0, min(1.0, (count - 4.0) / 8.0))
        span_score = max(0.0, min(1.0, (span - 0.04) / 0.14))
        roi = math.sqrt(point_score * span_score)
    if mad is None:
        mad_score = 1.0
    else:
        # Smoothly penalize noisy normal offsets; no hard cutoff.
        mad_score = math.exp(-max(0.0, mad) / 0.025)
    return roi, mad_score


def _world_support(edge, hit, cell_size):
    tangent = hit.get('roi_tangent_m')
    normal = hit.get('roi_signed_normal_m')
    if not isinstance(tangent, list) or not isinstance(normal, list):
        return []
    orient, grid_normal, _ = _edge_geometry(edge, cell_size)
    if orient == 'V':
        return [(grid_normal + float(n), float(t))
                for t, n in zip(tangent, normal)]
    return [(float(t), grid_normal + float(n))
            for t, n in zip(tangent, normal)]


def _local_forward(edge, hit, pose, cell_size):
    """Median endpoint forward coordinate in the corrected robot frame."""
    points = _world_support(edge, hit, cell_size)
    if points:
        wx, wy = (statistics.median(p[0] for p in points),
                  statistics.median(p[1] for p in points))
    else:
        orient, normal, tangent_start = _edge_geometry(edge, cell_size)
        wx, wy = ((normal, tangent_start + cell_size / 2) if orient == 'V'
                  else (tangent_start + cell_size / 2, normal))
    x, y, yaw = map(float, pose)
    return (wx - x) * math.cos(yaw) + (wy - y) * math.sin(yaw)


def _side_visibility_weight(forward_m):
    """Measured side-wall envelope with cell-center/build tolerance.

    The robot is near a cell center, so current + two forward cells reaches
    about 1.0 m; the next half-cell remains usable with medium confidence.
    """
    if forward_m < -0.20:
        return 0.18
    if forward_m <= 1.00:
        return 1.0
    if forward_m <= 1.20:
        return 0.70 - 0.15 * (forward_m - 1.00) / 0.20
    if forward_m <= 1.60:
        return 0.55 - 0.40 * (forward_m - 1.20) / 0.40
    return 0.12


def evaluate_wall_trust(edge, hit, pose, *, cell_size=0.4,
                        yaw_rate_rad_s=None):
    """Return soft, diagnostic factors for one post-snap edge hit.

    `pose` is corrected (x, y, yaw), in world coordinates.  A wall is a side
    wall when its tangent is closer to robot-forward than to robot-lateral.
    Distance to wall, ROI cluster geometry, MAD, and optional *actual* beam
    incidence contribute to geometry confidence.  Yaw rate affects only the
    promotion weight; pose correction weight is always 1.
    """
    if len(pose) != 3 or not all(math.isfinite(float(v)) for v in pose):
        raise ValueError('pose must contain finite x, y, yaw')
    orient, _, _ = _edge_geometry(edge, cell_size)
    yaw = float(pose[2])
    tangent_angle = math.pi / 2 if orient == 'V' else 0.0
    tangent_forward_alignment = abs(math.cos(tangent_angle - yaw))
    side = tangent_forward_alignment >= math.sqrt(0.5)
    forward = _local_forward(edge, hit, pose, cell_size)
    side_weight = _side_visibility_weight(forward) if side else 1.0

    distance = hit.get('distance_m')
    range_weight = (1.0 if distance is None else
                    math.exp(-max(0.0, float(distance) - 0.6) / 1.35))
    count, span = _roi_support(hit)
    roi_weight, mad_weight = _continuous_quality(
        count, span, hit.get('normal_mad_m'))

    # The merged scan currently lacks source-ray IDs.  Use incidence only
    # when a producer explicitly supplies a beam-level value; heading proxies
    # are deliberately ignored.
    incidence = hit.get('beam_incidence_rad')
    if incidence is None:
        incidence = hit.get('incidence_angle_rad')
    incidence_weight = (1.0 if incidence is None else
                        max(0.05, math.cos(min(math.pi / 2,
                                               abs(float(incidence)))) ** 2))

    if yaw_rate_rad_s is None:
        yaw_rate_rad_s = hit.get('odom_yaw_rate_rad_s')
    turn_factor = 1.0
    if yaw_rate_rad_s is not None:
        rate = abs(float(yaw_rate_rad_s))
        turn_factor = max(0.25, 1.0 - 1.5 * max(0.0, rate - 0.04))

    geometry = (range_weight * side_weight * roi_weight * mad_weight *
                incidence_weight)
    return TrustPrior(
        diagnostic_only=True,
        wall_side='side' if side else 'front',
        robot_local_forward_m=forward,
        range_weight=range_weight,
        side_visibility_weight=side_weight,
        roi_weight=roi_weight,
        mad_weight=mad_weight,
        incidence_weight=incidence_weight,
        turn_promotion_weight=turn_factor,
        geometry_weight=geometry,
        promotion_weight=geometry * turn_factor,
    )
