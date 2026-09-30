"""Bounded, offline pose correction from explicitly confirmed wall segments.

This module only returns a fit proposal. Callers must decide whether and when
to apply it; it has no map-writing, ROS, odometry, or motion-control side
effects. Endpoints must be actual scan hits projected using the supplied pose.
"""

from dataclasses import dataclass
import math
from typing import Iterable

from .pose import Pose2D, norm_angle


@dataclass(frozen=True)
class ProjectedEndpoint:
    x: float
    y: float
    is_hit: bool = True


@dataclass(frozen=True)
class KnownWallSegment:
    """One straight, surveyed/confirmed wall piece in maze coordinates.

    Split crooked walls into independently calibrated straight pieces. A wall
    discovered by the current scan must never be passed here.
    """

    start: tuple[float, float]
    end: tuple[float, float]
    confirmed: bool = False
    wall_id: str = ''

    def __post_init__(self):
        values = (*self.start, *self.end)
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError('wall endpoints must be finite')
        if math.hypot(self.end[0] - self.start[0],
                      self.end[1] - self.start[1]) < 1e-6:
            raise ValueError('wall segment must have nonzero length')


@dataclass(frozen=True)
class PoseCorrectionConfig:
    min_hits: int = 8
    min_hits_per_wall: int = 3
    association_gate_m: float = 0.12
    inlier_gate_m: float = 0.035
    max_median_residual_m: float = 0.018
    max_p90_residual_m: float = 0.04
    min_inlier_fraction: float = 0.60
    min_wall_inlier_fraction: float = 0.50
    min_wall_angle_deg: float = 15.0
    max_translation_m: float = 0.10
    max_yaw_rad: float = math.radians(5.0)
    huber_delta_m: float = 0.015
    max_iterations: int = 12

    def __post_init__(self):
        if self.min_hits < 3 or self.min_hits_per_wall < 1:
            raise ValueError('hit-count limits must be positive')
        positive = (self.association_gate_m, self.inlier_gate_m,
                    self.max_median_residual_m, self.max_p90_residual_m,
                    self.min_wall_angle_deg, self.max_translation_m,
                    self.max_yaw_rad, self.huber_delta_m)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError('distance, angle, and robust-fit limits must be positive')
        if self.inlier_gate_m > self.association_gate_m:
            raise ValueError('inlier gate cannot exceed association gate')
        if self.max_median_residual_m > self.max_p90_residual_m:
            raise ValueError('median residual limit cannot exceed p90 limit')
        if not 0 < self.min_wall_angle_deg < 90:
            raise ValueError('minimum wall angle must be below 90 degrees')
        if self.max_yaw_rad > math.pi:
            raise ValueError('maximum yaw correction cannot exceed pi')
        if not 0 < self.min_inlier_fraction <= 1:
            raise ValueError('min_inlier_fraction must be in (0, 1]')
        if not 0 < self.min_wall_inlier_fraction <= 1:
            raise ValueError('min_wall_inlier_fraction must be in (0, 1]')
        if self.max_iterations < 1:
            raise ValueError('max_iterations must be positive')


@dataclass(frozen=True)
class PoseCorrectionResult:
    accepted: bool
    reason: str
    corrected_pose: Pose2D | None = None
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    n_input_hits: int = 0
    n_associated: int = 0
    n_inliers: int = 0
    n_walls: int = 0
    median_residual_m: float | None = None
    p90_residual_m: float | None = None


def _solve3(a, b):
    """Solve a small dense 3x3 system with scaled partial pivoting."""
    m = [list(map(float, row)) + [float(rhs)] for row, rhs in zip(a, b)]
    scale = max((abs(v) for row in a for v in row), default=0.0)
    if scale == 0:
        return None
    pivots = []
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        value = abs(m[pivot][col])
        if value <= scale * 1e-10:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        pivots.append(value)
        divisor = m[col][col]
        m[col] = [v / divisor for v in m[col]]
        for row in range(3):
            if row == col:
                continue
            factor = m[row][col]
            m[row] = [x - factor * y for x, y in zip(m[row], m[col])]
    if min(pivots) / max(pivots) < 1e-9:
        return None
    result = [m[i][3] for i in range(3)]
    return result if all(math.isfinite(v) for v in result) else None


def _quantile(values, q):
    vals = sorted(values)
    if not vals:
        return math.inf
    pos = (len(vals) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(vals) - 1)
    return vals[lo] + (vals[hi] - vals[lo]) * (pos - lo)


def _signed_distance_and_tangent(point, wall):
    ax, ay = wall.start
    vx, vy = wall.end[0] - ax, wall.end[1] - ay
    length = math.hypot(vx, vy)
    tx, ty = vx / length, vy / length
    nx, ny = -ty, tx
    px, py = point
    along = (px - ax) * tx + (py - ay) * ty
    residual = (px - ax) * nx + (py - ay) * ny
    return residual, along, length, (nx, ny), (tx, ty)


def _diverse_normals(walls, min_angle_deg):
    normals = []
    for wall in walls:
        vx = wall.end[0] - wall.start[0]
        vy = wall.end[1] - wall.start[1]
        n = (-vy / math.hypot(vx, vy), vx / math.hypot(vx, vy))
        if all(abs(n[0] * m[0] + n[1] * m[1]) <
               math.cos(math.radians(min_angle_deg)) for m in normals):
            normals.append(n)
    return len(normals) >= 2


def propose_pose_correction(endpoints: Iterable[ProjectedEndpoint],
                            walls: Iterable[KnownWallSegment],
                            current_pose: Pose2D,
                            config: PoseCorrectionConfig | None = None
                            ) -> PoseCorrectionResult:
    """Fit a small pose delta from scan hits to known finite wall segments.

    Endpoint coordinates are already projected into the maze frame using
    ``current_pose``. The fitted rigid delta is applied about the robot base
    origin at that pose. Initial correspondences are frozen within an
    association gate; endpoints with ambiguous equally-close walls abstain.
    """
    cfg = config or PoseCorrectionConfig()
    if not all(math.isfinite(v) for v in
               (current_pose.x, current_pose.y, current_pose.yaw)):
        return PoseCorrectionResult(False, 'INVALID_CURRENT_POSE')
    points = [(float(p.x), float(p.y)) for p in endpoints
              if p.is_hit and math.isfinite(p.x) and math.isfinite(p.y)]
    wall_list = tuple(walls)
    if not all(w.confirmed for w in wall_list):
        return PoseCorrectionResult(False, 'UNCONFIRMED_WALL')
    if len(points) < cfg.min_hits:
        return PoseCorrectionResult(False, 'INSUFFICIENT_HITS',
                                    n_input_hits=len(points))
    if not _diverse_normals(wall_list, cfg.min_wall_angle_deg):
        return PoseCorrectionResult(False, 'INSUFFICIENT_WALL_DIRECTIONS',
                                    n_input_hits=len(points))

    # Assign each endpoint to one confirmed segment using its initial pose.
    pairs = []
    for p in points:
        candidates = []
        for wall in wall_list:
            residual, along, length, normal, tangent = \
                _signed_distance_and_tangent(p, wall)
            if -cfg.association_gate_m <= along <= length + cfg.association_gate_m:
                candidates.append((abs(residual), wall, normal, tangent))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > cfg.association_gate_m:
            continue
        if (len(candidates) > 1 and
                candidates[1][0] - candidates[0][0] < 0.005):
            continue
        _, wall, normal, tangent = candidates[0]
        pairs.append((p, wall, normal, tangent))
    if len(pairs) < cfg.min_hits:
        return PoseCorrectionResult(False, 'INSUFFICIENT_ASSOCIATIONS',
                                    n_input_hits=len(points),
                                    n_associated=len(pairs))

    wall_counts = {}
    for _, wall, _, _ in pairs:
        wall_counts[wall] = wall_counts.get(wall, 0) + 1
    supporting = [w for w, count in wall_counts.items()
                  if count >= cfg.min_hits_per_wall]
    if not _diverse_normals(supporting, cfg.min_wall_angle_deg):
        return PoseCorrectionResult(False, 'INSUFFICIENT_WALL_SUPPORT',
                                    n_input_hits=len(points),
                                    n_associated=len(pairs),
                                    n_walls=len(supporting))
    pairs = [pair for pair in pairs if pair[1] in supporting]
    pair_counts = {}
    for _, wall, _, _ in pairs:
        pair_counts[wall] = pair_counts.get(wall, 0) + 1

    # Optimize [dx, dy, dtheta * scale] so translation and angular columns
    # have comparable units. Linearize transformed points at each iteration.
    angular_scale = 0.4
    dx = dy = dyaw = 0.0
    inliers = []
    for iteration in range(cfg.max_iterations):
        c, s = math.cos(dyaw), math.sin(dyaw)
        normal_matrix = [[0.0] * 3 for _ in range(3)]
        rhs = [0.0] * 3
        residuals = []
        for p, wall, normal, _ in pairs:
            rx, ry = p[0] - current_pose.x, p[1] - current_pose.y
            qx = current_pose.x + dx + c * rx - s * ry
            qy = current_pose.y + dy + s * rx + c * ry
            r, along, length, _, _ = _signed_distance_and_tangent((qx, qy), wall)
            if not -cfg.association_gate_m <= along <= length + cfg.association_gate_m:
                continue
            residuals.append((r, p, wall, normal, rx, ry, c, s))
        # Use the gated correspondences with Huber weights during fitting.
        # A strict inlier-only fit at the initial pose cannot recover when its
        # bounded starting error is larger than the final residual threshold.
        solve_items = residuals
        if iteration > 0:
            solve_items = [item for item in residuals
                           if abs(item[0]) <= cfg.inlier_gate_m]
        if len(solve_items) < cfg.min_hits:
            break
        for r, p, wall, (nx, ny), rx, ry, c, s in solve_items:
            # d(R(theta)r)/dtheta, followed by angular column scaling.
            j = (nx, ny, (nx * (-s * rx - c * ry) +
                          ny * (c * rx - s * ry)) / angular_scale)
            # Huber limits each return's influence. Normalize by wall support
            # as well so a densely sampled side cannot drown out a sparse
            # nonparallel wall that is essential for observability.
            weight = min(1.0, cfg.huber_delta_m / max(abs(r), 1e-12))
            weight /= pair_counts[wall]
            for i in range(3):
                rhs[i] += -weight * j[i] * r
                for k in range(3):
                    normal_matrix[i][k] += weight * j[i] * j[k]
        step = _solve3(normal_matrix, rhs)
        if step is None:
            return PoseCorrectionResult(False, 'UNOBSERVABLE',
                                        n_input_hits=len(points),
                                        n_associated=len(pairs),
                                        n_inliers=sum(abs(item[0]) <=
                                                      cfg.inlier_gate_m
                                                      for item in residuals),
                                        n_walls=len(supporting))
        dx += step[0]
        dy += step[1]
        dyaw = norm_angle(dyaw + step[2] / angular_scale)
        if max(abs(step[0]), abs(step[1]),
               abs(step[2] / angular_scale)) < 1e-7:
            break

    c, s = math.cos(dyaw), math.sin(dyaw)
    final_residuals = []
    final_wall_counts = {}
    for p, wall, _, _ in pairs:
        rx, ry = p[0] - current_pose.x, p[1] - current_pose.y
        q = (current_pose.x + dx + c * rx - s * ry,
             current_pose.y + dy + s * rx + c * ry)
        r, along, length, _, _ = _signed_distance_and_tangent(q, wall)
        if (-cfg.association_gate_m <= along <=
                length + cfg.association_gate_m and
                abs(r) <= cfg.inlier_gate_m):
            final_residuals.append(abs(r))
            final_wall_counts[wall] = final_wall_counts.get(wall, 0) + 1
    final_support = [wall for wall, count in final_wall_counts.items()
                     if count >= cfg.min_hits_per_wall]
    frac = len(final_residuals) / max(len(pairs), 1)
    med, p90 = _quantile(final_residuals, .5), _quantile(final_residuals, .9)
    base = dict(n_input_hits=len(points), n_associated=len(pairs),
                n_inliers=len(final_residuals), n_walls=len(final_support),
                median_residual_m=med, p90_residual_m=p90)
    if len(final_residuals) < cfg.min_hits or frac < cfg.min_inlier_fraction:
        return PoseCorrectionResult(False, 'POOR_FIT', **base)
    if med > cfg.max_median_residual_m or p90 > cfg.max_p90_residual_m:
        return PoseCorrectionResult(False, 'RESIDUAL_TOO_LARGE', **base)
    if not _diverse_normals(final_support, cfg.min_wall_angle_deg):
        return PoseCorrectionResult(False, 'INSUFFICIENT_FINAL_WALL_SUPPORT',
                                    **base)
    if any(final_wall_counts.get(wall, 0) / pair_counts[wall] <
           cfg.min_wall_inlier_fraction for wall in supporting):
        return PoseCorrectionResult(False, 'POOR_WALL_SUPPORT', **base)
    if math.hypot(dx, dy) > cfg.max_translation_m or abs(dyaw) > cfg.max_yaw_rad:
        return PoseCorrectionResult(False, 'CORRECTION_EXCEEDS_BOUND', **base)
    corrected = Pose2D(current_pose.x + dx, current_pose.y + dy,
                       norm_angle(current_pose.yaw + dyaw))
    return PoseCorrectionResult(True, 'ACCEPTED', corrected, dx, dy, dyaw,
                                **base)
