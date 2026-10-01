"""Arc-length trajectory references for the fixed motion primitives."""

from dataclasses import dataclass
import math

from .motion_primitive import MotionPrimitive


@dataclass(frozen=True)
class ReferenceState:
    """A trajectory sample in world coordinates.

    ``yaw_ref`` is the held chassis heading. It is deliberately independent of
    the translational tangent, including while following an arc.
    """

    x: float
    y: float
    yaw_ref: float
    vx_world: float
    vy_world: float
    tangent_x: float
    tangent_y: float
    curvature: float
    progress_s: float


class TrajectoryReference:
    """Sample a chain of STRAIGHT/REVERSE/ARC primitives by distance."""

    def __init__(self, prims, yaw_ref: float):
        self.prims = tuple(p for p in prims if p.kind != 'STOP' and p.length > 0.0)
        self.yaw_ref = float(yaw_ref)
        if not math.isfinite(self.yaw_ref):
            raise ValueError('yaw_ref must be finite')
        self._ends = []
        total = 0.0
        for prim in self.prims:
            if prim.kind not in ('STRAIGHT', 'REVERSE', 'ARC'):
                raise ValueError(f'unsupported trajectory primitive: {prim.kind}')
            total += prim.length
            self._ends.append(total)
        self.length = total

    def sample(self, progress_s: float, speed: float = 0.0,
               acceleration: float = 0.0) -> ReferenceState:
        """Return pose, travel tangent, curvature, and world velocity feedforward.

        ``acceleration`` is accepted for speed-planner integration but does not
        alter the geometric reference or feedforward velocity in this version.
        """
        del acceleration
        if not all(math.isfinite(v) for v in (progress_s, speed)):
            raise ValueError('progress_s and speed must be finite')
        if speed < 0.0:
            raise ValueError('speed must be nonnegative; reverse is encoded by geometry')
        if not self.prims:
            raise ValueError('cannot sample an empty trajectory')

        s = min(self.length, max(0.0, progress_s))
        idx = len(self._ends) - 1
        for i, end in enumerate(self._ends):
            if s <= end:
                idx = i
                break
        before = self._ends[idx - 1] if idx else 0.0
        prim = self.prims[idx]
        local_s = min(prim.length, max(0.0, s - before))

        if prim.kind in ('STRAIGHT', 'REVERSE'):
            dx, dy = prim.p1[0] - prim.p0[0], prim.p1[1] - prim.p0[1]
            norm = math.hypot(dx, dy)
            tx, ty = dx / norm, dy / norm
            x = prim.p0[0] + tx * local_s
            y = prim.p0[1] + ty * local_s
            curvature = 0.0
        else:
            radius = prim.meta['r']
            angle = prim.yaw0 + math.copysign(local_s / radius, prim.yaw1)
            tx = -math.sin(angle) * math.copysign(1.0, prim.yaw1)
            ty = math.cos(angle) * math.copysign(1.0, prim.yaw1)
            x = prim.p0[0] + radius * math.cos(angle)
            y = prim.p0[1] + radius * math.sin(angle)
            curvature = math.copysign(1.0 / radius, prim.yaw1)

        return ReferenceState(
            x=x, y=y, yaw_ref=self.yaw_ref,
            vx_world=tx * speed, vy_world=ty * speed,
            tangent_x=tx, tangent_y=ty, curvature=curvature,
            progress_s=s,
        )

    def project(self, x: float, y: float, *, minimum_progress: float = 0.0,
                maximum_progress: float = None, direction=None,
                preferred_progress: float = None):
        """Project a point onto the remaining forward part of this route.

        Returns ``(progress_s, distance)``. Restricting the search to progress
        at or after ``minimum_progress`` prevents a return leg near the start
        from snapping back to an already traversed part of a loop.
        """
        values = [x, y, minimum_progress]
        if maximum_progress is not None:
            values.append(maximum_progress)
        if preferred_progress is not None:
            values.append(preferred_progress)
        if direction is not None:
            values.extend(direction)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('projection inputs must be finite')
        minimum_progress = min(self.length, max(0.0, minimum_progress))
        maximum_progress = (self.length if maximum_progress is None else
                            min(self.length, max(minimum_progress,
                                                 maximum_progress)))
        direction_norm = (math.hypot(*direction) if direction is not None
                          else 0.0)
        best = None
        before = 0.0
        for prim, end in zip(self.prims, self._ends):
            start = before
            before = end
            if end < minimum_progress or start > maximum_progress:
                continue
            local_min = max(0.0, minimum_progress - start)
            local_max = min(prim.length, maximum_progress - start)
            if local_max < local_min:
                continue
            if prim.kind in ('STRAIGHT', 'REVERSE'):
                dx, dy = prim.p1[0] - prim.p0[0], prim.p1[1] - prim.p0[1]
                length = math.hypot(dx, dy)
                along = ((x - prim.p0[0]) * dx + (y - prim.p0[1]) * dy) / (length * length)
                local = min(local_max, max(local_min, along * prim.length))
                px = prim.p0[0] + dx * local / length
                py = prim.p0[1] + dy * local / length
                tx, ty = dx / length, dy / length
            else:
                radius = prim.meta['r']
                angle = math.atan2(y - prim.p0[1], x - prim.p0[0])
                sign = math.copysign(1.0, prim.yaw1)
                raw = angle - prim.yaw0
                delta = math.atan2(math.sin(raw), math.cos(raw)) * sign
                local = min(local_max, max(local_min,
                            min(prim.length, delta * radius)))
                projected_angle = prim.yaw0 + sign * local / radius
                px = prim.p0[0] + radius * math.cos(projected_angle)
                py = prim.p0[1] + radius * math.sin(projected_angle)
                tx = -math.sin(projected_angle) * sign
                ty = math.cos(projected_angle) * sign
            alignment = ((direction[0] * tx + direction[1] * ty) /
                         direction_norm if direction_norm > 1e-6 else 0.0)
            progress = start + local
            proximity = (abs(progress - preferred_progress)
                         if preferred_progress is not None else 0.0)
            candidate = (progress, math.hypot(x - px, y - py), alignment,
                         proximity)
            if (best is None or candidate[1] < best[1] - 0.005 or
                    (abs(candidate[1] - best[1]) <= 0.005 and
                     (candidate[2] > best[2] + 1e-6 or
                      (abs(candidate[2] - best[2]) <= 1e-6 and
                       candidate[3] < best[3])))):
                best = candidate
        if best is None:
            return self.length, math.hypot(x - self.sample(self.length).x,
                                          y - self.sample(self.length).y)
        return best[0], best[1]
