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
