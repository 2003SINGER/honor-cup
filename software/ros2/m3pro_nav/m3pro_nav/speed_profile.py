"""Time parameterisation for the existing fixed maze motion primitives.

The geometry compiler owns the path.  This module only chooses a feasible
speed along it, including a full stop at an unknown decision boundary or a
reversal.  No plant or controller parameters are hidden in the profile.
"""

from dataclasses import dataclass
import math


_EPS = 1e-9


def _tangent(primitive, *, at_end=False):
    if primitive.kind in ('STRAIGHT', 'REVERSE'):
        dx = primitive.p1[0] - primitive.p0[0]
        dy = primitive.p1[1] - primitive.p0[1]
        length = math.hypot(dx, dy)
        return dx / length, dy / length
    if primitive.kind == 'ARC':
        angle = primitive.yaw0 + (primitive.yaw1 if at_end else 0.0)
        sign = math.copysign(1.0, primitive.yaw1)
        return -sign * math.sin(angle), sign * math.cos(angle)
    return None


@dataclass(frozen=True)
class SpeedSample:
    time: float
    progress_s: float
    speed: float
    acceleration: float
    primitive_index: int


@dataclass(frozen=True)
class _Phase:
    t0: float
    s0: float
    v0: float
    acceleration: float
    duration: float
    primitive_index: int

    @property
    def end_time(self):
        return self.t0 + self.duration


class SpeedProfile:
    """Immutable piecewise constant-acceleration profile over one chain.

    ``v_end`` is a seam speed ceiling, not a demand to accelerate to that speed
    over a short cell segment.  A backward braking pass and a forward
    acceleration pass lower seam speeds as needed.  Initial speed is fixed;
    an impossible stop from that speed is rejected.
    """

    def __init__(self, primitives, *, start_speed=0.0, a_acc=1.0, a_dec=1.0):
        if not primitives:
            raise ValueError("speed profile requires at least one primitive")
        if not all(math.isfinite(x) for x in (start_speed, a_acc, a_dec)):
            raise ValueError("speed parameters must be finite")
        if start_speed < 0 or a_acc <= 0 or a_dec <= 0:
            raise ValueError("speed must be nonnegative and acceleration limits positive")
        self._phases = []
        ceilings = [math.inf] * (len(primitives) + 1)
        ceilings[0] = start_speed
        for index, primitive in enumerate(primitives):
            if primitive.kind == 'STOP':
                if not math.isfinite(primitive.duration) or primitive.duration < 0:
                    raise ValueError("STOP duration must be finite and nonnegative")
                ceilings[index] = ceilings[index + 1] = 0.0
                continue
            if primitive.kind not in ('STRAIGHT', 'ARC', 'REVERSE'):
                raise ValueError(f"unsupported primitive kind: {primitive.kind}")
            length, vmax, vend = primitive.length, primitive.v_max, primitive.v_end
            if not all(math.isfinite(x) for x in (length, vmax, vend)):
                raise ValueError(f"primitive {index} has nonfinite speed data")
            if length <= 0 or vmax <= 0 or vend < 0 or vend > vmax + _EPS:
                raise ValueError(f"primitive {index} has invalid speed bounds")
            ceilings[index] = min(ceilings[index], vmax)
            ceilings[index + 1] = min(ceilings[index + 1], vend)
            if index + 1 < len(primitives) and primitives[index + 1].kind == 'STOP' and vend > _EPS:
                raise ValueError(f"primitive {index} must end at zero before STOP")
            if index + 1 < len(primitives) and primitives[index + 1].kind != 'STOP':
                now = _tangent(primitive, at_end=True)
                following = _tangent(primitives[index + 1])
                if (now[0] * following[0] + now[1] * following[1] < -1 + _EPS
                        and vend > _EPS):
                    raise ValueError(f"primitive {index} must end at zero before reversal")
        if ceilings[0] + _EPS < start_speed:
            raise ValueError("initial speed exceeds first primitive speed limit")
        for index in range(len(primitives) - 1, -1, -1):
            primitive = primitives[index]
            if primitive.kind != 'STOP':
                allowed = math.sqrt(ceilings[index + 1] ** 2 + 2 * a_dec * primitive.length)
                if index == 0 and start_speed > allowed + _EPS:
                    raise ValueError(f"primitive {index} cannot brake from initial speed")
                ceilings[index] = min(ceilings[index], allowed)
        ceilings[0] = start_speed
        for index, primitive in enumerate(primitives):
            if primitive.kind != 'STOP':
                allowed = math.sqrt(ceilings[index] ** 2 + 2 * a_acc * primitive.length)
                ceilings[index + 1] = min(ceilings[index + 1], allowed)
        self.boundary_speeds = tuple(ceilings)
        t = s = completed_distance = 0.0
        v = start_speed

        def append(index, acceleration, duration):
            nonlocal t, s, v
            if duration <= _EPS:
                return
            self._phases.append(_Phase(t, s, v, acceleration, duration, index))
            s += v * duration + 0.5 * acceleration * duration * duration
            v = max(0.0, v + acceleration * duration)
            t += duration

        for index, primitive in enumerate(primitives):
            kind = primitive.kind
            if kind == 'STOP':
                if v > _EPS:
                    raise ValueError(f"STOP {index} reached at {v:.6f} m/s")
                v = 0.0
                append(index, 0.0, primitive.duration)
                continue
            length, vmax, vend = primitive.length, primitive.v_max, ceilings[index + 1]

            peak_sq = (2 * a_acc * a_dec * length + a_dec * v * v +
                       a_acc * vend * vend) / (a_acc + a_dec)
            peak = min(vmax, math.sqrt(max(0.0, peak_sq)))
            if peak + _EPS < max(v, vend):
                raise ValueError(f"primitive {index} has infeasible peak speed")
            t_acc = max(0.0, (peak - v) / a_acc)
            t_dec = max(0.0, (peak - vend) / a_dec)
            d_acc = (v + peak) * t_acc / 2
            d_dec = (vend + peak) * t_dec / 2
            d_cruise = length - d_acc - d_dec
            if d_cruise < -1e-7:
                raise ValueError(f"primitive {index} exceeds its available distance")
            append(index, a_acc, t_acc)
            if d_cruise > _EPS:
                append(index, 0.0, d_cruise / peak)
            append(index, -a_dec, t_dec)
            # Pin floating-point phase sums to the geometric seam.
            completed_distance += length
            s = completed_distance
            v = vend

        self.duration = t
        self.length = s
        self.end_speed = v
        self._last_index = len(primitives) - 1

    def sample(self, time):
        if not math.isfinite(time):
            raise ValueError("sample time must be finite")
        if time <= 0:
            phase = self._phases[0] if self._phases else None
            return SpeedSample(0.0, 0.0, phase.v0 if phase else self.end_speed,
                               phase.acceleration if phase else 0.0,
                               phase.primitive_index if phase else 0)
        if time >= self.duration:
            return SpeedSample(self.duration, self.length, self.end_speed, 0.0,
                               self._last_index)
        for phase in self._phases:
            if time < phase.end_time:
                dt = time - phase.t0
                return SpeedSample(time,
                    phase.s0 + phase.v0 * dt + 0.5 * phase.acceleration * dt * dt,
                    max(0.0, phase.v0 + phase.acceleration * dt),
                    phase.acceleration, phase.primitive_index)
        raise RuntimeError("time is inside profile but outside its phases")


def plan_speed(primitives, *, start_speed=0.0, a_acc=1.0, a_dec=1.0):
    return SpeedProfile(primitives, start_speed=start_speed, a_acc=a_acc, a_dec=a_dec)
