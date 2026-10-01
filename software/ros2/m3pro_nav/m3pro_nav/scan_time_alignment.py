"""Source-time odometry history and conservative LaserScan alignment.

This module deliberately does not deskew individual beams. It interpolates a
single pose at the scan midpoint and abstains when the scan duration implies
more motion than the configured stationary-scan tolerance.
"""

from collections import deque
from dataclasses import dataclass
import math

from .pose import Pose2D, norm_angle


@dataclass(frozen=True)
class OdomPoseSample:
    stamp: float
    pose: Pose2D
    received_stamp: float | None = None


@dataclass(frozen=True)
class ScanAlignment:
    accepted: bool
    reason: str | None
    pose: Pose2D | None
    start_pose: Pose2D | None
    end_pose: Pose2D | None
    scan_start: float
    scan_end: float
    max_nearest_odom_skew: float | None
    mode: str = 'ABSTAIN'
    motion_window_start: float | None = None
    motion_window_end: float | None = None


class OdomPoseHistory:
    """Bounded source-stamp pose history; source stamps must strictly advance."""

    def __init__(self, *, history_s: float = 1.0, max_samples: int = 256):
        if not math.isfinite(history_s) or history_s <= 0:
            raise ValueError('history_s must be positive and finite')
        if not isinstance(max_samples, int) or max_samples < 2:
            raise ValueError('max_samples must be an integer >= 2')
        self.history_s = float(history_s)
        self.max_samples = max_samples
        self._samples = deque()

    def __len__(self):
        return len(self._samples)

    @property
    def samples(self):
        return tuple(self._samples)

    def add(self, stamp: float, pose: Pose2D,
            received_stamp: float | None = None) -> OdomPoseSample:
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError('odometry source stamp must be finite')
        if not all(math.isfinite(float(v)) for v in (pose.x, pose.y, pose.yaw)):
            raise ValueError('odometry pose must be finite')
        if received_stamp is not None and not math.isfinite(float(received_stamp)):
            raise ValueError('odometry receive stamp must be finite')
        if self._samples and stamp <= self._samples[-1].stamp:
            raise ValueError('odometry source stamps must strictly advance')
        sample = OdomPoseSample(stamp, pose.copy(),
                                None if received_stamp is None
                                else float(received_stamp))
        self._samples.append(sample)
        cutoff = stamp - self.history_s
        while len(self._samples) > 2 and self._samples[1].stamp < cutoff:
            self._samples.popleft()
        while len(self._samples) > self.max_samples:
            self._samples.popleft()
        return sample

    def interpolate(self, stamp: float, *, max_bracket_gap_s: float = 0.15,
                    max_extrapolation_s: float = 0.03):
        """Return ``(pose, nearest_sample_skew)`` or ``None`` if unbracketed.

        The bracket-gap limit prevents interpolation across missing odom data;
        extrapolation at either end is separately bounded. Yaw follows the
        shortest wrapped arc.
        """
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError('query stamp must be finite')
        if not math.isfinite(max_bracket_gap_s) or max_bracket_gap_s <= 0:
            raise ValueError('max_bracket_gap_s must be positive and finite')
        if not math.isfinite(max_extrapolation_s) or max_extrapolation_s < 0:
            raise ValueError('max_extrapolation_s must be finite and nonnegative')
        if not self._samples:
            return None
        samples = tuple(self._samples)
        if len(samples) == 1:
            if stamp == samples[0].stamp:
                return samples[0].pose.copy(), 0.0
            return None
        if stamp < samples[0].stamp:
            left, right = samples[0], samples[1]
            if left.stamp - stamp > max_extrapolation_s:
                return None
        elif stamp > samples[-1].stamp:
            left, right = samples[-2], samples[-1]
            if stamp - right.stamp > max_extrapolation_s:
                return None
        else:
            left = samples[0]
            right = samples[1]
            for candidate in samples[1:]:
                if stamp <= candidate.stamp:
                    right = candidate
                    break
                left = candidate
                right = candidate
            if stamp == left.stamp:
                return left.pose.copy(), 0.0
            if stamp == right.stamp:
                return right.pose.copy(), 0.0
        span = right.stamp - left.stamp
        if span <= 0 or span > max_bracket_gap_s:
            return None
        alpha = (stamp - left.stamp) / span
        nearest_skew = min(abs(stamp - left.stamp), abs(right.stamp - stamp))
        yaw_delta = norm_angle(right.pose.yaw - left.pose.yaw)
        return Pose2D(
            left.pose.x + alpha * (right.pose.x - left.pose.x),
            left.pose.y + alpha * (right.pose.y - left.pose.y),
            norm_angle(left.pose.yaw + alpha * yaw_delta),
        ), nearest_skew


class ScanTimeAligner:
    """Align a scan to source time and permit only near-stationary whole scans."""

    def __init__(self, history: OdomPoseHistory, *, max_bracket_gap_s: float = 0.15,
                 max_extrapolation_s: float = 0.03,
                 max_trust_motion_mps: float = 0.03,
                 max_trust_yaw_rate_radps: float = 0.05,
                 static_window_fallback_s: float = 0.25):
        if not isinstance(history, OdomPoseHistory):
            raise TypeError('history must be OdomPoseHistory')
        for name, value in (('max_bracket_gap_s', max_bracket_gap_s),
                            ('max_extrapolation_s', max_extrapolation_s),
                            ('max_trust_motion_mps', max_trust_motion_mps),
                            ('max_trust_yaw_rate_radps', max_trust_yaw_rate_radps),
                            ('static_window_fallback_s', static_window_fallback_s)):
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if max_bracket_gap_s == 0:
            raise ValueError('max_bracket_gap_s must be positive')
        if static_window_fallback_s == 0:
            raise ValueError('static_window_fallback_s must be positive')
        self.history = history
        self.max_bracket_gap_s = float(max_bracket_gap_s)
        self.max_extrapolation_s = float(max_extrapolation_s)
        self.max_trust_motion_mps = float(max_trust_motion_mps)
        self.max_trust_yaw_rate_radps = float(max_trust_yaw_rate_radps)
        self.static_window_fallback_s = float(static_window_fallback_s)

    def _motion_rates(self, start: float, end: float, p0: Pose2D,
                      p1: Pose2D) -> tuple[float, float]:
        """Maximum observed translation/yaw rate across an interval."""
        if end <= start:
            return 0.0, 0.0
        knots = [(start, p0)]
        knots.extend((s.stamp, s.pose) for s in self.history.samples
                     if start < s.stamp < end)
        knots.append((end, p1))
        translation_rate = yaw_rate = 0.0
        for (ta, pa), (tb, pb) in zip(knots, knots[1:]):
            dt = tb - ta
            if dt <= 0:
                continue
            translation_rate = max(
                translation_rate,
                math.hypot(pb.x - pa.x, pb.y - pa.y) / dt)
            yaw_rate = max(yaw_rate,
                           abs(norm_angle(pb.yaw - pa.yaw)) / dt)
        return translation_rate, yaw_rate

    def _align_static_window(self, stamp: float) -> ScanAlignment:
        """Use a bounded, source-stamped stationary window when scan timing is absent."""
        start = stamp - self.static_window_fallback_s
        end = stamp + self.static_window_fallback_s
        kwargs = dict(max_bracket_gap_s=self.max_bracket_gap_s,
                      max_extrapolation_s=0.0)
        left = self.history.interpolate(start, **kwargs)
        at_stamp = self.history.interpolate(stamp, **kwargs)
        right = self.history.interpolate(end, **kwargs)
        if left is None or at_stamp is None or right is None:
            return ScanAlignment(
                False, 'STATIC_WINDOW_NOT_COVERED', None,
                None if left is None else left[0],
                None if right is None else right[0], stamp, stamp, None,
                'STATIC_WINDOW_FALLBACK', start, end)
        p0, skew0 = left
        pose, skew_stamp = at_stamp
        p1, skew1 = right
        translation_rate, yaw_rate = self._motion_rates(start, end, p0, p1)
        skew = max(skew0, skew_stamp, skew1)
        if (translation_rate > self.max_trust_motion_mps or
                yaw_rate > self.max_trust_yaw_rate_radps):
            return ScanAlignment(
                False, 'STATIC_WINDOW_MOTION', None, p0, p1, stamp, stamp,
                skew, 'STATIC_WINDOW_FALLBACK', start, end)
        return ScanAlignment(
            True, None, pose, p0, p1, stamp, stamp, skew,
            'STATIC_WINDOW_FALLBACK', start, end)

    def align(self, frame) -> ScanAlignment:
        """Return a midpoint pose or ABSTAIN with a stable reason string.

        LaserScan ``header.stamp`` is treated as the first beam time. Since this
        implementation does not deskew, scans with measurable endpoint motion
        are rejected rather than projected from a misleading single pose.
        """
        start = float(frame.stamp)
        increment = float(frame.time_increment)
        count = len(frame.rays)
        if not math.isfinite(start) or not math.isfinite(increment) or increment < 0:
            return ScanAlignment(False, 'INVALID_SCAN_TIME', None, None, None,
                                 start, start, None)
        if len(frame.rays) > 1 and increment == 0:
            return self._align_static_window(start)
        end = start + max(0, count - 1) * increment
        if not math.isfinite(end):
            return ScanAlignment(False, 'INVALID_SCAN_TIME', None, None, None,
                                 start, start, None)
        start_sample = self.history.interpolate(
            start, max_bracket_gap_s=self.max_bracket_gap_s,
            max_extrapolation_s=self.max_extrapolation_s)
        end_sample = self.history.interpolate(
            end, max_bracket_gap_s=self.max_bracket_gap_s,
            max_extrapolation_s=self.max_extrapolation_s)
        mid = (start + end) / 2.0
        mid_sample = self.history.interpolate(
            mid, max_bracket_gap_s=self.max_bracket_gap_s,
            max_extrapolation_s=self.max_extrapolation_s)
        if start_sample is None or end_sample is None or mid_sample is None:
            return ScanAlignment(False, 'ODOM_TIME_UNBRACKETED_OR_SKEWED',
                                 None, None, None, start, end, None)
        p0, s0 = start_sample
        p1, s1 = end_sample
        pm, sm = mid_sample
        skew = max(s0, s1, sm)
        translation_rate, yaw_rate = self._motion_rates(start, end, p0, p1)
        if (translation_rate > self.max_trust_motion_mps or
                yaw_rate > self.max_trust_yaw_rate_radps):
            return ScanAlignment(False, 'MOTION_DISTORTION_REQUIRES_DESKEW',
                                 None, p0, p1, start, end, skew,
                                 'SCAN_INTERVAL')
        return ScanAlignment(True, None, pm, p0, p1, start, end, skew,
                             'SCAN_INTERVAL')
