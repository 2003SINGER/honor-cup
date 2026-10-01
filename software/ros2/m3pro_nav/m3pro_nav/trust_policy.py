#!/usr/bin/env python3
"""TrustPolicy —— raw grid candidates → TrustedEdgeObservation.

默认 diagnostic_only = true: 一条观测都不允许 commit 进 EdgeMap。
未实测标定前, 所有信任阈值必须保持 UNCALIBRATED —— 本模块不内置任何
"经验值", 参数接口留着, 值等现场数据说了算。

Runtime chain:
  DiagnosticObservation → TrustPolicy → TrustedEdgeObservation
      → ObservationAdapter → EdgeMap → CellAction → ActionHorizon
      → MotionRuntimeCore.append_suffix() (safe-extension 不停车续跑)"""

from dataclasses import dataclass
import math
import numbers


CALIBRATED_KEYS = ('max_residual_m', 'max_incidence_rad', 'max_range_m',
                   'min_uniqueness_margin_m', 'min_corner_distance_m',
                   'min_votes')


@dataclass(frozen=True)
class TrustThresholds:
    """信任阈值 —— 全部 UNCALIBRATED (None) 直到现场标定。

    任何字段为 None 时 policy 一律 ABSTAIN: 宁可不产生信息,
    也不凭猜测把错误地图写死。"""
    max_residual_m: float | None = None
    max_incidence_rad: float | None = None
    max_range_m: float | None = None
    min_uniqueness_margin_m: float | None = None
    min_corner_distance_m: float | None = None
    min_votes: int | None = None

    def __post_init__(self):
        for key in CALIBRATED_KEYS:
            if key == 'min_votes':
                continue
            value = getattr(self, key)
            if value is not None and (
                    isinstance(value, bool) or not isinstance(value, numbers.Real)
                    or not math.isfinite(float(value)) or value < 0):
                raise ValueError(f'{key} must be finite and nonnegative or None')
        if self.min_votes is not None and (
                isinstance(self.min_votes, bool)
                or not isinstance(self.min_votes, int)
                or self.min_votes < 1):
            raise ValueError('min_votes must be a positive integer or None')

    @property
    def calibrated(self):
        return all(getattr(self, k) is not None for k in CALIBRATED_KEYS)

    def uncalibrated_fields(self):
        return tuple(k for k in CALIBRATED_KEYS
                     if getattr(self, k) is None)


@dataclass(frozen=True)
class TrustedEdgeObservation:
    """Calibrated evidence that may be passed to EdgeMap."""
    edge_id: tuple
    state: str                    # 'WALL' / 'OPEN'
    residual: float
    range: float
    incidence_angle: float
    stamp: float


@dataclass(frozen=True)
class TrustDecision:
    """TrustPolicy 的输出: 诊断层永远得到解释, 不允许裸 trusted=False."""
    accepted: bool
    reason: str                   # DIAGNOSTIC_ONLY / UNCALIBRATED / ...
    observation: TrustedEdgeObservation | None


class TrustPolicy:
    def __init__(self, *, diagnostic_only=True, thresholds=None):
        self.diagnostic_only = diagnostic_only
        self.thresholds = thresholds or TrustThresholds()
        self._votes = {}                 # (edge, state) -> (count, last stamp)

    def _vote(self, edge, state, stamp):
        """Count at most one accepted measurement per edge/state/scan."""
        key = (tuple(edge), state)
        count, last_stamp = self._votes.get(key, (0, None))
        if last_stamp is not None and stamp <= last_stamp:
            return False, True
        count = min(self.thresholds.min_votes, count + 1)
        self._votes[key] = (count, stamp)
        return count >= self.thresholds.min_votes, False

    @staticmethod
    def _validate_stamp(stamp):
        if (isinstance(stamp, bool) or not isinstance(stamp, numbers.Real)
                or not math.isfinite(float(stamp))):
            raise ValueError('stamp must be a finite numeric scan timestamp')

    def evaluate(self, observation, stamp=0.0):
        """DiagnosticObservation → TrustDecision.

        diagnostic_only 或阈值未标定 → 一律拒绝并给出明确原因;
        阈值齐全且 diagnostic_only=False 才可能产生 TrustedEdgeObservation
        供正式感知前端使用。"""
        self._validate_stamp(stamp)
        if self.diagnostic_only:
            return TrustDecision(False, 'DIAGNOSTIC_ONLY', None)
        if not self.thresholds.calibrated:
            missing = self.thresholds.uncalibrated_fields()
            return TrustDecision(
                False, 'UNCALIBRATED:' + ','.join(missing), None)
        # Diagnostic thresholds only label observations for analysis. Their
        # UNIQUE/NONE/AMBIGUOUS outcome must never become a hidden production
        # trust gate. Re-evaluate the raw candidates using calibrated limits.
        candidates = getattr(observation, 'candidates', ())
        if not candidates:
            return TrustDecision(False, 'NO_CANDIDATE', None)
        candidates = sorted(candidates, key=lambda c: c.residual)
        cand = candidates[0]
        th = self.thresholds
        if cand.residual > th.max_residual_m:
            return TrustDecision(False, 'LARGE_RESIDUAL', None)
        if cand.incidence_angle > th.max_incidence_rad:
            return TrustDecision(False, 'GRAZING_INCIDENCE', None)
        if cand.range > th.max_range_m:
            return TrustDecision(False, 'RANGE_EXCEEDED', None)
        if (len(candidates) > 1
                and candidates[1].residual - cand.residual
                < th.min_uniqueness_margin_m):
            return TrustDecision(False, 'WEAK_UNIQUENESS', None)
        if cand.distance_to_corner < th.min_corner_distance_m:
            return TrustDecision(False, 'NEAR_CORNER', None)
        enough_votes, duplicate_stamp = self._vote(cand.edge_id, 'WALL', stamp)
        if duplicate_stamp:
            return TrustDecision(False, 'DUPLICATE_STAMP', None)
        if not enough_votes:
            return TrustDecision(False, 'MIN_VOTES', None)
        return TrustDecision(True, 'OK', TrustedEdgeObservation(
            cand.edge_id, 'WALL', cand.residual, cand.range,
            cand.incidence_angle, stamp))

    def plausible_wall(self, observation):
        """Whether raw hit geometry can veto OPEN in this scan.

        Vote count is intentionally excluded: a wall visible in one scan must
        not be contradicted by free-path rays from that same scan.
        """
        if self.diagnostic_only or not self.thresholds.calibrated:
            return False
        candidates = sorted(getattr(observation, 'candidates', ()),
                            key=lambda c: c.residual)
        if not candidates:
            return False
        cand, th = candidates[0], self.thresholds
        return (cand.residual <= th.max_residual_m
                and cand.incidence_angle <= th.max_incidence_rad
                and cand.range <= th.max_range_m
                and cand.distance_to_corner >= th.min_corner_distance_m
                and (len(candidates) == 1 or
                     candidates[1].residual - cand.residual >=
                     th.min_uniqueness_margin_m))

    def evaluate_open_edge(self, edge, *, stamp, range_m,
                           incidence_angle, corner_distance_m):
        """Evaluate one free-path edge from one ray.

        Production callers pass the frame's shared scan stamp for every ray;
        only the first qualifying ray for an edge in that scan adds a vote.
        """
        self._validate_stamp(stamp)
        if self.diagnostic_only:
            return TrustDecision(False, 'DIAGNOSTIC_ONLY', None)
        if not self.thresholds.calibrated:
            missing = self.thresholds.uncalibrated_fields()
            return TrustDecision(False, 'UNCALIBRATED:' + ','.join(missing),
                                 None)
        if not isinstance(edge, tuple) or not edge:
            raise ValueError('edge must be a non-empty canonical edge tuple')
        values = (range_m, incidence_angle, corner_distance_m)
        if any(isinstance(v, bool) or not isinstance(v, numbers.Real)
               or not math.isfinite(float(v)) for v in values):
            raise ValueError('OPEN evidence features must be finite numbers')
        th = self.thresholds
        if range_m > th.max_range_m:
            return TrustDecision(False, 'RANGE_EXCEEDED', None)
        if incidence_angle > th.max_incidence_rad:
            return TrustDecision(False, 'GRAZING_INCIDENCE', None)
        if corner_distance_m < th.min_corner_distance_m:
            return TrustDecision(False, 'NEAR_CORNER', None)
        enough_votes, duplicate_stamp = self._vote(edge, 'OPEN', stamp)
        if duplicate_stamp:
            return TrustDecision(False, 'DUPLICATE_STAMP', None)
        if not enough_votes:
            return TrustDecision(False, 'MIN_VOTES', None)
        return TrustDecision(True, 'OK_OPEN', TrustedEdgeObservation(
            edge, 'OPEN', 0.0, float(range_m), float(incidence_angle), stamp))
