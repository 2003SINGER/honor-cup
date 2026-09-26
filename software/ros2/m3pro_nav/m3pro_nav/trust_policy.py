#!/usr/bin/env python3
"""TrustPolicy —— DiagnosticObservation → (未来) TrustedEdgeObservation.

默认 diagnostic_only = true: 一条观测都不允许 commit 进 EdgeMap。
未实测标定前, 所有信任阈值必须保持 UNCALIBRATED —— 本模块不内置任何
"经验值", 参数接口留着, 值等现场数据说了算。

未来链路 (本轮只到接口, 不驱动导航):
  DiagnosticObservation → TrustPolicy → TrustedEdgeObservation
      → ObservationAdapter → EdgeMap → CellAction → ActionHorizon
      → MotionRuntimeCore.append_suffix() (safe-extension 不停车续跑)"""

from dataclasses import dataclass


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

    @property
    def calibrated(self):
        return all(getattr(self, k) is not None for k in CALIBRATED_KEYS)

    def uncalibrated_fields(self):
        return tuple(k for k in CALIBRATED_KEYS
                     if getattr(self, k) is None)


@dataclass(frozen=True)
class TrustedEdgeObservation:
    """未来喂给 EdgeMap 的可信观测 (本轮只定义形状, 不产生实例)."""
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

    def evaluate(self, observation, stamp=0.0):
        """DiagnosticObservation → TrustDecision.

        diagnostic_only 或阈值未标定 → 一律拒绝并给出明确原因;
        阈值齐全且 diagnostic_only=False 才可能产生 TrustedEdgeObservation
        (供未来标定后的正式感知前端使用)。"""
        if self.diagnostic_only:
            return TrustDecision(False, 'DIAGNOSTIC_ONLY', None)
        if not self.thresholds.calibrated:
            missing = self.thresholds.uncalibrated_fields()
            return TrustDecision(
                False, 'UNCALIBRATED:' + ','.join(missing), None)
        if observation.outcome == 'OPEN':
            if not observation.open_edges:
                return TrustDecision(False, 'NO_OPEN_EDGE', None)
            edge = observation.open_edges[0]
            return TrustedDecision_accept_open(edge, observation, stamp)
        if observation.outcome != 'UNIQUE' or observation.candidate is None:
            return TrustDecision(False, 'NOT_UNIQUE', None)
        cand = observation.candidate
        th = self.thresholds
        if cand.residual > th.max_residual_m:
            return TrustDecision(False, 'LARGE_RESIDUAL', None)
        if cand.incidence_angle > th.max_incidence_rad:
            return TrustDecision(False, 'GRAZING_INCIDENCE', None)
        if cand.range > th.max_range_m:
            return TrustDecision(False, 'RANGE_EXCEEDED', None)
        if (len(observation.candidates) > 1
                and observation.candidates[1].residual - cand.residual
                < th.min_uniqueness_margin_m):
            return TrustDecision(False, 'WEAK_UNIQUENESS', None)
        if cand.distance_to_corner < th.min_corner_distance_m:
            return TrustDecision(False, 'NEAR_CORNER', None)
        return TrustDecision(True, 'OK', TrustedEdgeObservation(
            cand.edge_id, 'WALL', cand.residual, cand.range,
            cand.incidence_angle, stamp))


def TrustedDecision_accept_open(edge, observation, stamp):
    # OPEN 证据的接受路径 (future): 单条 free 穿越只算一票,
    # EdgeMap 的真迟滞 (T_CONFIRM/T_FLIP) 由上层投票机制承担。
    return TrustDecision(True, 'OK_OPEN', TrustedEdgeObservation(
        edge, 'OPEN', 0.0, 0.0, 0.0, stamp))
