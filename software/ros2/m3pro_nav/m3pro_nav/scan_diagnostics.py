#!/usr/bin/env python3
"""ScanDiagnostics —— 感知诊断统计的在线/离线同一聚合链 (纯 Python).

铁律: 在线节点与离线 bag 重放必须走同一个 collector + summary ——
禁止 online_analysis.py / offline_analysis_v2.py 两套代码。
同一 bag 重放两次, 与时间无关的统计必须逐字节一致。

分位数用 1mm 直方图桶精确可加: 每帧记录桶计数, 聚合即为全量,
不需要保留原始样本 (静态 session 几百万 beam 也不膨胀)。
本模块绝不自动宣布"可信距离是多少" —— 只收曲线, 结论人来定。"""

from dataclasses import dataclass, field as dc_field
import math

from .grid_association import UNIQUE, AMBIGUOUS, NONE

RANGE_BINS = ((0.0, 0.4), (0.4, 0.8), (0.8, 1.2), (1.2, 1.6), (1.6, math.inf))
BIN_LABELS = ('0.0-0.4', '0.4-0.8', '0.8-1.2', '1.2-1.6', '1.6+')
RESIDUAL_HIST_MM = 50            # 0..50mm, 1mm 一桶
INCIDENCE_HIST_DEG = 90          # 0..90°, 1° 一桶
CORNER_HIST_MM = 200             # 0..200mm, 1mm 一桶


def _hist_add(hist, value, scale):
    idx = int(value / scale)
    if 0 <= idx < len(hist):
        hist[idx] += 1


def _bin_index(r):
    for i, (lo, hi) in enumerate(RANGE_BINS):
        if lo <= r < hi:
            return i
    return len(RANGE_BINS) - 1


class FrameAccumulator:
    """单帧聚合 → 可 JSON 序列化的 frame record (写入 frames.jsonl)."""

    def __init__(self):
        self.stamp = None
        self.n_rays = 0
        self.n_valid = 0
        self.outcomes = {UNIQUE: 0, AMBIGUOUS: 0, NONE: 0, 'OPEN': 0}
        self.reasons = {}
        self.edge_counts = {}
        self.open_edge_counts = {}
        self.bins = [{'n': 0, 'unique': 0, 'ambiguous': 0,
                      'residual_hist': [0] * RESIDUAL_HIST_MM,
                      'incidence_hist': [0] * INCIDENCE_HIST_DEG,
                      'corner_hist': [0] * CORNER_HIST_MM}
                     for _ in RANGE_BINS]

    def accumulate(self, observations, stamp):
        self.stamp = stamp
        self.n_rays = len(observations)
        for obs in observations:
            if obs.outcome == NONE and obs.reason in ('INVALID_RANGE',
                                                      'OUT_OF_RANGE',
                                                      'NO_TRANSFORM'):
                continue                       # 无效/未变换 beam 不进统计桶
            self.n_valid += 1
            self.outcomes[obs.outcome] = self.outcomes.get(obs.outcome, 0) + 1
            if obs.reason:
                self.reasons[obs.reason] = self.reasons.get(obs.reason, 0) + 1
            if obs.candidate is not None:
                cand = obs.candidate
                b = self.bins[_bin_index(cand.range)]
                b['n'] += 1
                if obs.outcome == UNIQUE:
                    b['unique'] += 1
                    self.edge_counts[str(cand.edge_id)] = \
                        self.edge_counts.get(str(cand.edge_id), 0) + 1
                elif obs.outcome == AMBIGUOUS:
                    b['ambiguous'] += 1
                _hist_add(b['residual_hist'], cand.residual, 0.001)
                _hist_add(b['incidence_hist'], cand.incidence_angle,
                          math.pi / 180.0)
                _hist_add(b['corner_hist'], cand.distance_to_corner, 0.001)
            if obs.open_edges:
                self.outcomes['OPEN'] = self.outcomes.get('OPEN', 0) + 1
                for e in obs.open_edges:
                    self.open_edge_counts[str(e)] = \
                        self.open_edge_counts.get(str(e), 0) + 1

    def to_json(self):
        return {
            'stamp': self.stamp,
            'n_rays': self.n_rays,
            'n_valid': self.n_valid,
            'outcomes': self.outcomes,
            'reasons': self.reasons,
            'edge_counts': self.edge_counts,
            'open_edge_counts': self.open_edge_counts,
            'bins': [{'n': b['n'], 'unique': b['unique'],
                      'ambiguous': b['ambiguous'],
                      'residual_hist': b["residual_hist"],
                      'incidence_hist': b["incidence_hist"],
                      'corner_hist': b["corner_hist"]}
                     for b in self.bins],
        }


def summarize_frames(frames):
    """frames.jsonl 记录列表 → 汇总 summary dict (确定性, 无时间依赖)。

    输出 unique/ambiguous rate、残差分位数 (p50/p90/p95)、按距离桶拆分、
    拒绝原因直方图、per-edge 观测计数。绝不输出"可信距离"结论。"""
    total = {'n_rays': 0, 'n_valid': 0}
    outcomes = {UNIQUE: 0, AMBIGUOUS: 0, NONE: 0, 'OPEN': 0}
    reasons = {}
    edge_counts = {}
    open_edge_counts = {}
    bin_stats = [{'n': 0, 'unique': 0, 'ambiguous': 0,
                  'residual_hist': [0] * RESIDUAL_HIST_MM,
                  'incidence_hist': [0] * INCIDENCE_HIST_DEG,
                  'corner_hist': [0] * CORNER_HIST_MM}
                 for _ in RANGE_BINS]
    n_frames = 0
    for f in frames:
        n_frames += 1
        for k in total:
            total[k] += f.get(k, 0)
        for k, v in f.get('outcomes', {}).items():
            outcomes[k] = outcomes.get(k, 0) + v
        for k, v in f.get('reasons', {}).items():
            reasons[k] = reasons.get(k, 0) + v
        for k, v in f.get('edge_counts', {}).items():
            edge_counts[k] = edge_counts.get(k, 0) + v
        for k, v in f.get('open_edge_counts', {}).items():
            open_edge_counts[k] = open_edge_counts.get(k, 0) + v
        for i, b in enumerate(f.get('bins', [])):
            bs = bin_stats[i]
            for k in ('n', 'unique', 'ambiguous'):
                bs[k] += b.get(k, 0)
            for hk in ('residual_hist', 'incidence_hist', 'corner_hist'):
                for j, n in enumerate(b.get(hk, [])):
                    bs[hk][j] += n

    def q(hist, qv, scale):
        t = sum(hist)
        if t == 0:
            return None
        target = math.ceil(qv * t)
        acc = 0
        for i, n in enumerate(hist):
            acc += n
            if acc >= target:
                return round((i + 0.5) * scale, 4)
        return None

    bin_out = []
    for label, bs in zip(BIN_LABELS, bin_stats):
        bin_out.append({
            'range_m': label,
            'n': bs['n'],
            'unique': bs['unique'],
            'ambiguous': bs['ambiguous'],
            'unique_rate': round(bs['unique'] / bs['n'], 4) if bs['n'] else None,
            'ambiguous_rate': round(bs['ambiguous'] / bs['n'], 4) if bs['n'] else None,
            'residual_p50_mm': q(bs['residual_hist'], 0.50, 1),
            'residual_p90_mm': q(bs['residual_hist'], 0.90, 1),
            'residual_p95_mm': q(bs['residual_hist'], 0.95, 1),
            'incidence_p50_deg': q(bs['incidence_hist'], 0.50, 1),
            'incidence_p90_deg': q(bs['incidence_hist'], 0.90, 1),
            'corner_p10_mm': q(bs['corner_hist'], 0.10, 1),
        })
    n_valid = total['n_valid']
    return {
        'n_frames': n_frames,
        'n_rays': total['n_rays'],
        'n_valid': n_valid,
        'unique_rate': round(outcomes[UNIQUE] / n_valid, 4) if n_valid else None,
        'ambiguous_rate': round(outcomes[AMBIGUOUS] / n_valid, 4) if n_valid else None,
        'none_rate': round(outcomes[NONE] / n_valid, 4) if n_valid else None,
        'outcomes': outcomes,
        'reject_reasons': dict(sorted(reasons.items(),
                                      key=lambda kv: -kv[1])),
        'by_range_bin': bin_out,
        'per_edge_counts': dict(sorted(edge_counts.items(),
                                       key=lambda kv: -kv[1])),
        'open_edge_counts': dict(sorted(open_edge_counts.items(),
                                        key=lambda kv: -kv[1])),
        'conclusion': 'NONE — trusted thresholds must be decided by a human '
                      'after reviewing this data; this summary never '
                      'auto-derives a trusted range.',
    }
