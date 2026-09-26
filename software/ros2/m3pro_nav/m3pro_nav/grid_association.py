#!/usr/bin/env python3
"""GridAssociation —— maze frame hit/free-path → canonical edge 候选 (纯 Python).

正式比赛网格: CELL = 0.4m, 7×7 场地, canonical lines x = k·0.4 / y = k·0.4.
edge_id 与 EdgeMap.edge_key 同一 canonical 约定 (同一条物理边一个 key),
未来 ObservationAdapter 可直接落 EdgeMap.

铁律 (GPT 感知工作台定稿):
  - "最近墙"绝不自动当答案: 结果分 UNIQUE / AMBIGUOUS / NONE,
    两候选不可靠区分时必须 ABSTAIN (AMBIGUOUS);
  - 每个候选/拒绝都携带标定要用的原始特征:
    residual / range / beam angle / incidence angle / along-edge /
    distance-to-corner / uniqueness margin;
  - hit 之后的区域不做任何推理 (no reasoning beyond hit);
  - 本模块只产生 diagnostic evidence, 绝不写 EdgeMap。"""

from dataclasses import dataclass
import math

from .frame_projector import FIELD_N, FIELD_SIZE, WorldRay
from .pose import C, DIRV, OPP

# 判定阈值: 仅用于"诊断分层"(决定显示为什么颜色/统计进哪一桶),
# 未标定前绝不作为信任判据 —— TrustPolicy 默认 diagnostic_only.
DEFAULT_MAX_RESIDUAL = 0.05      # 超过 → NONE / LARGE_RESIDUAL
DEFAULT_AMBIGUITY_MARGIN = 0.02  # 最佳/次佳差小于此 → AMBIGUOUS_EDGE
DEFAULT_CORNER_GUARD = 0.05      # 投影点离格角小于此 → NEAR_CORNER

UNIQUE, AMBIGUOUS, NONE = 'UNIQUE', 'AMBIGUOUS', 'NONE'

_CROSS_TOL = 1e-6                # 端点"恰在线上"容差 (浮点噪声远小于此)


def edge_id_for_line(orientation, line_k, seg_j):
    """canonical line (orientation, k, j) → EdgeMap 同构 canonical edge key.

    vertical line x = k·0.4, 段 y∈[j·0.4,(j+1)·0.4]:
      分隔 cells (k-1, j) 与 (k, j) —— 取字典序较小格 + 其指向较大格的方向;
      k=0/7 为场地边界 ('B', cell, dir)。horizontal 同理。"""
    if orientation == 'V':
        if line_k <= 0:
            return ('B', (0, seg_j), 'W')
        if line_k >= FIELD_N:
            return ('B', (FIELD_N - 1, seg_j), 'E')
        a, b = (line_k - 1, seg_j), (line_k, seg_j)
        d = 'E'                     # a 的东边
    else:
        if line_k <= 0:
            return ('B', (seg_j, 0), 'S')
        if line_k >= FIELD_N:
            return ('B', (seg_j, FIELD_N - 1), 'N')
        a, b = (seg_j, line_k - 1), (seg_j, line_k)
        d = 'N'                     # a 的北边
    return (a, d) if a <= b else (b, OPP[d])


@dataclass(frozen=True)
class EdgeCandidate:
    """单个 canonical edge 候选 + 全部标定特征."""
    edge_id: tuple
    orientation: str               # 'V' (x=k·C) / 'H' (y=k·C)
    residual: float                # hit 到该线的垂直距离 (m)
    range: float                   # ray 距离
    beam_angle: float              # ray 世界系方向角 (rad)
    incidence_angle: float         # 射线与墙法线夹角 (rad, 0=正对)
    along_edge: float              # 投影点沿边的坐标 (m)
    distance_to_corner: float      # 投影点到最近格角的距离 (m)
    hit_x: float
    hit_y: float


@dataclass(frozen=True)
class DiagnosticObservation:
    """一条 ray 的网格关联诊断结果 (不写 EdgeMap, 只统计/显示)."""
    ray_index: int
    outcome: str                   # UNIQUE / AMBIGUOUS / NONE / OPEN
    reason: str | None             # 拒绝原因 (NONE/AMBIGUOUS 时必有)
    candidate: EdgeCandidate | None    # UNIQUE 时的最佳候选
    candidates: tuple                   # 全部候选 (UNIQUE/AMBIGUOUS)
    open_edges: tuple                   # free path 穿过的 canonical edges

    # 拒绝原因枚举: INVALID_RANGE / NO_TRANSFORM / OUT_OF_FIELD /
    # NO_GRID_CANDIDATE / AMBIGUOUS_EDGE / LARGE_RESIDUAL / NEAR_CORNER


class GridAssociation:
    def __init__(self, *, max_residual=DEFAULT_MAX_RESIDUAL,
                 ambiguity_margin=DEFAULT_AMBIGUITY_MARGIN,
                 corner_guard=DEFAULT_CORNER_GUARD):
        self.max_residual = max_residual
        self.ambiguity_margin = ambiguity_margin
        self.corner_guard = corner_guard

    # ---- hit → 候选 ----

    def _candidates_for_hit(self, ray: WorldRay):
        """hit 点的 vertical/horizontal 候选 (各至多一条, 带全部特征)."""
        out = []
        beam_ang = math.atan2(ray.dir_y, ray.dir_x)
        for orientation, v, dv in (('V', ray.hx, ray.dir_x),
                                   ('H', ray.hy, ray.dir_y)):
            k = round(v / C)
            if k < 0 or k > FIELD_N:
                continue
            line = k * C
            residual = abs(v - line)
            if residual > self.max_residual + 1e-12 and residual > 0.5 * C:
                # 离最近线超过半格 → 该方向无候选
                continue
            along = ray.hy if orientation == 'V' else ray.hx
            if not (0.0 <= along <= FIELD_SIZE):
                continue                       # 投影落在场外: 该线无对应边
            seg_j = min(int(along / C), FIELD_N - 1)
            # incidence: 射线方向与墙法线夹角 (0 = 正对墙)
            normal_comp = dv if orientation == 'V' else dv
            incidence = math.acos(max(-1.0, min(1.0, abs(normal_comp))))
            corner = min(along - seg_j * C, (seg_j + 1) * C - along)
            out.append(EdgeCandidate(
                edge_id_for_line(orientation, k, seg_j),
                orientation, residual, ray.range, beam_ang,
                incidence, along, corner, ray.hx, ray.hy))
        return out

    def _in_field(self, x, y):
        return 0.0 <= x <= FIELD_SIZE and 0.0 <= y <= FIELD_SIZE

    def associate_hit(self, ray: WorldRay) -> DiagnosticObservation:
        """单条有效 ray 的 hit 关联 (UNIQUE / AMBIGUOUS / NONE + 原因)."""
        if not ray.valid:
            return DiagnosticObservation(ray.index, NONE,
                                         ray.invalid_reason, None, (), ())
        if not self._in_field(ray.hx, ray.hy):
            return DiagnosticObservation(ray.index, NONE, 'OUT_OF_FIELD',
                                         None, (), ())
        cands = self._candidates_for_hit(ray)
        if not cands:
            return DiagnosticObservation(ray.index, NONE, 'NO_GRID_CANDIDATE',
                                         None, (), ())
        cands = tuple(sorted(cands, key=lambda c: c.residual))
        best = cands[0]
        if best.residual > self.max_residual:
            return DiagnosticObservation(ray.index, NONE, 'LARGE_RESIDUAL',
                                         None, cands, ())
        margin = (cands[1].residual - best.residual if len(cands) > 1
                  else math.inf)
        if margin < self.ambiguity_margin:
            return DiagnosticObservation(ray.index, AMBIGUOUS, 'AMBIGUOUS_EDGE',
                                         None, cands, ())
        if best.distance_to_corner < self.corner_guard:
            return DiagnosticObservation(ray.index, NONE, 'NEAR_CORNER',
                                         None, cands, ())
        return DiagnosticObservation(ray.index, UNIQUE, None, best, cands, ())

    # ---- free path → OPEN 证据 ----

    def open_edges_along(self, ray: WorldRay):
        """origin→hit 穿过的 canonical edges (hit 前的 free 空间证据).

        hit 之后不做推理; 无效 ray 不产生任何证据.
        端点容差: 迷宫墙面恰在 canonical line 上, 正对命中时 hit 浮点噪声
        可能落在线两侧 ±1e-16 —— 必须视为"打在线上"而非穿越 (否则会给
        有墙的边投 OPEN 票)。origin 恰在线上 (边界停靠) 同理不算穿越。"""
        if not ray.valid:
            return ()
        edges = []
        for orientation in ('V', 'H'):
            for k in range(FIELD_N + 1):
                line = k * C
                if orientation == 'V':
                    a, b = ray.ox - line, ray.hx - line
                else:
                    a, b = ray.oy - line, ray.hy - line
                if abs(a) < _CROSS_TOL or abs(b) < _CROSS_TOL:
                    continue                   # 端点恰在线上: 打在墙上, 非穿越
                if a * b >= 0:
                    continue                   # 未跨越该线
                t = -a / (b - a)
                cx = ray.ox + t * (ray.hx - ray.ox)
                cy = ray.oy + t * (ray.hy - ray.oy)
                along = cy if orientation == 'V' else cx
                if not (0.0 <= along <= FIELD_SIZE):
                    continue
                seg_j = min(int(along / C), FIELD_N - 1)
                edges.append(edge_id_for_line(orientation, k, seg_j))
        return tuple(edges)

    def process(self, world_rays):
        """整帧 WorldRay[] → DiagnosticObservation[] (hit 关联 + OPEN 证据)."""
        results = []
        for ray in world_rays:
            obs = self.associate_hit(ray)
            edges = self.open_edges_along(ray)
            if edges and obs.outcome != NONE:
                obs = DiagnosticObservation(
                    obs.ray_index, obs.outcome, obs.reason,
                    obs.candidate, obs.candidates, edges)
            elif edges and obs.outcome == NONE:
                # hit 无效/出界, 但 free path 仍有效 (INVALID_RANGE 时无 hit,
                # 不推理; OUT_OF_FIELD 的 hit 之前路径仍在场内)
                if obs.reason == 'OUT_OF_FIELD':
                    obs = DiagnosticObservation(
                        obs.ray_index, obs.outcome, obs.reason,
                        obs.candidate, obs.candidates, edges)
            results.append(obs)
        return results
