#!/usr/bin/env python3
"""RealObservationAdapter —— WorldRay[] (maze 帧) → StreamNav.observe 接口.

把 scan_debug 工作台同一条感知管线 (ScanAdapter→FrameProjector→
GridAssociation) 的输出转换成核心算法消费的离散观测:

    hits  = {(cell, axis): (dist, alpha)}          WALL 证据 (calibrated hit)
    opens = [(cell, axis, dist, alpha)]             OPEN 证据 (free path 穿越)

语义对齐仿真端 sense_from:
  dist  = 车到该墙/边线沿轴向的垂直距离 (米)
  alpha = 射线与墙法线夹角 (rad, 0=正对) —— 真实几何, 非模拟噪声;
          StreamNav._err 的信任模型用它自然拒绝掠射观测
  edge key 与 EdgeMap canonical 同构 (interior (cell,d) / boundary ('B',cell,d))

与仿真的差别 (诚实记录):
  - 仿真每轴沿走廊顺序扫到第一面墙 (语义完美遮挡); 真实 ray 天然物理遮挡;
  - 仿真的 alpha 是噪声模型; 真实 alpha 是几何入射角;
  - 诊断标签不控制正式地图写入; 原始候选由 TrustPolicy 独立判断;
  - 无回波射线 (OUT_OF_RANGE 且 range 有限) 只贡献 free-path OPEN 证据
    (合成 capped ray), 绝不当作墙面。

本模块纯 Python, 无 ROS; 单元测试用合成 ray 验证地图构建正确性。"""

import math
from dataclasses import dataclass, field

from .frame_projector import WorldRay
from .grid_association import GridAssociation, UNIQUE
from .pose import C, DIRV


def _axis_of_edge(edge_id):
    """canonical edge key → (cell, direction) 供 observe_wall/open 消费."""
    if edge_id[0] == 'B':
        _, cell, d = edge_id
        return cell, d
    cell, d = edge_id
    return cell, d


def _is_vertical(edge_id):
    """edge 朝向: E/W → vertical line (x = k·C); N/S → horizontal."""
    _, d = _axis_of_edge(edge_id)
    return d in ('E', 'W')


def _line_k(edge_id):
    """edge 所在 canonical line 的 k (格线序号)."""
    cell, d = _axis_of_edge(edge_id)
    i, j = cell
    return {'E': i + 1, 'W': i, 'N': j + 1, 'S': j}[d]


@dataclass
class FrameObservationStats:
    """单帧观测统计 (写 evidence 日志, 供事后 LLM 分析)."""
    n_rays: int = 0
    n_valid: int = 0
    n_unique_wall: int = 0
    n_ambiguous: int = 0
    n_rejected: int = 0
    n_open_edges: int = 0
    n_raw_open_edges: int = 0
    n_trusted_wall_edges: int = 0
    alignment_mode: str | None = None
    odom_skew_s: float | None = None
    reject_reasons: dict = field(default_factory=dict)

    def to_json(self):
        return {'n_rays': self.n_rays, 'n_valid': self.n_valid,
                'n_unique_wall': self.n_unique_wall,
                'n_ambiguous': self.n_ambiguous,
                'n_rejected': self.n_rejected,
                'n_open_edges': self.n_open_edges,
                'n_raw_open_edges': self.n_raw_open_edges,
                'n_trusted_wall_edges': self.n_trusted_wall_edges,
                'alignment_mode': self.alignment_mode,
                'odom_skew_s': self.odom_skew_s,
                'reject_reasons': self.reject_reasons}


class RealObservationAdapter:
    def __init__(self, association=None, *, free_path_cap_m=3.0,
                 trust_policy=None, allow_open_evidence=True):
        self.association = association or GridAssociation()
        self.free_path_cap_m = free_path_cap_m
        self.trust_policy = trust_policy
        self.allow_open_evidence = bool(allow_open_evidence)

    def to_nav_observation(self, world_rays, pose, *, stamp=None):
        """WorldRay[] + 当前 maze 位姿 → (hits, opens, stats).

        world_rays 为 None (无外参) → 全体 ABSTAIN, 返回空观测."""
        if world_rays is None:
            return {}, [], FrameObservationStats()
        if self.trust_policy is not None and stamp is None:
            raise ValueError('source scan stamp is required for trusted observations')
        stats = FrameObservationStats(n_rays=len(world_rays))
        real, synthetic = [], []
        for ray in world_rays:
            if ray.valid:
                real.append(ray)
                stats.n_valid += 1
            elif (ray.invalid_reason == 'OUT_OF_RANGE'
                    and math.isfinite(ray.range)):
                # 无回波: 合成 capped free-path ray (只出 OPEN, 不出 WALL)
                cap = min(ray.range, self.free_path_cap_m)
                synthetic.append(WorldRay(
                    ray.index, ray.ox, ray.oy, ray.dir_x, ray.dir_y,
                    ray.ox + ray.dir_x * cap, ray.oy + ray.dir_y * cap,
                    cap, None))
                stats.n_valid += 1
            else:
                stats.reject_reasons[ray.invalid_reason] = \
                    stats.reject_reasons.get(ray.invalid_reason, 0) + 1

        hits = {}
        raw_wall_best = {}                # edge → representative raw hit
        open_best = {}                    # edge → (OPEN evidence, source ray)

        def keep_open(edge, ev, ray):
            # The most normal ray can cross right at a cell corner and fail a
            # calibrated corner guard while other rays cross the same edge
            # well inside its finite segment. Prefer finite-edge support in
            # the trusted path; the raw diagnostic path retains its old rank.
            if self.trust_policy is None:
                rank = (-ev[3],)
            else:
                travel, corner = self._open_features(edge, ray)
                rank = (corner, -ev[3], -travel)
            if edge not in open_best or rank > open_best[edge][2]:
                open_best[edge] = (ev, ray, rank)
        # 1) Keep diagnostic classifications for telemetry, but select the
        # raw best candidate for the separately calibrated trust policy.
        observations = self.association.process(real) if real else []
        for ray, obs in zip(real, observations):
            cand = (obs.candidate if obs.outcome == UNIQUE else
                    (obs.candidates[0] if self.trust_policy is not None
                     and obs.candidates else None))
            if cand is not None:
                edge = cand.edge_id
                cell, d = _axis_of_edge(edge)
                if cand.orientation == 'V':
                    dist = abs(round(cand.hit_x / C) * C - pose.x)
                else:
                    dist = abs(round(cand.hit_y / C) * C - pose.y)
                # 同一物理边每帧只挑一个 raw 候选送进信任策略。
                candidate_rank = (
                    bool(self.trust_policy and
                         self.trust_policy.plausible_wall(obs)),
                    -cand.residual, -cand.incidence_angle)
                old = raw_wall_best.get(edge)
                old_rank = ((bool(self.trust_policy and
                                  self.trust_policy.plausible_wall(old)),
                             -old.candidates[0].residual,
                             -old.candidates[0].incidence_angle)
                            if old is not None else None)
                if old_rank is None or candidate_rank > old_rank:
                    raw_wall_best[edge] = obs
                    hits[(cell, d)] = (dist, cand.incidence_angle)
            if obs.outcome == UNIQUE:
                stats.n_unique_wall += 1
            else:
                if obs.reason:
                    stats.reject_reasons[obs.reason] = \
                        stats.reject_reasons.get(obs.reason, 0) + 1
                if obs.outcome == 'AMBIGUOUS':
                    stats.n_ambiguous += 1
                else:
                    stats.n_rejected += 1
            if self.allow_open_evidence:
                for edge in obs.open_edges:
                    ev = self._open_evidence((edge,), ray, pose)[0]
                    keep_open(edge, ev, ray)
        # 2) 无回波射线: 只出 free-path OPEN 证据 (同帧逐边一票)
        if self.allow_open_evidence:
            for ray in synthetic:
                for edge in self.association.open_edges_along(ray):
                    ev = self._open_evidence((edge,), ray, pose)[0]
                    keep_open(edge, ev, ray)
        # A plausible raw hit vetoes same-frame OPEN, even if it has not yet
        # accumulated enough distinct scans for a WALL commit.
        raw_wall_edges = {
            edge for edge, obs in raw_wall_best.items()
            if self.trust_policy is None or
            self.trust_policy.plausible_wall(obs)
        }
        if self.trust_policy is not None:
            trusted_hits = {}
            for edge, obs in raw_wall_best.items():
                decision = self.trust_policy.evaluate(obs, stamp=stamp)
                if decision.accepted:
                    nav_edge = _axis_of_edge(edge)
                    trusted_hits[nav_edge] = hits[nav_edge]
                else:
                    stats.reject_reasons[decision.reason] = \
                        stats.reject_reasons.get(decision.reason, 0) + 1
            hits = trusted_hits
        stats.n_trusted_wall_edges = len(hits)
        stats.n_raw_open_edges = sum(edge not in raw_wall_edges
                                     for edge in open_best)
        opens = []
        for edge, (ev, ray, _) in open_best.items():
            if edge in raw_wall_edges:
                continue
            if self.trust_policy is not None:
                range_m, corner_distance = self._open_features(edge, ray)
                decision = self.trust_policy.evaluate_open_edge(
                    edge, stamp=stamp, range_m=range_m,
                    incidence_angle=ev[3],
                    corner_distance_m=corner_distance)
                if not decision.accepted:
                    stats.reject_reasons[decision.reason] = \
                        stats.reject_reasons.get(decision.reason, 0) + 1
                    continue
            opens.append(ev)
        stats.n_open_edges = len(opens)
        return hits, opens, stats

    def _open_features(self, edge, ray):
        cell, direction = _axis_of_edge(edge)
        line = _line_k(edge) * C
        if direction in ('E', 'W'):
            travel = (line - ray.ox) / ray.dir_x
            along = ray.oy + travel * ray.dir_y
            segment_index = cell[1]
        else:
            travel = (line - ray.oy) / ray.dir_y
            along = ray.ox + travel * ray.dir_x
            segment_index = cell[0]
        corner = min(along - segment_index * C,
                     (segment_index + 1) * C - along)
        return abs(travel), max(0.0, corner)

    def _open_evidence(self, edges, ray, pose):
        out = []
        for edge in edges:
            cell, d = _axis_of_edge(edge)
            k = _line_k(edge)
            if _is_vertical(edge):
                dist = abs(k * C - pose.x)
                alpha = math.acos(max(-1.0, min(1.0, abs(ray.dir_x))))
            else:
                dist = abs(k * C - pose.y)
                alpha = math.acos(max(-1.0, min(1.0, abs(ray.dir_y))))
            out.append((cell, d, dist, alpha))
        return out
