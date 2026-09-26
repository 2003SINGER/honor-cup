#!/usr/bin/env python3
"""RealObservationAdapter —— WorldRay[] (maze 帧) → StreamNav.observe 接口.

把 scan_debug 工作台同一条感知管线 (ScanAdapter→FrameProjector→
GridAssociation) 的输出转换成核心算法消费的离散观测:

    hits  = {(cell, axis): (dist, alpha)}          WALL 证据 (UNIQUE hit)
    opens = [(cell, axis, dist, alpha)]             OPEN 证据 (free path 穿越)

语义对齐仿真端 sense_from:
  dist  = 车到该墙/边线沿轴向的垂直距离 (米)
  alpha = 射线与墙法线夹角 (rad, 0=正对) —— 真实几何, 非模拟噪声;
          StreamNav._err 的信任模型用它自然拒绝掠射观测
  edge key 与 EdgeMap canonical 同构 (interior (cell,d) / boundary ('B',cell,d))

与仿真的差别 (诚实记录):
  - 仿真每轴沿走廊顺序扫到第一面墙 (语义完美遮挡); 真实 ray 天然物理遮挡;
  - 仿真的 alpha 是噪声模型; 真实 alpha 是几何入射角;
  - AMBIGUOUS/NONE hit → ABSTAIN (不产生任何证据), 与 scan_debug 诊断一致;
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
    reject_reasons: dict = field(default_factory=dict)

    def to_json(self):
        return {'n_rays': self.n_rays, 'n_valid': self.n_valid,
                'n_unique_wall': self.n_unique_wall,
                'n_ambiguous': self.n_ambiguous,
                'n_rejected': self.n_rejected,
                'n_open_edges': self.n_open_edges,
                'reject_reasons': self.reject_reasons}


class RealObservationAdapter:
    def __init__(self, association=None, *, free_path_cap_m=3.0):
        self.association = association or GridAssociation()
        self.free_path_cap_m = free_path_cap_m

    def to_nav_observation(self, world_rays, pose):
        """WorldRay[] + 当前 maze 位姿 → (hits, opens, stats).

        world_rays 为 None (无外参) → 全体 ABSTAIN, 返回空观测."""
        if world_rays is None:
            return {}, [], FrameObservationStats()
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
        hit_alpha = {}                    # edge → 最佳 (最小入射角) 观测
        open_best = {}                    # edge → 最佳 OPEN 观测 (逐帧一票)
        # 1) 有效 hit 关联: UNIQUE → WALL; AMBIGUOUS/NONE → ABSTAIN
        observations = self.association.process(real) if real else []
        for ray, obs in zip(real, observations):
            if obs.outcome == UNIQUE and obs.candidate is not None:
                cand = obs.candidate
                edge = cand.edge_id
                cell, d = _axis_of_edge(edge)
                if cand.orientation == 'V':
                    dist = abs(round(cand.hit_x / C) * C - pose.x)
                else:
                    dist = abs(round(cand.hit_y / C) * C - pose.y)
                # 同一边同帧多 ray: 保正对 (最小入射角) 的那条 —— 掠射
                # 观测的 err 天然大, 不应覆盖正对的好测量
                if edge not in hit_alpha or \
                        cand.incidence_angle < hit_alpha[edge]:
                    hit_alpha[edge] = cand.incidence_angle
                    hits[(cell, d)] = (dist, cand.incidence_angle)
                stats.n_unique_wall += 1
            else:
                if obs.reason:
                    stats.reject_reasons[obs.reason] = \
                        stats.reject_reasons.get(obs.reason, 0) + 1
                if obs.outcome == 'AMBIGUOUS':
                    stats.n_ambiguous += 1
                else:
                    stats.n_rejected += 1
            for ev in self._open_evidence(obs.open_edges, ray, pose):
                edge = (ev[0], ev[1])
                if edge not in open_best or ev[3] < open_best[edge][3]:
                    open_best[edge] = ev
        # 2) 无回波射线: 只出 free-path OPEN 证据 (同帧逐边一票)
        for ray in synthetic:
            for ev in self._open_evidence(
                    self.association.open_edges_along(ray), ray, pose):
                edge = (ev[0], ev[1])
                if edge not in open_best or ev[3] < open_best[edge][3]:
                    open_best[edge] = ev
        opens = list(open_best.values())
        stats.n_open_edges = len(opens)
        return hits, opens, stats

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
