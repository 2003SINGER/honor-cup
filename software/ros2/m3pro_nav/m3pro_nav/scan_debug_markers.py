#!/usr/bin/env python3
"""scan_debug 批量 Marker 构建 (纯 Python, 无 ROS).

输出与 ROS 无关的 marker 描述 dict 列表, ROS 节点只做消息转换。
铁律: 禁止每 beam 一个 Marker —— 一帧固定 6 个批量 Marker
(POINTS / LINE_LIST), RViz 只是观察同一批数据, 不是算法数据通路。"""

from .frame_projector import FIELD_N, FIELD_SIZE
from .pose import C
from .grid_association import UNIQUE, AMBIGUOUS

MARKER_GRID = 0
MARKER_RAYS = 1
MARKER_HITS_UNIQUE = 2
MARKER_HITS_AMBIGUOUS = 3
MARKER_WALL_CANDIDATES = 4
MARKER_OPEN_CANDIDATES = 5

COLORS = {
    MARKER_GRID: (0.35, 0.35, 0.35, 0.6),
    MARKER_RAYS: (0.5, 0.5, 0.5, 0.25),
    MARKER_HITS_UNIQUE: (0.0, 0.9, 0.0, 0.9),
    MARKER_HITS_AMBIGUOUS: (1.0, 0.8, 0.0, 0.9),
    MARKER_WALL_CANDIDATES: (0.9, 0.2, 0.2, 0.9),
    MARKER_OPEN_CANDIDATES: (0.2, 0.5, 1.0, 0.8),
}


def _marker(mid, mtype, points, color, scale):
    return {'id': mid, 'type': mtype, 'points': points, 'color': color,
            'scale': scale}


def build_markers(world_rays, observations):
    """一帧 WorldRay[] + DiagnosticObservation[] → 批量 marker 描述列表.

    world_rays 为 None (NO_TRANSFORM) 时只画网格。"""
    markers = [_marker(
        MARKER_GRID, 'LINE_LIST',
        _grid_lines(), COLORS[MARKER_GRID], 0.005)]
    if world_rays is None:
        return markers

    # rays: origin → hit (灰, 半透明)
    ray_pts = []
    hit_unique, hit_ambig = [], []
    for ray, obs in zip(world_rays, observations):
        if not ray.valid:
            continue
        ray_pts.extend([(ray.ox, ray.oy), (ray.hx, ray.hy)])
        if obs.outcome == UNIQUE:
            hit_unique.append((ray.hx, ray.hy))
        elif obs.outcome == AMBIGUOUS:
            hit_ambig.append((ray.hx, ray.hy))
    if ray_pts:
        markers.append(_marker(MARKER_RAYS, 'LINE_LIST', ray_pts,
                               COLORS[MARKER_RAYS], 0.001))
    if hit_unique:
        markers.append(_marker(MARKER_HITS_UNIQUE, 'POINTS', hit_unique,
                               COLORS[MARKER_HITS_UNIQUE], 0.02))
    if hit_ambig:
        markers.append(_marker(MARKER_HITS_AMBIGUOUS, 'POINTS', hit_ambig,
                               COLORS[MARKER_HITS_AMBIGUOUS], 0.02))

    # UNIQUE 候选墙段: 沿 canonical line 的被观测区间 (红)
    wall_segs = {}
    for obs in observations:
        if obs.outcome == UNIQUE and obs.candidate is not None:
            cand = obs.candidate
            key = str(cand.edge_id)
            lo, hi, _ = wall_segs.get(key, (math_inf(), -math_inf(), None))
            lo = min(lo, cand.along_edge - 0.02)
            hi = max(hi, cand.along_edge + 0.02)
            wall_segs[key] = (lo, hi, cand)
    wall_pts = []
    for lo, hi, cand in wall_segs.values():
        if cand.orientation == 'V':
            x = round(cand.hit_x / C) * C
            wall_pts.extend([(x, lo), (x, hi)])
        else:
            y = round(cand.hit_y / C) * C
            wall_pts.extend([(lo, y), (hi, y)])
    if wall_pts:
        markers.append(_marker(MARKER_WALL_CANDIDATES, 'LINE_LIST', wall_pts,
                               COLORS[MARKER_WALL_CANDIDATES], 0.02))

    # OPEN 证据: 被穿越 edge 的中点短线 (蓝)
    open_mid = {}
    for obs in observations:
        for e in obs.open_edges:
            open_mid[str(e)] = open_mid.get(str(e), 0) + 1
    open_pts = []
    for key, n in open_mid.items():
        edge = eval_edge_key(key)
        p0, p1 = edge_endpoints(edge)
        mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        L = (dx * dx + dy * dy) ** 0.5
        ux, uy = dx / L, dy / L
        open_pts.extend([(mid[0] - ux * 0.03, mid[1] - uy * 0.03),
                         (mid[0] + ux * 0.03, mid[1] + uy * 0.03)])
    if open_pts:
        markers.append(_marker(MARKER_OPEN_CANDIDATES, 'LINE_LIST', open_pts,
                               COLORS[MARKER_OPEN_CANDIDATES], 0.01))
    return markers


def math_inf():
    return float('inf')


def _grid_lines():
    pts = []
    for k in range(FIELD_N + 1):
        v = k * C
        pts.extend([(v, 0.0), (v, FIELD_SIZE)])
        pts.extend([(0.0, v), (FIELD_SIZE, v)])
    return pts


def eval_edge_key(key_str):
    """edge_counts 的 str(key) → tuple key (与 EdgeMap canonical 同构)."""
    import ast
    return ast.literal_eval(key_str)


def edge_endpoints(edge_id):
    """canonical edge key → 迷宫坐标端点 (画 OPEN 短线用)."""
    if edge_id[0] == 'B':
        _, cell, d = edge_id
    else:
        cell, d = edge_id
    i, j = cell
    from .pose import DIRV
    dv = DIRV[d]
    # cell 的 d 边: 从格角出发的 0.4m 线段
    if dv == (0, 1):                     # N
        return ((i * C, (j + 1) * C), ((i + 1) * C, (j + 1) * C))
    if dv == (0, -1):                    # S
        return ((i * C, j * C), ((i + 1) * C, j * C))
    if dv == (1, 0):                     # E
        return (((i + 1) * C, j * C), ((i + 1) * C, (j + 1) * C))
    return ((i * C, j * C), (i * C, (j + 1) * C))          # W
