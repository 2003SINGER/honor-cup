"""Small, truth-free temporal checks for canonical WALL evidence.

This module only consumes post-snap edge-local endpoint summaries. It does not
change pose correction, build a map track, or treat repeated frames as new
viewpoints. Callers keep temporal continuity and independent-view credits as
separate quantities.
"""
from __future__ import annotations

import math
import statistics


def _edge_geometry(edge, cell_size):
    """Return (orientation, grid-normal coordinate, tangent-cell index)."""
    if edge[0] == 'B':
        _, cell, direction = edge
    else:
        cell, direction = edge
    x, y = cell
    if direction == 'E': return 'V', (x + 1) * cell_size, y
    if direction == 'W': return 'V', x * cell_size, y
    if direction == 'N': return 'H', (y + 1) * cell_size, x
    if direction == 'S': return 'H', y * cell_size, x
    raise ValueError(f'unknown edge direction: {direction!r}')


def _world_points(edge, hit, cell_size):
    tangent = hit.get('roi_tangent_m')
    normal = hit.get('roi_signed_normal_m')
    if not isinstance(tangent, list) or not isinstance(normal, list):
        return []
    orient, grid_normal, _ = _edge_geometry(edge, cell_size)
    if orient == 'V':
        return [(grid_normal + float(n), float(t))
                for t, n in zip(tangent, normal)]
    return [(float(t), grid_normal + float(n))
            for t, n in zip(tangent, normal)]


def _median_nearest_distance(a, b):
    if not a or not b:
        return math.inf
    nearest = [min(math.dist(p, q) for q in b) for p in a]
    return statistics.median(nearest)


def _same_physical_support(previous, current, cell_size, max_point_shift_m,
                           max_normal_shift_m):
    prev_edge, prev_hit = previous['edge'], previous['hit']
    curr_edge, curr_hit = current['edge'], current['hit']
    prev_orient, prev_normal, _ = _edge_geometry(prev_edge, cell_size)
    curr_orient, curr_normal, _ = _edge_geometry(curr_edge, cell_size)
    if prev_orient != curr_orient:
        return False
    # Compare measured world normal positions, not just canonical edge IDs.
    pn = prev_normal + float(prev_hit.get('normal_residual_m') or 0.)
    cn = curr_normal + float(curr_hit.get('normal_residual_m') or 0.)
    if abs(pn - cn) > max_normal_shift_m:
        return False
    a = _world_points(prev_edge, prev_hit, cell_size)
    b = _world_points(curr_edge, curr_hit, cell_size)
    return min(_median_nearest_distance(a, b),
               _median_nearest_distance(b, a)) <= max_point_shift_m


def add_temporal_evidence(edge_votes, *, cell_size=0.4, max_gap_s=0.25,
                          max_point_shift_m=0.08, max_normal_shift_m=0.05,
                          max_jump_translation_m=0.20,
                          max_jump_yaw_rad=math.radians(8)):
    """Annotate frame votes with temporal streak and impossible-ID-jump flags.

    A streak requires the same edge in adjacent logged frames, a short time
    gap, stable measured wall normal, and overlapping physical endpoint
    support. A neighboring edge ID is marked for abstention only when the old
    ID disappears and the endpoint cloud itself continues across the shared
    cell boundary under a small pose change.
    """
    all_samples = sorted((sample for values in edge_votes.values()
                          for sample in values), key=lambda v: v['frame_index'])
    by_frame = {}
    for sample in all_samples:
        by_frame.setdefault(sample['frame_index'], []).append(sample)
    last_by_edge = {}
    previous_frame = None
    for frame_index in sorted(by_frame):
        samples = by_frame[frame_index]
        current_edges = {sample['edge'] for sample in samples}
        prev_samples = (by_frame.get(previous_frame, [])
                        if previous_frame is not None and frame_index == previous_frame + 1
                        else [])
        dt_by_sample = {}
        for sample in samples:
            sample['temporal_streak'] = 1
            sample['temporal_continuity'] = False
            sample['edge_id_jump_abstain'] = False
            prior = last_by_edge.get(sample['edge'])
            dt = (float(sample['stamp']) - float(prior['stamp'])) if prior else math.inf
            dt_by_sample[id(sample)] = dt
            if (prior and dt > 0 and dt <= max_gap_s and
                    prior['frame_index'] + 1 == frame_index and
                    _same_physical_support(prior, sample, cell_size,
                                           max_point_shift_m,
                                           max_normal_shift_m)):
                sample['temporal_streak'] = prior['temporal_streak'] + 1
                sample['temporal_continuity'] = True

        for sample in samples:
            if sample['temporal_continuity']:
                continue
            edge = sample['edge']
            orient, _, tangent_index = _edge_geometry(edge, cell_size)
            for prior in prev_samples:
                old_edge = prior['edge']
                if old_edge == edge or old_edge in current_edges:
                    continue
                old_orient, _, old_tangent_index = _edge_geometry(old_edge, cell_size)
                if orient != old_orient or abs(tangent_index - old_tangent_index) != 1:
                    continue
                dt = float(sample['stamp']) - float(prior['stamp'])
                if dt <= 0 or dt > max_gap_s:
                    continue
                pose_a, pose_b = prior['pose'], sample['pose']
                if (math.dist(pose_a[:2], pose_b[:2]) > max_jump_translation_m or
                        abs(math.atan2(math.sin(pose_b[2]-pose_a[2]),
                                       math.cos(pose_b[2]-pose_a[2]))) > max_jump_yaw_rad):
                    continue
                if _same_physical_support(prior, sample, cell_size,
                                          max_point_shift_m,
                                          max_normal_shift_m):
                    sample['edge_id_jump_abstain'] = True
                    break

        for sample in samples:
            last_by_edge[sample['edge']] = sample
        previous_frame = frame_index
    return edge_votes
