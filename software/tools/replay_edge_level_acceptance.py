#!/usr/bin/env python3
"""Posthoc edge-level SENSOR belief replay from frame-grid-snap JSONL.

Consumes the existing causal replay log (truth was not used to produce it).
Only fitted hit segments can cast WALL evidence. The old /scan_multi point
cloud has no trustworthy free-space semantics, so this tool never casts OPEN.
"""
from __future__ import annotations

import argparse
import ast
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))
sys.path.insert(0, str(ROOT / 'software' / 'tools'))

from m3pro_nav.edge_map import EdgeMap, WALL, UNKNOWN
from m3pro_nav.grid_association import edge_id_for_line
from audit_lidar_odom_calibration import edge_truth
from analyze_scan_edge import CELL_SIZE_M

N = 7
VIEW_POSITION_M = 0.20
VIEW_YAW_RAD = math.radians(20)


def canonical_edges():
    return {edge_id_for_line(o, k, j)
            for o in ('H', 'V') for k in range(N + 1) for j in range(N)}


def edge_text(edge):
    if edge[0] == 'B':
        _, (x, y), d = edge
        return f'B({x},{y},{d})'
    cell, d = edge
    return f'{d}({cell[0]},{cell[1]})'


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _world_segment(seg, source, target):
    """Rigidly move a predicted-world segment onto the accepted corrected pose."""
    sx, sy, st = source
    tx, ty, tt = target
    cs, ss, ct, stn = math.cos(st), math.sin(st), math.cos(tt), math.sin(tt)
    def transform(p):
        dx, dy = p[0] - sx, p[1] - sy
        bx, by = cs * dx + ss * dy, -ss * dx + cs * dy
        return tx + ct * bx - stn * by, ty + stn * bx + ct * by
    return transform(seg['a']), transform(seg['b'])


def _edge_midpoint(edge):
    if edge[0] == 'B':
        _, (x, y), d = edge
        if d == 'W': return (0.0, (y + .5) * CELL_SIZE_M)
        if d == 'E': return (N * CELL_SIZE_M, (y + .5) * CELL_SIZE_M)
        if d == 'S': return ((x + .5) * CELL_SIZE_M, 0.0)
        return ((x + .5) * CELL_SIZE_M, N * CELL_SIZE_M)
    (x, y), d = edge
    if d == 'E': return ((x + 1) * CELL_SIZE_M, (y + .5) * CELL_SIZE_M)
    if d == 'W': return (x * CELL_SIZE_M, (y + .5) * CELL_SIZE_M)
    if d == 'N': return ((x + .5) * CELL_SIZE_M, (y + 1) * CELL_SIZE_M)
    return ((x + .5) * CELL_SIZE_M, y * CELL_SIZE_M)


def _view_clusters(samples):
    """Greedy representative clusters; nearby position AND heading are correlated."""
    reps = []
    for sample in samples:
        pose = sample['pose']
        for i, rep in enumerate(reps):
            rp = rep['pose']
            if (math.dist(pose[:2], rp[:2]) <= VIEW_POSITION_M and
                    abs(_wrap(pose[2] - rp[2])) <= VIEW_YAW_RAD):
                rep['frames'].append(sample['frame_index'])
                break
        else:
            reps.append({'pose': pose, 'frames': [sample['frame_index']]})
    return reps


def replay(log_path, truth_path, json_path, csv_path):
    expected_edges = canonical_edges()
    if len(expected_edges) != 112:
        raise ValueError('unexpected canonical edge inventory')
    edge_frames = defaultdict(dict)
    frame_count = 0
    for frame_index, line in enumerate(Path(log_path).read_text().splitlines()):
        if not line.strip():
            continue
        frame = json.loads(line)
        frame_count += 1
        if 'wall_edge_hits' not in frame:
            raise ValueError('replay JSONL lacks per-cell wall_edge_hits; rerun replay with updated tool')
        corrected = frame['corrected_pose']
        current = defaultdict(list)
        for hit in frame['wall_edge_hits']:
            edge = ast.literal_eval(hit['edge_id'])
            current[edge].append(hit)
        # At most one sensor vote for an edge in one scan/frame, regardless of
        # segment fragmentation or duplicate fits. The representative is the
        # strongest supported segment; this does not change the vote count.
        for edge, observations in current.items():
            chosen = max(observations, key=lambda item: item['support_points'])
            seg_index = chosen['segment_index']
            support = chosen['support_points']
            dist = chosen['distance_m']
            edge_frames[edge][frame_index] = {
                'frame_index': frame_index, 'stamp': frame['stamp'],
                'elapsed_s': frame['elapsed_s'], 'pose': corrected,
                'support_points': support, 'segment_index': seg_index,
                'distance_m': dist,
                'same_frame_piece_count': chosen.get('same_frame_piece_count', len(observations))}

    em = EdgeMap(N)
    # Known field boundary structure only. These four perimeter statements are
    # not the interior maze truth: they are the arena contract (closed outer
    # boundary except the declared entrance and exit).
    for edge in expected_edges:
        if edge[0] != 'B':
            continue
        _, cell, direction = edge
        is_portal = (cell, direction) in {((3, 0), 'S'), ((0, 0), 'W')}
        em.set_boundary(cell, direction, 'OPEN' if is_portal else WALL)
    rows = []
    state_events = defaultdict(list)
    for edge in sorted(expected_edges, key=edge_text):
        observations = edge_frames.get(edge, {})
        votes = sorted(observations.values(), key=lambda x: x['frame_index'])
        if edge[0] == 'B':
            c, d = edge[1], edge[2]
        else:
            c, d = edge[0], edge[1]
        for vote in votes:
            before = em.soft.get(edge, {'state': UNKNOWN})['state']
            em.observe_wall(c, d, dist=vote['distance_m'], stamp=vote['stamp'])
            after = em.soft.get(edge, {'state': UNKNOWN})['state']
            if after != before:
                state_events[edge].append({
                    'frame_index': vote['frame_index'], 'stamp': vote['stamp'],
                    'elapsed_s': vote['elapsed_s'],
                    'from': before, 'to': after,
                    'score': em.soft[edge]['score']})
        final_soft = em.soft.get(edge, {'state': UNKNOWN})['state']
        final_effective = em.state(c, d)
        first_wall = next((e for e in state_events[edge] if e['to'] == WALL), None)
        ever_wall = first_wall is not None
        clusters = _view_clusters(votes)
        history = state_events[edge]
        rows.append({
            '_edge_key': edge,
            'edge_id': edge_text(edge),
            'wall_hits': len(votes), 'open_hits': None,
            'independent_view_hits': len(clusters),
            'correlated_repeat_hits': max(0, len(votes) - len(clusters)),
            'first_wall_frame': first_wall['frame_index'] if first_wall else None,
            'first_wall_stamp': first_wall['stamp'] if first_wall else None,
            'first_wall_elapsed_s': first_wall['elapsed_s'] if first_wall else None,
            'first_open_frame': None, 'first_open_stamp': None,
            'final_state': final_soft, 'effective_final_state': final_effective,
            'ever_wall': ever_wall,
            'wall_state_history': history,
            'view_clusters': clusters,
        })

    # The complete causal evidence accumulation and SENSOR state history above
    # are fixed before the user truth file is opened. Truth is joined only for
    # posthoc scoring; the separately declared perimeter contract is static.
    truth = edge_truth(json.loads(Path(truth_path).read_text()))
    if set(truth) != expected_edges:
        raise ValueError('truth must resolve exactly 112 canonical edges')
    true_final, true_effective, false_ever, false_final = [], [], [], []
    for row in rows:
        edge = row.pop('_edge_key')
        is_wall = truth[edge]
        row['truth'] = 'WALL' if is_wall else 'OPEN'
        row['ever_false_wall'] = bool(not is_wall and row['ever_wall'])
        row['final_false_wall'] = bool(not is_wall and row['final_state'] == WALL)
        row['effective_false_wall'] = bool(not is_wall and row['effective_final_state'] == WALL)
        if is_wall and row['final_state'] == WALL:
            true_final.append(edge)
        if is_wall and row['effective_final_state'] == WALL:
            true_effective.append(edge)
        if not is_wall and row['ever_wall']:
            false_ever.append(edge)
        if not is_wall and row['final_state'] == WALL:
            false_final.append(edge)
    summary = {
        'status': 'OFFLINE_POSTHOC_EDGE_ACCEPTANCE',
        'input_log': str(log_path), 'truth_path': str(truth_path),
        'frames': frame_count, 'canonical_edges': len(rows),
        'truth_counts': {'WALL': sum(truth.values()), 'OPEN': len(truth)-sum(truth.values())},
        'sensor_model': {
            'implementation': 'm3pro_nav.edge_map.EdgeMap exact SENSOR belief API',
            'constants': {'T_CONFIRM': 2, 'T_FLIP': 4},
            'per_frame_per_edge_vote_cap': 1,
            'open_hits': None,
            'open_votes_applied': 0,
            'open_evidence_available': False,
            'open_vote_reason': 'old /scan_multi point cloud has no reliable free-space evidence',
            'view_cluster_rule': {'position_m': VIEW_POSITION_M,
                                  'yaw_deg': math.degrees(VIEW_YAW_RAD)},
            'state_votes_include_correlated_frames': True,
            'perimeter_hard_seed': 'closed boundary WALL except declared S entrance and W exit; interior truth not seeded',
            'note': 'view counts are diagnostic; EdgeMap receives each frame hit, matching its current scan-level SENSOR behavior. With no OPEN evidence, OPEN transition and wall withdrawal cannot be accepted from this replay.'},
        'metrics': {
            'true_wall_finally_wall': len(true_final),
            'true_wall_recall': len(true_final) / sum(truth.values()) if sum(truth.values()) else None,
            'true_wall_final_soft': len(true_final),
            'true_wall_final_effective': len(true_effective),
            'true_wall_total': sum(truth.values()),
            'open_finally_wall': len(false_final), 'open_ever_wall': len(false_ever),
            'open_effectively_wall': sum(row['effective_false_wall'] for row in rows),
            'false_edges_final': [edge_text(e) for e in false_final],
            'false_edges_ever': [edge_text(e) for e in false_ever],
            'unknown_final_edges': sum(row['final_state'] == UNKNOWN for row in rows)},
        'edges': rows,
    }
    Path(json_path).write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    columns = ['edge_id', 'truth', 'wall_hits', 'open_hits', 'independent_view_hits',
               'correlated_repeat_hits', 'first_wall_frame', 'first_wall_stamp',
               'first_wall_elapsed_s', 'first_open_frame', 'first_open_stamp',
               'final_state', 'effective_final_state', 'ever_wall', 'ever_false_wall', 'final_false_wall',
               'effective_false_wall',
               'wall_state_history']
    with Path(csv_path).open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            csv_row = {key: row[key] for key in columns}
            csv_row['wall_state_history'] = json.dumps(row['wall_state_history'], ensure_ascii=False)
            writer.writerow(csv_row)
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log', type=Path, required=True)
    p.add_argument('--truth', type=Path, required=True)
    p.add_argument('--json', type=Path, default=Path('/tmp/edge-level-acceptance.json'))
    p.add_argument('--csv', type=Path, default=Path('/tmp/edge-level-acceptance.csv'))
    a = p.parse_args()
    print(json.dumps(replay(a.log, a.truth, a.json, a.csv)['metrics'], indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
