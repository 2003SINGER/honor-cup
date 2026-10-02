#!/usr/bin/env python3
"""Offline threshold study for the diagnostic robot-local WALL prior.

Consumes existing compressed post-snap frame logs; does not change pose
correction or map state. Truth is loaded only after per-frame prior values and
independent-view cluster credits have been frozen.
"""
from __future__ import annotations

import argparse
import ast
import json
import math
import statistics
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))
sys.path.insert(0, str(ROOT / 'software' / 'tools'))

from audit_lidar_odom_calibration import edge_truth
from m3pro_nav.wall_trust_prior import evaluate_wall_trust

CELL = .4
VIEW_POSITION_M = .20
VIEW_YAW_RAD = math.radians(20)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_of(frame):
    pose = frame.get('corrected_pose') or frame.get('pose')
    if isinstance(pose, dict):
        return [float(pose[k]) for k in ('x', 'y', 'theta')]
    return [float(v) for v in pose]


def read_frames(path):
    cmd = ['zstd', '-dc', str(path)] if str(path).endswith('.zst') else ['cat', str(path)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
    assert proc.stdout is not None
    frames = []
    for i, line in enumerate(proc.stdout):
        if not line.strip():
            continue
        frame = json.loads(line)
        frame['_frame_index'] = i
        frames.append(frame)
    code = proc.wait()
    if code:
        raise RuntimeError(f'log reader exited {code}: {path}')
    return frames


def _contiguous_support(hit, half_width_cm=5, gap_m=.06):
    tangent, normal = hit.get('roi_tangent_m'), hit.get('roi_signed_normal_m')
    if not isinstance(tangent, list) or not isinstance(normal, list):
        return 0, 0.
    values = sorted(float(t) for t, n in zip(tangent, normal)
                    if abs(float(n)) <= half_width_cm / 100.)
    if not values:
        return 0, 0.
    runs, run = [], [values[0]]
    for value in values[1:]:
        if value - run[-1] > gap_m:
            runs.append(run)
            run = []
        run.append(value)
    runs.append(run)
    best = max(runs, key=lambda r: (r[-1]-r[0], len(r)))
    return len(best), best[-1] - best[0]


def _eligible(hit):
    # Reuse the demonstrated narrow-ROI continuous-cluster candidate as the
    # spatial evidence floor; the prior only weights evidence above this floor.
    count, span = _contiguous_support(hit)
    return count >= 6 and span >= .18


def collect_view_credits(frames):
    votes = defaultdict(list)
    for frame in frames:
        pose = pose_of(frame)
        for hit in frame.get('wall_edge_hits', []):
            if not _eligible(hit):
                continue
            edge = ast.literal_eval(hit['edge_id'])
            rate = hit.get('odom_yaw_rate_rad_s', frame.get('odom_yaw_rate_rad_s'))
            prior = evaluate_wall_trust(edge, hit, pose, cell_size=CELL,
                                        yaw_rate_rad_s=rate)
            votes[edge].append({'frame': frame['_frame_index'], 'pose': pose,
                                'weight': prior.promotion_weight,
                                'prior': prior})

    # One maximum-weight representative per independent viewpoint cluster.
    # Repeated stationary frames can improve persistence diagnostics but cannot
    # mint extra credits in this P3-only benchmark.
    credits = {}
    for edge, samples in votes.items():
        reps = []
        for sample in samples:
            for rep in reps:
                if (math.dist(sample['pose'][:2], rep['pose'][:2]) <= VIEW_POSITION_M and
                        abs(wrap(sample['pose'][2]-rep['pose'][2])) <= VIEW_YAW_RAD):
                    if sample['weight'] > rep['weight']:
                        rep.update(sample)
                    break
            else:
                reps.append(dict(sample))
        credits[edge] = reps
    return credits


def summarize(credits, truth, thresholds=(0., 1., 1.25, 1.5, 1.75, 2., 2.5)):
    results = []
    n_walls = sum(truth.values())
    n_open = len(truth) - n_walls
    for threshold in thresholds:
        confirmed = set()
        for edge in truth:
            reps = credits.get(edge, [])
            # Near and measured-medium evidence can use one independent view;
            # far evidence must come from two distinct clusters.  Two or more
            # consecutive frames inside one viewpoint never increase this count.
            all_far = bool(reps) and all(
                r['prior'].side_visibility_weight <= .55 or
                r['prior'].range_weight < .55 for r in reps)
            required_views = 2 if all_far else 1
            if (len(reps) >= required_views and
                    sum(r['weight'] for r in reps) >= threshold):
                confirmed.add(edge)
        true_confirmed = sum(bool(truth[e]) for e in confirmed)
        false_edges = sorted((e for e in confirmed if not truth[e]), key=repr)
        results.append({
            'credit_threshold': threshold,
            'required_independent_views': '2 for far; 1 otherwise',
            'true_wall_confirmed': true_confirmed,
            'true_wall_total': n_walls,
            'true_wall_recall': true_confirmed / n_walls if n_walls else None,
            'false_open_confirmed': len(false_edges),
            'false_open_edge_ids': [repr(e) for e in false_edges],
            'missed_true_edge_ids': [repr(e) for e in truth if truth[e] and e not in confirmed],
        })
    return results


def run(log, truth_path):
    frames = read_frames(log)
    credits = collect_view_credits(frames)
    truth = edge_truth(json.loads(Path(truth_path).read_text()))
    thresholds = summarize(credits, truth)
    all_reps = [rep for reps in credits.values() for rep in reps]
    diagnostics = {
        'eligible_edges': len(credits),
        'independent_view_credits': len(all_reps),
        'mean_promotion_weight': (statistics.mean(r['weight'] for r in all_reps)
                                  if all_reps else None),
        'wall_side_credits': sum(r['prior'].wall_side == 'side' for r in all_reps),
        'front_wall_credits': sum(r['prior'].wall_side == 'front' for r in all_reps),
        'turning_credits': sum(r['prior'].turn_promotion_weight < .999 for r in all_reps),
    }
    return {'status': 'DIAGNOSTIC_ONLY_POST_SNAP_PRIOR_SWEEP',
            'input_log': str(log), 'frames': len(frames), 'truth_path': str(truth_path),
            'pose_correction_weight': 1.0,
            'spatial_floor': {'source': 'existing contiguous ROI candidate',
                              'normal_half_width_cm': 5, 'min_cluster_points': 6,
                              'min_contiguous_span_m': .18},
            'view_cluster': {'position_m': VIEW_POSITION_M,
                             'yaw_rad': VIEW_YAW_RAD,
                             'credits_per_cluster': 1},
            'diagnostics': diagnostics,
            'thresholds': thresholds}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--truth', required=True, type=Path)
    parser.add_argument('--json', type=Path)
    args = parser.parse_args()
    result = run(args.log, args.truth)
    rendered = json.dumps(result, indent=2, ensure_ascii=False) + '\n'
    if args.json:
        args.json.write_text(rendered)
    print(rendered, end='')


if __name__ == '__main__':
    main()
