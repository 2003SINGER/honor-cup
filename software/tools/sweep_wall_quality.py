#!/usr/bin/env python3
"""Offline sweep of WALL-evidence quality gates over frame-grid-snap JSONL.

All candidate hit filtering and EdgeMap evolution happens before truth is read.
Truth is joined only for final topology metrics. This tool does not alter pose
correction or the source replay log.
"""
from __future__ import annotations

import argparse
import ast
import csv
import itertools
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))
sys.path.insert(0, str(ROOT / 'software' / 'tools'))

from m3pro_nav.edge_map import EdgeMap, WALL, UNKNOWN
from m3pro_nav.grid_association import edge_id_for_line
from m3pro_nav.wall_evidence_quality import add_temporal_evidence
from audit_lidar_odom_calibration import edge_truth

N = 7
VIEW_POSITION_M = 0.20
VIEW_YAW_RAD = math.radians(20)
ROI_CM = (2, 3, 4, 5, 8, 10, 12)
DISTANCE_LIMITS_M = (0.25, 0.40, 0.60, 1.20, 2.0, None)
GRAZING_LIMITS_DEG = (None, 10, 20, 35, 60, 85)
YAW_RATE_LIMITS = (0.03, 0.08, math.inf)
VIEW_CONFIRMATIONS = (1, 2, 3)
CONTIGUOUS_GAP_M = .06


def canonical_edges():
    return {edge_id_for_line(o, k, j)
            for o in ('H', 'V') for k in range(N + 1) for j in range(N)}


def edge_text(edge):
    if edge[0] == 'B':
        _, (x, y), d = edge
        return f'B({x},{y},{d})'
    cell, d = edge
    return f'{d}({cell[0]},{cell[1]})'


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _roi_measure(hit, cm):
    """Read the producer's ROI sweep, tolerating dict/list representations."""
    roi = hit.get('roi')
    if isinstance(roi, dict):
        item = roi.get(str(cm), roi.get(f'{cm}cm'))
        if isinstance(item, dict):
            return item.get('support_points'), item.get('span_m')
    for key in ('roi_support_points_by_half_width_cm', 'roi_counts_by_half_width_cm',
                'roi_support_points', 'roi_counts'):
        value = hit.get(key)
        if isinstance(value, dict):
            for k in (str(cm), f'{cm}cm', f'{cm:02d}'):
                if k in value:
                    item = value[k]
                    if isinstance(item, dict):
                        return item.get('support_points', item.get('count')), item.get('span_m')
                    return item, None
        elif isinstance(value, list) and len(value) == len(ROI_CM):
            item = value[ROI_CM.index(cm)]
            if isinstance(item, dict):
                return item.get('support_points', item.get('count')), item.get('span_m')
            return item, None
    # Wider posthoc bands are reconstructed from the finite endpoints retained
    # by the producer (already clipped to its declared maximum ROI and edge span).
    tangent = hit.get('roi_tangent_m')
    normal = hit.get('roi_signed_normal_m')
    if isinstance(tangent, list) and isinstance(normal, list):
        interval = hit.get('roi_tangential_interval_m')
        pairs = [(float(t), float(n)) for t, n in zip(tangent, normal)
                 if interval is None or float(interval[0]) <= float(t) <= float(interval[1])]
        selected = [t for t, n in pairs if abs(n) <= cm / 100.0]
        return len(selected), (max(selected) - min(selected) if len(selected) >= 2 else 0.0)
    # Do not reuse the old full hit support as if it were a measurement at every
    # requested ROI width. The fixed baseline reads those fields explicitly.
    return None, None


def _roi_contiguous_measure(hit, cm):
    """Largest supported run inside one edge, without bridging empty space."""
    item = (hit.get('roi') or {}).get(str(cm))
    if isinstance(item, dict) and 'longest_contiguous_span_m' in item:
        return (item.get('longest_contiguous_support_points'),
                item['longest_contiguous_span_m'])
    tangent = hit.get('roi_tangent_m')
    normal = hit.get('roi_signed_normal_m')
    if not isinstance(tangent, list) or not isinstance(normal, list):
        return None, None
    points = sorted(float(t) for t, n in zip(tangent, normal)
                    if abs(float(n)) <= cm / 100.)
    if not points:
        return 0, 0.
    runs = []
    run = [points[0]]
    for point in points[1:]:
        if point - run[-1] > CONTIGUOUS_GAP_M:
            runs.append(run)
            run = []
        run.append(point)
    runs.append(run)
    best = max(runs, key=lambda r: (r[-1] - r[0], len(r)))
    return len(best), best[-1] - best[0]


def _get_heading_error(hit):
    for k in ('heading_to_wall_normal_rad', 'heading_wall_normal_rad',
              'heading_to_normal_rad', 'grazing_angle_rad'):
        if k in hit and hit[k] is not None:
            return abs(float(hit[k]))
    return None


def _mad(hit):
    for k in ('normal_mad_m', 'normal_offset_mad_m', 'normal_residual_mad', 'normal_mad'):
        if k in hit and hit[k] is not None:
            return float(hit[k])
    return None


def _yaw_rate(frame, hit):
    for obj in (hit, frame):
        for k in ('odom_yaw_rate_rad_s', 'odom_yaw_rate_radps', 'yaw_rate_rad_s', 'odom_yaw_rate'):
            if obj.get(k) is not None:
                return abs(float(obj[k]))
    return None


def _pose(frame):
    p = frame.get('corrected_pose') or frame.get('pose')
    if isinstance(p, dict):
        return [float(p[k]) for k in ('x', 'y', 'theta')]
    return [float(v) for v in p]


def _view_clusters(samples, position_m=VIEW_POSITION_M, yaw_rad=VIEW_YAW_RAD):
    reps = []
    for sample in samples:
        pose = sample['pose']
        for rep in reps:
            rp = rep['pose']
            if (math.dist(pose[:2], rp[:2]) <= position_m and
                    abs(wrap(pose[2] - rp[2])) <= yaw_rad):
                rep['frames'].append(sample['frame_index'])
                break
        else:
            reps.append({'pose': pose, 'frames': [sample['frame_index']]})
    return reps


def _load_frames(log_path):
    frames = []
    for index, line in enumerate(Path(log_path).read_text().splitlines()):
        if line.strip():
            f = json.loads(line)
            if 'wall_edge_hits' not in f:
                raise ValueError('JSONL missing wall_edge_hits; regenerate frame-grid-snap log')
            f['_frame_index'] = index
            frames.append(f)
    return frames


def _dedupe_frame_hits(frames):
    by_edge = defaultdict(list)
    for frame in frames:
        current = defaultdict(list)
        for hit in frame['wall_edge_hits']:
            edge = ast.literal_eval(hit['edge_id'])
            current[edge].append(hit)
        for edge, hits in current.items():
            chosen = max(hits, key=lambda h: h.get('support_points', 0))
            by_edge[edge].append({
                'frame_index': frame['_frame_index'], 'stamp': frame['stamp'],
                'elapsed_s': frame.get('elapsed_s'), 'pose': _pose(frame),
                'hit': chosen, 'yaw_rate': _yaw_rate(frame, chosen),
                'duplicate_count': len(hits),
            })
    return by_edge


def _passes(sample, config):
    if sample.get('edge_id_jump_abstain'):
        return False
    if sample.get('temporal_streak', 1) < config.get('min_temporal_streak', 0):
        return False
    hit = sample['hit']
    if config.get('use_roi_support', True):
        count, span = _roi_measure(hit, config['roi_half_width_cm'])
    else:
        count, span = hit.get('support_points'), hit.get('span_m')
    # Counts/spans absent from older logs cannot establish support for a larger
    # ROI; the emitted basic hit remains eligible only for the fixed baseline.
    if count is None or float(count) < config['min_support_points']:
        return False
    if span is None or float(span) < config['min_span_m']:
        return False
    if config.get('min_contiguous_span_m') is not None:
        run_count, run_span = _roi_contiguous_measure(
            hit, config['contiguous_roi_half_width_cm'])
        if (run_count is None or
                run_count < config.get('min_contiguous_support_points', 6) or
                run_span < config['min_contiguous_span_m']):
            return False
    dist = hit.get('distance_m')
    if dist is None or (config['max_distance_m'] is not None and
                        float(dist) > config['max_distance_m']):
        return False
    angle = _get_heading_error(hit)
    if config['max_grazing_deg'] is not None:
        if angle is None or angle > math.radians(config['max_grazing_deg']):
            return False
    mad = _mad(hit)
    if config['max_normal_mad_m'] is not None:
        if mad is None or mad > config['max_normal_mad_m']:
            return False
    yr = sample['yaw_rate']
    if config['max_yaw_rate_rad_s'] is not None:
        if yr is None or yr > config['max_yaw_rate_rad_s']:
            return False
    return True


def _simulate(edge_votes, config):
    """Apply quality filter, per-frame cap, and EdgeMap; returns pre-truth state."""
    if not all(samples and 'temporal_streak' in samples[0]
               for samples in edge_votes.values()):
        for edge, samples in edge_votes.items():
            for sample in samples:
                sample['edge'] = edge
        add_temporal_evidence(edge_votes)
    em = EdgeMap(N)
    all_edges = canonical_edges()
    for edge in all_edges:
        if edge[0] != 'B':
            continue
        _, cell, direction = edge
        portal = (cell, direction) in {((3, 0), 'S'), ((0, 0), 'W')}
        em.set_boundary(cell, direction, 'OPEN' if portal else WALL)
    result = {}
    view_position_m = config.get('view_position_m', VIEW_POSITION_M)
    view_yaw_rad = config.get('view_yaw_rad', VIEW_YAW_RAD)
    for edge in all_edges:
        eligible = [v for v in edge_votes.get(edge, []) if _passes(v, config)]
        required_views = config['independent_view_confirmations']
        if edge[0] == 'B':
            c, d = edge[1], edge[2]
        else:
            c, d = edge
        history = []
        first_wall = None
        def apply_vote(v, confirmation_sample=None):
            nonlocal first_wall
            before = em.soft.get(edge, {'state': UNKNOWN})['state']
            em.observe_wall(c, d, dist=float(v['hit']['distance_m']), stamp=v['stamp'])
            after = em.soft.get(edge, {'state': UNKNOWN})['state']
            if after != before:
                when = confirmation_sample or v
                event = {'frame_index': when['frame_index'], 'stamp': when['stamp'],
                         'elapsed_s': when['elapsed_s'], 'from': before, 'to': after,
                         'score': em.soft[edge]['score']}
                history.append(event)
                if after == WALL and first_wall is None:
                    first_wall = event
        # Process observations in time order. A gated policy buffers distinct
        # views; only the frame that reaches the threshold releases the buffer.
        # Later distinct views then contribute one ordinary vote each.
        view_reps, buffered = [], []
        armed = (required_views == 0)
        for v in eligible:
            if required_views == 0:
                apply_vote(v)
                continue
            if any(math.dist(v['pose'][:2], rep['pose'][:2]) <= view_position_m and
                   abs(wrap(v['pose'][2] - rep['pose'][2])) <= view_yaw_rad
                   for rep in view_reps):
                continue
            view_reps.append({'pose': v['pose']})
            if not armed:
                buffered.append(v)
                if len(buffered) < required_views:
                    continue
                armed = True
                for pending in buffered:
                    apply_vote(pending, confirmation_sample=v)
                buffered.clear()
            else:
                apply_vote(v)
        # Distinct view count remains diagnostic even when the gate never arms.
        views = [{'pose': r['pose']} for r in view_reps]
        votes_applied = (len(eligible) if required_views == 0 else
                         (len(view_reps) if armed else 0))
        # Keep the measured physical offset as continuous geometry. A stable
        # offset alone does not establish that the selected discrete ID is
        # correct; false neighboring edges may also have coherent offsets.
        offsets = [float(v['hit']['normal_residual_m']) for v in eligible
                   if v['hit'].get('normal_residual_m') is not None]
        offset_median = statistics.median(offsets) if offsets else None
        result[edge] = {
            'wall_hits': votes_applied, 'quality_hits': len(eligible),
            'independent_views': len(views),
            'max_temporal_streak': max((v.get('temporal_streak', 1)
                                        for v in edge_votes.get(edge, [])), default=0),
            'temporal_continuity_hits': sum(bool(v.get('temporal_continuity'))
                                             for v in edge_votes.get(edge, [])),
            'edge_id_jump_abstentions': sum(bool(v.get('edge_id_jump_abstain'))
                                             for v in edge_votes.get(edge, [])),
            'final_state': em.soft.get(edge, {'state': UNKNOWN})['state'],
            'effective_final_state': em.state(c, d),
            'ever_wall': first_wall is not None,
            'first_wall': first_wall, 'history': history,
            'physical_normal_offset_median_m': offset_median,
            'physical_normal_offset_mad_m': (
                statistics.median(abs(x - offset_median) for x in offsets)
                if offsets else None),
        }
    return result


def _configurations():
    """Staged sweep: full ROI×distance×heading grid, then temporal/MAD ablation."""
    seen = set()
    def add(cfg):
        key = json.dumps(cfg, sort_keys=True)
        if key not in seen:
            seen.add(key)
            return cfg
        return None

    # Core spatial quality grid keeps other gates open, so wide ROIs and side
    # views can show their actual recall/false-wall tradeoff.
    for roi, dist, grazing in itertools.product(
            ROI_CM, DISTANCE_LIMITS_M, GRAZING_LIMITS_DEG):
        cfg = {'roi_half_width_cm': roi, 'use_roi_support': True,
               'min_support_points': 6, 'min_span_m': .08,
               'max_distance_m': dist, 'max_grazing_deg': grazing,
               'max_yaw_rate_rad_s': None,
               'independent_view_confirmations': 1,
               'max_normal_mad_m': None}
        unique = add(cfg)
        if unique is not None:
            yield unique

    # Temporal correlation and residual-shape gates are swept at a deliberately
    # permissive spatial setting to avoid multiplying a redundant huge grid.
    for yaw, views, mad in itertools.product(
            (0.03, 0.08, None), VIEW_CONFIRMATIONS, (None, 0.01, 0.02, 0.04)):
        cfg = {'roi_half_width_cm': 12, 'use_roi_support': True,
               'min_support_points': 6, 'min_span_m': .08,
               'max_distance_m': None, 'max_grazing_deg': None,
               'max_yaw_rate_rad_s': yaw,
               'independent_view_confirmations': views,
               'max_normal_mad_m': mad, 'min_temporal_streak': 0}
        unique = add(cfg)
        if unique is not None:
            yield unique


def _fixed_baseline_config():
    return {'roi_half_width_cm': None, 'use_roi_support': False,
            'min_support_points': 6, 'min_span_m': .08,
            'max_distance_m': None, 'max_grazing_deg': None,
            'max_yaw_rate_rad_s': None, 'independent_view_confirmations': 0,
            'max_normal_mad_m': None, 'min_temporal_streak': 0}


def _span_view_configurations():
    """Targeted sweep on original hit support, without geometry proxy gates."""
    for span, views in itertools.product(
            (.08, .10, .12, .15, .18, .20, .25, .30), (0, 2, 3, 4, 5, 6)):
        yield {'roi_half_width_cm': None, 'use_roi_support': False,
               'min_support_points': 6, 'min_span_m': span,
               'max_distance_m': None, 'max_grazing_deg': None,
               'max_yaw_rate_rad_s': None,
               'independent_view_confirmations': views,
               'max_normal_mad_m': None, 'min_temporal_streak': 0}


def _contiguous_roi_configurations():
    """Test direct endpoint rectangles separately from whole-fit span."""
    for cm, span, views in itertools.product(
            (2, 3, 5), (.08, .12, .15, .18), (0, 2)):
        yield {**_fixed_baseline_config(),
               'contiguous_roi_half_width_cm': cm,
               'min_contiguous_support_points': 6,
               'min_contiguous_span_m': span,
               'independent_view_confirmations': views,
               'min_temporal_streak': 0}


def _temporal_configurations():
    """Persistence sweep; independent viewpoints remain a separate gate."""
    for streak, view_position_m, view_yaw_deg in itertools.product(
            (2, 3), (.05, .10, .20), (5, 10, 20)):
        yield {'roi_half_width_cm': 2, 'use_roi_support': True,
               'min_support_points': 6, 'min_span_m': .08,
               'max_distance_m': None, 'max_grazing_deg': None,
               'max_yaw_rate_rad_s': None,
               'independent_view_confirmations': 1,
               'view_position_m': view_position_m,
               'view_yaw_rad': math.radians(view_yaw_deg),
               'max_normal_mad_m': None,
               'min_temporal_streak': streak}


def _score(simulated, truth):
    walls = [e for e, value in truth.items() if value]
    opens = [e for e, value in truth.items() if not value]
    confirmed = [e for e in walls if simulated[e]['final_state'] == WALL]
    open_final = [e for e in opens if simulated[e]['final_state'] == WALL]
    open_ever = [e for e in opens if simulated[e]['ever_wall']]
    effective = [e for e in opens if simulated[e]['effective_final_state'] == WALL]
    return {
        'true_wall_confirmed': len(confirmed), 'true_wall_total': len(walls),
        'true_wall_recall': len(confirmed) / len(walls) if walls else None,
        'open_final_soft_wall': len(open_final), 'open_ever_soft_wall': len(open_ever),
        'open_final_effective_wall': len(effective),
        'missed_true_wall_edges': [edge_text(e) for e in walls if e not in confirmed],
        'false_final_edges': [edge_text(e) for e in open_final],
        'false_ever_edges': [edge_text(e) for e in open_ever],
        'edges': {edge_text(e): {
                    'wall_hits': simulated[e]['wall_hits'],
                    'quality_hits': simulated[e]['quality_hits'],
                    'independent_views': simulated[e]['independent_views'],
                    'max_temporal_streak': simulated[e]['max_temporal_streak'],
                    'temporal_continuity_hits': simulated[e]['temporal_continuity_hits'],
                    'edge_id_jump_abstentions': simulated[e]['edge_id_jump_abstentions'],
                    'final_state': simulated[e]['final_state'],
                    'effective_final_state': simulated[e]['effective_final_state'],
                    'ever_wall': simulated[e]['ever_wall'],
                    'first_wall_frame': (simulated[e]['first_wall'] or {}).get('frame_index'),
                    'first_wall_stamp': (simulated[e]['first_wall'] or {}).get('stamp'),
                    'first_wall_elapsed_s': (simulated[e]['first_wall'] or {}).get('elapsed_s'),
                    'wall_state_history': simulated[e]['history'],
                    'physical_normal_offset_median_m': simulated[e]['physical_normal_offset_median_m'],
                    'physical_normal_offset_mad_m': simulated[e]['physical_normal_offset_mad_m'],
                  } for e in sorted(simulated, key=edge_text)},
    }


def _dominates(a, b):
    # Higher recall and lower false ever/final are preferred; at least one strict.
    av = (a['metrics']['true_wall_recall'], -a['metrics']['open_ever_soft_wall'],
          -a['metrics']['open_final_soft_wall'])
    bv = (b['metrics']['true_wall_recall'], -b['metrics']['open_ever_soft_wall'],
          -b['metrics']['open_final_soft_wall'])
    return all(x >= y for x, y in zip(av, bv)) and any(x > y for x, y in zip(av, bv))


def sweep(log_path, truth_path, json_path, csv_path):
    frames = _load_frames(log_path)
    edge_votes = _dedupe_frame_hits(frames)
    configurations = list(_configurations())
    candidates = []
    for cfg in configurations:
        # Keep truth out of every filter and state transition. It is loaded below
        # only after all candidate EdgeMap histories have been frozen.
        candidates.append({'config': cfg, 'simulation': _simulate(edge_votes, cfg)})
    baseline_cfg = _fixed_baseline_config()
    baseline = _simulate(edge_votes, baseline_cfg)
    span_view_candidates = [
        {'config': cfg, 'simulation': _simulate(edge_votes, cfg)}
        for cfg in _span_view_configurations()]
    contiguous_candidates = [
        {'config': cfg, 'simulation': _simulate(edge_votes, cfg)}
        for cfg in _contiguous_roi_configurations()]
    temporal_candidates = [
        {'config': cfg, 'simulation': _simulate(edge_votes, cfg)}
        for cfg in _temporal_configurations()]
    truth = edge_truth(json.loads(Path(truth_path).read_text()))
    if set(truth) != canonical_edges():
        raise ValueError('truth must resolve exactly 112 canonical edges')
    for item in candidates:
        item['metrics'] = _score(item.pop('simulation'), truth)
    baseline_scored = _score(baseline, truth)
    span_view_results = []
    for item in span_view_candidates:
        span_view_results.append({'config': item['config'],
                                  'metrics': _score(item['simulation'], truth)})
    contiguous_results = [
        {'config': item['config'], 'metrics': _score(item['simulation'], truth)}
        for item in contiguous_candidates]
    temporal_results = [
        {'config': item['config'], 'metrics': _score(item['simulation'], truth)}
        for item in temporal_candidates]
    pareto = [item for item in candidates
              if not any(_dominates(other, item) for other in candidates)]
    # Keep the Pareto records sorted in a useful recall/false-wall order.
    pareto.sort(key=lambda x: (-x['metrics']['true_wall_recall'],
                               x['metrics']['open_ever_soft_wall'],
                               x['metrics']['open_final_soft_wall']))
    # Detailed per-edge histories are retained for the baseline and Pareto
    # tradeoffs; the full grid stays compact while preserving every aggregate.
    pareto_ids = {id(item) for item in pareto}
    all_candidate_summaries = []
    for item in candidates:
        if id(item) not in pareto_ids:
            item['metrics'].pop('edges', None)
        all_candidate_summaries.append(item)
    supported = {key: sum(1 for edge in edge_votes.values() for v in edge
                          if ((key == 'roi_metrics' and any(_roi_measure(v['hit'], cm)[0] is not None for cm in ROI_CM)) or
                              (key == 'normal_mad' and _mad(v['hit']) is not None) or
                              (key == 'heading_proxy' and _get_heading_error(v['hit']) is not None) or
                              (key == 'yaw_rate' and v['yaw_rate'] is not None)))
                 for key in ('roi_metrics', 'normal_mad', 'heading_proxy', 'yaw_rate')}
    output = {'status': 'OFFLINE_POSTHOC_WALL_QUALITY_SWEEP',
              'input_log': str(log_path), 'truth_path': str(truth_path),
              'frames': len(frames), 'canonical_edges': len(canonical_edges()),
              'truth_counts': {'WALL': sum(truth.values()), 'OPEN': len(truth)-sum(truth.values())},
              'evidence_field_coverage': supported,
              'method': {'pose_correction_changed': False, 'truth_used_online': False,
                         'edge_map': 'm3pro_nav.edge_map.EdgeMap',
                         'vote_policy': 'one quality-selected hit per edge/frame; independent view clusters only cast votes',
                         'temporal_evidence': {
                             'module': 'm3pro_nav.wall_evidence_quality',
                             'matching': 'same canonical edge in adjacent logged frames, <=0.25 s, overlapping post-snap endpoint support, stable world normal position',
                             'edge_id_jump': 'abstain only if the prior ID disappears and a neighboring tangent-cell ID has matching physical endpoints under a small corrected-pose delta',
                             'temporal_persistence_and_independent_view_credit_are_separate': True},
                         'heading_metric': 'endpoint geometry proxy; not per-beam incidence angle',
                         'view_cluster': {'position_m': VIEW_POSITION_M, 'yaw_deg': math.degrees(VIEW_YAW_RAD)},
                         'quality_sweep_configurations': len(configurations),
                         'targeted_span_view_configurations': len(span_view_results),
                         'contiguous_roi_configurations': len(contiguous_results),
                         'temporal_configurations': len(temporal_results),
                         'contiguous_gap_m': CONTIGUOUS_GAP_M,
                         'targeted_span_view_policy': 'original per-edge support_points/span_m; no distance, heading, yaw-rate, ROI, or MAD gate',
                         'comparison_note': 'The fixed baseline uses existing per-cell >=6 points and >=8 cm producer support, all distances/angles/rates, and EdgeMap frame voting.'},
              'fixed_baseline': {'config': baseline_cfg, 'metrics': baseline_scored},
              'targeted_span_view_sweep': span_view_results,
              'contiguous_roi_sweep': contiguous_results,
              'temporal_sweep': temporal_results,
              'pareto': pareto, 'all_candidates': all_candidate_summaries}
    Path(json_path).write_text(json.dumps(output, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    cols = ['sweep_kind', 'min_span_m', 'roi_half_width_cm',
            'contiguous_roi_half_width_cm', 'min_contiguous_support_points',
            'min_contiguous_span_m', 'max_distance_m', 'max_grazing_deg',
            'max_yaw_rate_rad_s', 'independent_view_confirmations',
            'max_normal_mad_m', 'min_temporal_streak',
            'view_position_m', 'view_yaw_rad',
            'true_wall_confirmed', 'true_wall_total',
            'true_wall_recall', 'open_final_soft_wall', 'open_ever_soft_wall',
            'open_final_effective_wall', 'missed_true_wall_edges', 'false_final_edges', 'false_ever_edges']
    with Path(csv_path).open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=cols, lineterminator='\n')
        writer.writeheader()
        for item in candidates:
            row = {'sweep_kind': 'core', **item['config'], **item['metrics']}
            for key in ('missed_true_wall_edges', 'false_final_edges', 'false_ever_edges'):
                row[key] = json.dumps(row[key], ensure_ascii=False)
            writer.writerow({k: row.get(k) for k in cols})
        for item in span_view_results:
            row = {'sweep_kind': 'targeted_span_view', **item['config'], **item['metrics']}
            for key in ('missed_true_wall_edges', 'false_final_edges', 'false_ever_edges'):
                row[key] = json.dumps(row[key], ensure_ascii=False)
            writer.writerow({k: row.get(k) for k in cols})
        for item in contiguous_results:
            row = {'sweep_kind': 'contiguous_roi', **item['config'], **item['metrics']}
            for key in ('missed_true_wall_edges', 'false_final_edges', 'false_ever_edges'):
                row[key] = json.dumps(row[key], ensure_ascii=False)
            writer.writerow({k: row.get(k) for k in cols})
        for item in temporal_results:
            row = {'sweep_kind': 'temporal', **item['config'], **item['metrics']}
            for key in ('missed_true_wall_edges', 'false_final_edges', 'false_ever_edges'):
                row[key] = json.dumps(row[key], ensure_ascii=False)
            writer.writerow({k: row.get(k) for k in cols})
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--truth', required=True, type=Path)
    parser.add_argument('--json', type=Path, default=Path('/tmp/wall-quality-sweep.json'))
    parser.add_argument('--csv', type=Path, default=Path('/tmp/wall-quality-sweep.csv'))
    args = parser.parse_args()
    result = sweep(args.log, args.truth, args.json, args.csv)
    print(json.dumps({'frames': result['frames'], 'field_coverage': result['evidence_field_coverage'],
                      'baseline': result['fixed_baseline']['metrics'] | {'config': result['fixed_baseline']['config']},
                      'pareto_count': len(result['pareto']),
                      'pareto': [{'config': x['config'], 'metrics': {k: v for k, v in x['metrics'].items() if k != 'edges'}} for x in result['pareto']]},
                     indent=2, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
