#!/usr/bin/env python3
"""Causal offline replay: corrected odometry plus continuous geometry per edge.

This is an experiment, not an online mapper.  Edge IDs stay discrete while
the normal coordinate of a wall segment is estimated in metres.  Every scan
is first predicted from the previous corrected pose and the wheel-odometry
increment.  Only geometry promoted by earlier scans can correct the current
pose.  Current-scan geometry observations are incorporated after correction.

The supplied topology is loaded only after replay, for optional scoring.  This
prototype uses merged /scan_multi hit endpoints only; it cannot infer OPEN or
deskew individual beams from the available bag.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))

import audit_lidar_odom_calibration as audit
from analyze_scan_edge import (CELL_SIZE_M, HEADING_RAD, decode_laser_scan,
    decode_odometry, decode_tf_message, static_transform)
from m3pro_nav.grid_association import GridAssociation, UNIQUE
from m3pro_nav.pose import Pose2D
from m3pro_nav.pose_correction import (KnownWallSegment,
    PoseCorrectionConfig, ProjectedEndpoint, propose_pose_correction)

DEFAULT_SESSION = ROOT / 'field_data' / '20261001_174719_joystick_full_maze'
DEFAULT_LOG = Path('/tmp/honor-cup-continuous-wall-geometry.jsonl')
DEFAULT_SUMMARY = Path('/tmp/honor-cup-continuous-wall-geometry-summary.json')
CELL = CELL_SIZE_M
MAX_ODOM_BRACKET_S = .15


def interpolate_bounded_pose(samples, stamp: float,
                             max_gap_s: float = MAX_ODOM_BRACKET_S) -> Pose2D:
    """Interpolate only inside an odometry bracket no wider than max_gap_s."""
    times = [sample[0] for sample in samples]
    i = bisect.bisect_left(times, stamp)
    if i < len(samples) and times[i] == stamp:
        return samples[i][1]
    if i == 0 or i == len(samples):
        raise ValueError('scan stamp has no odometry bracket')
    ta, a = samples[i - 1]
    tb, b = samples[i]
    if tb - ta > max_gap_s:
        raise ValueError(f'odometry bracket gap {tb-ta:.6f}s exceeds {max_gap_s:.3f}s')
    fraction = (stamp - ta) / (tb - ta)
    dyaw = math.atan2(math.sin(b.yaw - a.yaw), math.cos(b.yaw - a.yaw))
    return Pose2D(a.x + fraction * (b.x - a.x),
                  a.y + fraction * (b.y - a.y),
                  a.yaw + fraction * dyaw)


def _tf(p):
    return audit.Transform2D(p.x, p.y, p.yaw)


def _pose(t):
    return Pose2D(t.x, t.y, t.yaw)


def _edge_geometry(edge_id):
    """Return orientation, ideal normal coordinate and segment bounds."""
    if edge_id[0] == 'B':
        _, (i, j), d = edge_id
    else:
        (i, j), d = edge_id
    if d == 'E':
        return 'V', (i + 1) * CELL, j * CELL, (j + 1) * CELL
    if d == 'W':
        return 'V', i * CELL, j * CELL, (j + 1) * CELL
    if d == 'N':
        return 'H', (j + 1) * CELL, i * CELL, (i + 1) * CELL
    if d == 'S':
        return 'H', j * CELL, i * CELL, (i + 1) * CELL
    raise ValueError(f'unknown edge direction: {d}')


def _wall_segment(edge_id, coordinate):
    orient, _ideal, lo, hi = _edge_geometry(edge_id)
    if orient == 'V':
        return KnownWallSegment((coordinate, lo), (coordinate, hi), True,
                                repr(edge_id))
    return KnownWallSegment((lo, coordinate), (hi, coordinate), True,
                            repr(edge_id))


def _view_distance(a, b):
    dist = math.hypot(a[0] - b[0], a[1] - b[1])
    dyaw = abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))
    return dist, dyaw


@dataclass
class EdgeGeometry:
    edge_id: tuple
    orientation: str
    coordinate: float
    views: list[list[float]] = field(default_factory=list)
    stable: bool = False
    conflicts: int = 0

    def observe(self, coordinate: float, pose: Pose2D, stamp: float):
        view = [pose.x, pose.y, pose.yaw, stamp, coordinate, 1.0]
        nearby = next((v for v in self.views
                       if _view_distance(v[:3], view[:3])[0] < .15 and
                       _view_distance(v[:3], view[:3])[1] < math.radians(15)),
                      None)
        is_new_view = nearby is None
        if nearby is None:
            self.views.append(view)
        else:
            n = nearby[5]
            nearby[4] = (nearby[4] * n + coordinate) / (n + 1)
            nearby[3] = max(nearby[3], stamp)
            nearby[5] = n + 1

        separated_views = [v for v in self.views if all(
            abs(v[3] - other[3]) >= 1.0 and
            (_view_distance(v[:3], other[:3])[0] >= .15 or
             _view_distance(v[:3], other[:3])[1] >= math.radians(15))
            for other in self.views if other is not v)]
        if not self.stable and len(separated_views) >= 4:
            samples = [v[4] for v in separated_views]
            med = sorted(samples)[len(samples) // 2]
            spread = sorted(abs(x - med) for x in samples)[len(samples) // 2]
            if spread <= .012 and max(samples) - min(samples) <= .04:
                self.coordinate = med
                self.stable = True
        elif self.stable and is_new_view:
            # Freeze geometry after promotion.  A slow update would absorb
            # wheel-odometry drift into the map; disagreements are evidence
            # for a future joint estimator, not permission to move this wall.
            if abs(coordinate - self.coordinate) > .025:
                self.conflicts += 1


def _frame_line_measurements(rays, pose):
    """One robust line-coordinate observation per edge per scan."""
    assoc = GridAssociation(max_residual=.10)
    groups = defaultdict(list)
    for observation in assoc.process(rays):
        if observation.outcome != UNIQUE or observation.candidate is None:
            continue
        c = observation.candidate
        # Require hit orientation to agree with the candidate line direction.
        tangential = c.hit_y if c.orientation == 'V' else c.hit_x
        groups[c.edge_id].append((c.hit_x if c.orientation == 'V' else c.hit_y,
                                  tangential))
    out = {}
    for edge_id, pts in groups.items():
        orient, _ideal, _lo, _hi = _edge_geometry(edge_id)
        if len(pts) < 4:
            continue
        coords = sorted(p[0] for p in pts)
        med = coords[len(coords) // 2]
        inliers = [p for p in pts if abs(p[0] - med) <= .025]
        if len(inliers) < 4:
            continue
        span = max(p[1] for p in inliers) - min(p[1] for p in inliers)
        if span < .08:
            continue
        out[edge_id] = (orient, sorted(p[0] for p in inliers)[len(inliers) // 2],
                        len(inliers), span)
    return out


def _project(scan, pose, base_from_laser):
    transform = audit.Transform2D(pose.x, pose.y, pose.yaw)
    return audit.scan_frame(scan, Pose2D(0.0, 0.0, 0.0), transform,
                            base_from_laser)


def _read_bag(session):
    meta = session / 'session.yaml'
    values = {}
    for line in meta.read_text(encoding='utf-8').splitlines():
        if ':' in line:
            k, v = line.split(':', 1)
            values[k.strip()] = v.strip().strip('"\'')
    cell = tuple(int(v.strip()) for v in values['cell'].strip('[]').split(','))
    heading = values['heading']
    if heading not in HEADING_RAD:
        raise ValueError(f'unsupported anchor heading: {heading}')
    bags = sorted((session / 'bag').rglob('*.db3'))
    if len(bags) != 1:
        raise ValueError(f'expected one SQLite bag, found {len(bags)}')
    db = sqlite3.connect(f'file:{bags[0]}?mode=ro', uri=True)
    topics = {n: (i, typ) for i, n, typ in
              db.execute('SELECT id,name,type FROM topics')}
    for name in (values['scan_topic'], values['odom_topic'], '/tf_static'):
        if name not in topics:
            db.close()
            raise ValueError(f'missing topic {name}')
    odoms = []
    for (_t, blob) in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[values['odom_topic']][0],)):
        stamp, frame, child, pose = decode_odometry(blob)
        if (frame, child) != (values['odom_frame'], values['base_frame']):
            db.close()
            raise ValueError(f'odom frames {frame}/{child} mismatch session')
        odoms.append((stamp, pose))
    odoms.sort(key=lambda x: x[0])
    tf = []
    for (blob,) in db.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics['/tf_static'][0],)):
        tf.extend(decode_tf_message(blob))
    base_from_laser = static_transform(tf, values['base_frame'],
                                        values['laser_frame'])
    anchor = Pose2D((cell[0] + .5) * CELL,
                    (cell[1] + .5) * CELL, HEADING_RAD[heading])
    return db, topics, values, odoms, base_from_laser, anchor


def _truth_eval(path, seen):
    # Called only after replay; truth never enters association or correction.
    truth = json.loads(path.read_text(encoding='utf-8'))
    expected = audit.edge_truth(truth)
    discovered = {k for k, v in seen.items() if v.stable}
    tp = sum(expected.get(k) is True for k in discovered)
    fp = sum(expected.get(k) is False for k in discovered)
    fn = sum(v is True and k not in discovered for k, v in expected.items())
    return {'anchor_assumption': 'session.yaml cell/heading and ideal 0.4m axes are correct; physical image transform remains unresolved',
            'stable_edges': len(discovered), 'true_positive': tp,
            'false_positive': fp, 'missed_true_wall_edges': fn,
            'precision': tp / (tp + fp) if tp + fp else None,
            'recall': tp / (tp + fn) if tp + fn else None,
            'false_positive_edge_ids': [repr(k) for k in sorted(discovered, key=repr)
                if expected.get(k) is False],
            'true_positive_edge_ids': [repr(k) for k in sorted(discovered, key=repr)
                if expected.get(k) is True]}


def replay(session, log_path, summary_path, truth_path=None):
    if log_path.parent != Path('/tmp') or summary_path.parent != Path('/tmp'):
        raise ValueError('outputs must be directly under /tmp')
    db, topics, values, odoms, extrinsic, corrected = _read_bag(session)
    anchor_pose = corrected
    edge_geometry: dict[tuple, EdgeGeometry] = {}
    counters = Counter()
    correction_sizes = []
    odom_path = 0.0
    estimated_path = 0.0
    prev_odom = prev_corrected = None
    first_odom = None
    anchored_odom_deltas = []
    last_scan_time = None
    log_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with log_path.open('w', encoding='utf-8') as out:
            for (_bag_t, blob) in db.execute(
                    'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                    (topics[values['scan_topic']][0],)):
                scan = decode_laser_scan(blob)
                if scan['frame_id'] != values['laser_frame']:
                    raise ValueError(f"unexpected scan frame {scan['frame_id']}")
                try:
                    odom = interpolate_bounded_pose(odoms, scan['stamp'])
                except ValueError:
                    counters['no_odom_bracket'] += 1
                    continue
                if prev_odom is None:
                    # First manually declared anchor initializes both poses.
                    predicted = corrected
                    first_odom = odom
                else:
                    # Corrected pose(t-1) ⊕ (odom(t-1)^-1 ⊕ odom(t)).
                    delta = audit.compose(audit.inverse(_tf(prev_odom)), _tf(odom))
                    predicted = _pose(audit.compose(_tf(prev_corrected), delta))
                    odom_path += math.hypot(odom.x - prev_odom.x,
                                            odom.y - prev_odom.y)
                    estimated_path += math.hypot(predicted.x - prev_corrected.x,
                                                 predicted.y - prev_corrected.y)

                # Only history-stable line geometries can correct this scan.
                old = [g for g in edge_geometry.values()
                       if g.stable and g.conflicts == 0]
                _, rays, endpoints = _project(scan, predicted, extrinsic)
                walls = tuple(_wall_segment(g.edge_id, g.coordinate) for g in old)
                proposal = propose_pose_correction(endpoints, walls, predicted,
                                                   PoseCorrectionConfig())
                corrected_now = proposal.corrected_pose if proposal.accepted else predicted
                correction = math.hypot(proposal.dx, proposal.dy) if proposal.accepted else 0.0
                if proposal.accepted:
                    counters['accepted_corrections'] += 1
                    correction_sizes.append(correction)
                else:
                    counters['rejected_' + proposal.reason] += 1

                # Now admit current scan line observations using corrected pose.
                corrected_pose, rays2, endpoints2 = _project(scan, corrected_now,
                                                              extrinsic)
                baseline_delta = audit.compose(audit.inverse(_tf(first_odom)),
                                               _tf(odom))
                baseline_pose = _pose(audit.compose(_tf(anchor_pose),
                                                    baseline_delta))
                trajectory_delta = math.hypot(corrected_now.x - baseline_pose.x,
                                              corrected_now.y - baseline_pose.y)
                yaw_delta = math.atan2(
                    math.sin(corrected_now.yaw - baseline_pose.yaw),
                    math.cos(corrected_now.yaw - baseline_pose.yaw))
                anchored_odom_deltas.append((trajectory_delta, abs(yaw_delta)))
                measurements = _frame_line_measurements(rays2, corrected_pose)
                for edge_id, (orient, coordinate, count, span) in measurements.items():
                    item = edge_geometry.get(edge_id)
                    if item is None:
                        _o, ideal, _lo, _hi = _edge_geometry(edge_id)
                        item = EdgeGeometry(edge_id, orient, ideal)
                        edge_geometry[edge_id] = item
                    was_stable = item.stable
                    # Use the wheel-odometry frame as the viewpoint proxy so
                    # map corrections do not manufacture viewpoint diversity.
                    # This still cannot rule out odometry drift masquerading
                    # as physical translation, which remains an experiment
                    # limitation and is why promoted line geometry is frozen.
                    item.observe(coordinate, odom, scan['stamp'])
                    counters['edge_measurements'] += 1
                    if not was_stable and item.stable:
                        counters['promoted_stable_edges'] += 1
                counters['scans'] += 1
                counters['scans_with_stable_prior'] += bool(old)
                row = {
                    'stamp': scan['stamp'], 'elapsed_s': (scan['stamp'] - last_scan_time
                        if last_scan_time is not None else 0.0),
                    'odom_pose': [odom.x, odom.y, odom.yaw],
                    'predicted_pose_from_previous_corrected_plus_odom_delta':
                        [predicted.x, predicted.y, predicted.yaw],
                    'prior_stable_edge_ids': [repr(g.edge_id) for g in old],
                    'proposal': {'accepted': proposal.accepted,
                        'reason': proposal.reason, 'dx_m': proposal.dx,
                        'dy_m': proposal.dy, 'dyaw_rad': proposal.dyaw,
                        'associated_hits': proposal.n_associated,
                        'inliers': proposal.n_inliers,
                        'prior_walls': proposal.n_walls,
                        'median_residual_m': proposal.median_residual_m,
                        'p90_residual_m': proposal.p90_residual_m},
                    'corrected_pose': [corrected_now.x, corrected_now.y,
                                       corrected_now.yaw],
                    'difference_from_anchor_plus_raw_odom_m_rad':
                        [trajectory_delta, yaw_delta],
                    'current_scan_measurements': len(measurements),
                    'stable_edge_count_after_scan': sum(g.stable for g in edge_geometry.values()),
                    'endpoint_geometry': 'merged scan finite hits only; no OPEN evidence'}
                out.write(json.dumps(row, separators=(',', ':')) + '\n')
                prev_odom, prev_corrected = odom, corrected_now
                corrected = corrected_now
                last_scan_time = scan['stamp']
    finally:
        db.close()

    stable = [g for g in edge_geometry.values() if g.stable]
    offsets = [g.coordinate - _edge_geometry(g.edge_id)[1] for g in stable]
    summary = {
        'result': 'OFFLINE_EXPERIMENT_ONLY',
        'session': session.name,
        'model': 'discrete edge identity; continuous axis-aligned normal coordinate per edge',
        'prediction': 'previous corrected pose composed with relative wheel-odometry increment',
        'update_order': 'historical stable conflict-free walls correct pose first; current scan updates candidate geometry after correction; promoted geometry is frozen and conflicts only logged',
        'scans': counters['scans'], 'skipped_no_odom_bracket': counters['no_odom_bracket'],
        'edge_measurements': counters['edge_measurements'],
        'stable_edges_promoted': counters['promoted_stable_edges'],
        'stable_edges_at_end': len(stable),
        'stable_edges_with_conflicts': sum(g.conflicts > 0 for g in stable),
        'total_post_promotion_conflicting_viewpoints': sum(g.conflicts for g in stable),
        'stable_edge_geometry': [{'edge_id': repr(g.edge_id),
            'orientation': g.orientation, 'coordinate_m': g.coordinate,
            'ideal_coordinate_m': _edge_geometry(g.edge_id)[1],
            'offset_from_ideal_m': g.coordinate - _edge_geometry(g.edge_id)[1],
            'independent_view_clusters': len(g.views),
            'conflict_viewpoints': g.conflicts} for g in stable],
        'accepted_pose_corrections': counters['accepted_corrections'],
        'correction_rejections': {k.removeprefix('rejected_'): v for k, v in counters.items()
                                  if k.startswith('rejected_')},
        'correction_translation_m_p50_p90': [audit.quantile(correction_sizes, q)
            for q in (.5, .9)] if correction_sizes else None,
        'stable_wall_offset_from_ideal_m_abs_p50_p90':
            [audit.quantile([abs(x) for x in offsets], q) for q in (.5, .9)]
            if offsets else None,
        'wheel_odom_path_between_scan_samples_m': odom_path,
        'integrated_predicted_path_m': estimated_path,
        'deviation_from_anchor_plus_raw_odom_m_p50_p90_final_max':
            ([audit.quantile([x[0] for x in anchored_odom_deltas], q)
              for q in (.5, .9)] +
             ([anchored_odom_deltas[-1][0],
               max(x[0] for x in anchored_odom_deltas)]
              if anchored_odom_deltas else [])) if anchored_odom_deltas else None,
        'yaw_deviation_from_anchor_plus_raw_odom_rad_p50_p90_final_max':
            ([audit.quantile([x[1] for x in anchored_odom_deltas], q)
              for q in (.5, .9)] +
             ([anchored_odom_deltas[-1][1],
               max(x[1] for x in anchored_odom_deltas)]
              if anchored_odom_deltas else [])) if anchored_odom_deltas else None,
        'stability_rule': 'one robust estimate per edge per scan; at least four wheel-odometry pose clusters; clusters differ by >=0.15m or >=15deg and are >=1s apart; median absolute deviation <=12mm and full spread <=40mm; odometry drift can still fake viewpoint diversity',
        'geometry': 'Manhattan orientation and ideal 0.4m segment identity remain assumptions; only each segment normal coordinate can move; physical wall tilt/length distortion not modeled',
        'limitations': [
            'odometry drift can contaminate geometry before any stable wall correction; stability is temporal and viewpoint-based, not ground-truth validation',
            'edge association still uses nearest ideal-grid line within 0.10m to name candidate edge IDs',
            'only merged /scan_multi is present; sensor origins, source identity, beam times are unavailable, so OPEN evidence and true deskew are excluded',
            'there is no independent physical pose or wall-survey truth; truth topology can score discrete IDs only under the unresolved session anchor/axis assumption',
            'prototype is causal offline evidence, not production-ready and not applied to the vehicle'],
    }
    if truth_path is not None:
        summary['post_replay_topology_score'] = _truth_eval(truth_path, edge_geometry)
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n',
                            encoding='utf-8')
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, default=DEFAULT_SESSION)
    p.add_argument('--log', type=Path, default=DEFAULT_LOG)
    p.add_argument('--summary', type=Path, default=DEFAULT_SUMMARY)
    p.add_argument('--truth-score', type=Path,
                   default=ROOT / 'field' / 'maze_truth_7x7.json')
    args = p.parse_args()
    summary = replay(args.session, args.log, args.summary, args.truth_score)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
