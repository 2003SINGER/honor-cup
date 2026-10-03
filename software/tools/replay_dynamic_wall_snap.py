#!/usr/bin/env python3
"""Causal, read-only per-scan wall correction replay for the joystick bag.

STATUS: SUPERSEDED
SUPERSEDED_BY: replay_frame_grid_snap.py (offline WALL candidate)
Reason: two-scan confirmation produced 55 true / 50 false unique WALL IDs.

This experimental tool reads only the supplied rosbag2 SQLite database. It
does not import maze truth, write ROS state, or infer OPEN edges. Confirmed
walls must have been accumulated from earlier scans in this replay.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sqlite3
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))

import audit_lidar_odom_calibration as audit
from analyze_scan_edge import (CELL_SIZE_M, HEADING_RAD, decode_laser_scan,
    decode_odometry, decode_tf_message, inverse, static_transform)
from m3pro_nav.edge_map import EdgeMap, WALL
from m3pro_nav.grid_association import GridAssociation
from m3pro_nav.pose import Pose2D
from m3pro_nav.pose_correction import (KnownWallSegment, PoseCorrectionConfig,
    ProjectedEndpoint, propose_pose_correction)
from m3pro_nav.observation_adapter import RealObservationAdapter

DEFAULT_SESSION = ROOT / 'field_data' / '20261001_174719_joystick_full_maze'
DEFAULT_LOG = Path('/tmp/honor-cup-dynamic-wall-snap.jsonl')
DEFAULT_SUMMARY = Path('/tmp/honor-cup-dynamic-wall-snap-summary.json')


def confirmed_soft_walls(edge_map: EdgeMap) -> tuple[KnownWallSegment, ...]:
    """Convert only prior-frame SENSOR-confirmed soft WALL edges to segments.

    EdgeMap's UNKNOWN->WALL transition requires two distinct source stamps.
    Hard perimeter assumptions and derived topology walls are intentionally
    excluded because this replay is meant to use observations only.
    """
    out = []
    for key, belief in edge_map.soft.items():
        if belief.get('state') != WALL:
            continue
        if key[0] == 'B':
            _, cell, direction = key
        else:
            cell, direction = key
        i, j = cell
        if direction in ('E', 'W'):
            x = (i + (direction == 'E')) * CELL_SIZE_M
            start, end = (x, j * CELL_SIZE_M), (x, (j + 1) * CELL_SIZE_M)
        else:
            y = (j + (direction == 'N')) * CELL_SIZE_M
            start, end = (i * CELL_SIZE_M, y), ((i + 1) * CELL_SIZE_M, y)
        out.append(KnownWallSegment(start, end, confirmed=True,
                                    wall_id=EdgeMap._fk(edge_map, key)))
    return tuple(out)


def _pose_transform(pose: Pose2D):
    return audit.Transform2D(pose.x, pose.y, pose.yaw)


def interpolate_bounded_pose(samples, stamp: float, max_gap_s: float = 0.15):
    """Strict scan-stamp interpolation; never extrapolate or bridge long gaps."""
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


@dataclass
class ReplayState:
    maze_from_odom: audit.Transform2D
    edge_map: EdgeMap = field(default_factory=lambda: EdgeMap(7))
    last_stamp: float | None = None
    last_odom: Pose2D | None = None
    counts: Counter = field(default_factory=Counter)
    max_confirmed_walls: int = 0


def step_scan(state: ReplayState, scan: dict, odom: Pose2D,
              base_from_laser: audit.Transform2D, *,
              config: PoseCorrectionConfig | None = None) -> dict:
    """Correct from old walls first, then admit this scan's WALL votes."""
    stamp = float(scan['stamp'])
    association = GridAssociation()
    adapter = RealObservationAdapter(association, allow_open_evidence=False)

    speed = yaw_rate = None
    if state.last_stamp is not None and state.last_odom is not None:
        dt = stamp - state.last_stamp
        if dt > 0:
            speed = math.hypot(odom.x - state.last_odom.x,
                               odom.y - state.last_odom.y) / dt
            dyaw = math.atan2(math.sin(odom.yaw - state.last_odom.yaw),
                              math.cos(odom.yaw - state.last_odom.yaw))
            yaw_rate = abs(dyaw) / dt
    motion = ('unknown' if speed is None else
              'moving' if speed > .03 or yaw_rate > .05 else 'static')

    # Propose only from confirmed walls that existed before this scan.
    walls = confirmed_soft_walls(state.edge_map)
    _, rays, endpoints = audit.scan_frame(scan, odom, state.maze_from_odom,
                                         base_from_laser)
    valid_rays = [r for r in rays if r.valid]
    result = propose_pose_correction(
        endpoints, walls,
        audit.compose(state.maze_from_odom, _pose_transform(odom)), config)
    applied = False
    if result.accepted:
        # The correction is fit about the robot base pose. Derive the new
        # frame transform from that corrected base and the same odom pose;
        # left-multiplying (dx,dy,dyaw) rotates about the global origin.
        state.maze_from_odom = audit.compose(
            _pose_transform(result.corrected_pose),
            inverse(_pose_transform(odom)))
        applied = True
        _, rays, endpoints = audit.scan_frame(scan, odom,
                                             state.maze_from_odom,
                                             base_from_laser)
        valid_rays = [r for r in rays if r.valid]

    maze_pose = audit.compose(state.maze_from_odom, _pose_transform(odom))
    hits, _opens, _stats = adapter.to_nav_observation(
        rays, maze_pose, stamp=stamp)
    frame_edges = set()
    for (cell, direction), (distance, _alpha) in hits.items():
        key = state.edge_map.edge_key(cell, direction)
        if key in frame_edges:
            continue
        frame_edges.add(key)
        state.edge_map.observe_wall(cell, direction, dist=distance,
                                    stamp=stamp)

    confirmed_after = len(confirmed_soft_walls(state.edge_map))
    state.max_confirmed_walls = max(state.max_confirmed_walls,
                                    confirmed_after)
    state.counts['scans'] += 1
    state.counts['accepted'] += int(applied)
    state.counts['wall_votes'] += len(frame_edges)
    state.counts['frames_with_prior_walls'] += bool(walls)
    state.counts['frames_with_multi_direction_walls'] += (
        len({(w.start[0] == w.end[0]) for w in walls}) > 1)

    row = {
        'stamp': stamp,
        'odom_pose': [odom.x, odom.y, odom.yaw],
        'motion': motion,
        'speed_mps': speed,
        'yaw_rate_radps': yaw_rate,
        'prior_confirmed_wall_segments': len(walls),
        'prior_confirmed_wall_edge_ids': [w.wall_id for w in walls],
        'wall_vote_edges_this_frame': len(frame_edges),
        'proposal': {
            'reason': result.reason,
            'accepted': result.accepted,
            'dx_m': result.dx,
            'dy_m': result.dy,
            'dyaw_rad': result.dyaw,
            'input_hits': result.n_input_hits,
            'associated': result.n_associated,
            'inliers': result.n_inliers,
            'supporting_walls': result.n_walls,
            'median_residual_m': result.median_residual_m,
            'p90_residual_m': result.p90_residual_m,
            'applied': applied,
        },
        'maze_from_odom': [state.maze_from_odom.x,
                           state.maze_from_odom.y,
                           state.maze_from_odom.yaw],
        'valid_scan_endpoints': len(valid_rays),
        'endpoint_source_limit': (
            '/scan_multi merged scan: source beam and true origin/time are '
            'unavailable; finite endpoints only, no OPEN evidence'),
    }
    state.last_stamp, state.last_odom = stamp, odom
    return row


def load_inputs(session: Path):
    meta = session / 'session.yaml'
    values = {}
    for line in meta.read_text(encoding='utf-8').splitlines():
        if ':' in line:
            key, value = line.split(':', 1)
            values[key.strip()] = value.strip().strip('"\'')
    cell = tuple(int(v.strip()) for v in values['cell'].strip('[]').split(','))
    heading = values['heading']
    if heading not in HEADING_RAD:
        raise ValueError(f'unsupported session heading {heading!r}')
    bags = sorted((session / 'bag').rglob('*.db3'))
    if len(bags) != 1:
        raise ValueError(f'expected exactly one SQLite bag, found {len(bags)}')
    db = sqlite3.connect(f'file:{bags[0]}?mode=ro', uri=True)
    topic_rows = list(db.execute('SELECT id,name,type FROM topics'))
    topics = {name: (tid, msg_type) for tid, name, msg_type in topic_rows}
    required = (values['scan_topic'], values['odom_topic'], '/tf_static')
    missing = [name for name in required if name not in topics]
    if missing:
        db.close()
        raise ValueError(f'bag missing required topics: {missing}')
    odoms = []
    for _, blob in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[values['odom_topic']][0],)):
        stamp, frame, child, pose = decode_odometry(blob)
        if (frame, child) != (values['odom_frame'], values['base_frame']):
            db.close()
            raise ValueError(f'odometry frames {frame}/{child} do not match metadata')
        odoms.append((stamp, pose))
    odoms.sort(key=lambda row: row[0])
    tf = []
    for (blob,) in db.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics['/tf_static'][0],)):
        tf.extend(decode_tf_message(blob))
    base_from_laser = static_transform(tf, values['base_frame'],
                                        values['laser_frame'])
    if not odoms:
        db.close()
        raise ValueError('bag has no odometry samples')
    first_odom = odoms[0][1]
    anchor_pose = audit.Transform2D(
        (cell[0] + .5) * CELL_SIZE_M,
        (cell[1] + .5) * CELL_SIZE_M,
        HEADING_RAD[heading])
    maze_from_odom = audit.compose(anchor_pose,
                                   inverse(_pose_transform(first_odom)))
    return db, topics, values, odoms, base_from_laser, maze_from_odom


def run_replay(session: Path, log_path: Path, summary_path: Path,
               limit_s: float | None = None) -> dict:
    if (Path(os.path.normpath(log_path)).parent != Path('/tmp') or
            Path(os.path.normpath(summary_path)).parent != Path('/tmp')):
        raise ValueError('replay outputs must be written directly under /tmp')
    db, topics, values, odoms, base_from_laser, maze_from_odom = load_inputs(session)
    state = ReplayState(maze_from_odom)
    start_stamp = None
    skipped_pose = 0
    reason_counts = Counter()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with log_path.open('w', encoding='utf-8') as out:
            for _, blob in db.execute(
                    'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                    (topics[values['scan_topic']][0],)):
                scan = decode_laser_scan(blob)
                if scan['frame_id'] != values['laser_frame']:
                    raise ValueError(f"scan frame {scan['frame_id']!r} does not match metadata")
                if start_stamp is None:
                    start_stamp = scan['stamp']
                elapsed = scan['stamp'] - start_stamp
                if limit_s is not None and elapsed > limit_s:
                    break
                try:
                    odom = interpolate_bounded_pose(odoms, scan['stamp'])
                except ValueError as exc:
                    skipped_pose += 1
                    reason_counts['NO_ODOM_BRACKET'] += 1
                    skip_row = {
                        'stamp': scan['stamp'], 'elapsed_s': elapsed,
                        'odom_pose': None,
                        'motion': 'unknown',
                        'speed_mps': None,
                        'yaw_rate_radps': None,
                        'prior_confirmed_wall_segments': len(
                            confirmed_soft_walls(state.edge_map)),
                        'prior_confirmed_wall_edge_ids': [
                            w.wall_id for w in confirmed_soft_walls(state.edge_map)],
                        'wall_vote_edges_this_frame': 0,
                        'proposal': {
                            'reason': 'NO_ODOM_BRACKET', 'detail': str(exc),
                            'accepted': False, 'dx_m': None, 'dy_m': None,
                            'dyaw_rad': None, 'median_residual_m': None,
                            'p90_residual_m': None, 'applied': False},
                        'maze_from_odom': [state.maze_from_odom.x,
                                           state.maze_from_odom.y,
                                           state.maze_from_odom.yaw],
                        'skip_reason': str(exc),
                        'endpoint_source_limit': '/scan_multi; no OPEN evidence'}
                    out.write(json.dumps(skip_row,
                        separators=(',', ':')) + '\n')
                    continue
                row = step_scan(state, scan, odom, base_from_laser)
                row['elapsed_s'] = elapsed
                reason_counts[row['proposal']['reason']] += 1
                out.write(json.dumps(row, separators=(',', ':')) + '\n')
    finally:
        db.close()
    summary = {
        'result': 'DIAGNOSTIC_FAILURE_NOT_FOR_ONLINE_USE',
        'result_note': 'This raw-candidate replay does not establish reliable correction. Acceptance here is not evidence of online readiness; the map is vulnerable to correlated-frame self-reinforcement.',
        'session': session.name,
        'scan_topic': values['scan_topic'],
        'odom_topic': values['odom_topic'],
        'scan_frame': values['laser_frame'],
        'odom_frame': values['odom_frame'],
        'base_frame': values['base_frame'],
        'scans_replayed': state.counts['scans'],
        'scans_skipped_no_odom_pose': skipped_pose,
        'odom_bracket_max_gap_s': 0.15,
        'proposal_reasons': dict(reason_counts),
        'proposals_applied': state.counts['accepted'],
        'wall_votes_added': state.counts['wall_votes'],
        'frames_with_prior_confirmed_walls': state.counts['frames_with_prior_walls'],
        'max_prior_confirmed_wall_segments': state.max_confirmed_walls,
        'final_confirmed_wall_segments': len(confirmed_soft_walls(state.edge_map)),
        'final_edge_states': [
            {'edge_id': EdgeMap._fk(state.edge_map, key),
             'state': belief.get('state'), 'score': belief.get('score')}
            for key, belief in sorted(state.edge_map.soft.items(),
                                      key=lambda item: EdgeMap._fk(state.edge_map, item[0]))],
        'final_maze_from_odom': [state.maze_from_odom.x,
                                 state.maze_from_odom.y,
                                 state.maze_from_odom.yaw],
        'estimator_inputs': 'previous-frame EdgeMap soft SENSOR WALL edges only; diagnostic raw candidates, not production-trusted physical walls; no truth map; no OPEN votes',
        'edge_map_confirmation_limit': 'EdgeMap confirms SENSOR WALL after two distinct scan stamps; this replay bypasses TrustPolicy and is susceptible to adjacent-frame correlated evidence and self-reinforcement',
        'observability_limit': 'A single wall direction constrains wall-normal translation and some yaw but leaves along-wall translation unobservable; the existing solver requires nonparallel wall support and should reject straight-corridor-only frames.',
        'scan_geometry_limit': '/scan_multi has merged endpoints; original beam source/origin/time lost; no deskew or OPEN inference',
        'log_path': str(log_path),
    }
    summary_path.write_text(json.dumps(summary, indent=2) + '\n',
                            encoding='utf-8')
    return summary


def score_prior_walls(log_path: Path, truth_path: Path, output_path: Path) -> dict:
    """Post-hoc evaluation only; called after estimator replay is complete."""
    if Path(os.path.normpath(output_path)).parent != Path('/tmp'):
        raise ValueError('truth-score output must be written directly under /tmp')
    truth = audit.edge_truth(json.loads(truth_path.read_text(encoding='utf-8')))
    keyer = EdgeMap(7)
    truth_by_id = {keyer._fk(edge): is_wall for edge, is_wall in truth.items()}
    frame_tp = frame_fp = frame_n = 0
    edge_seen = set()
    edge_tp = edge_fp = 0
    for line in log_path.read_text(encoding='utf-8').splitlines():
        row = json.loads(line)
        for edge_id in row.get('prior_confirmed_wall_edge_ids', ()):
            if edge_id not in truth_by_id:
                continue
            frame_n += 1
            frame_tp += int(truth_by_id[edge_id])
            frame_fp += int(not truth_by_id[edge_id])
            if edge_id not in edge_seen:
                edge_seen.add(edge_id)
                edge_tp += int(truth_by_id[edge_id])
                edge_fp += int(not truth_by_id[edge_id])
    result = {
        'evaluation': 'post-hoc prior-wall scoring; truth is read only after the estimation JSONL is complete',
        'truth_path': str(truth_path),
        'frame_wall_observations': frame_n,
        'frame_wall_true_positive': frame_tp,
        'frame_wall_false_positive': frame_fp,
        'frame_wall_precision': frame_tp / frame_n if frame_n else None,
        'unique_predicted_edges': len(edge_seen),
        'unique_true_positive_edges': edge_tp,
        'unique_false_positive_edges': edge_fp,
        'unique_edge_precision': edge_tp / len(edge_seen) if edge_seen else None,
        'limitations': 'frame votes are temporally correlated; truth physical image transform unresolved; conditional on session anchor and ideal grid axes',
    }
    output_path.write_text(json.dumps(result, indent=2) + '\n',
                           encoding='utf-8')
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', nargs='?', type=Path, default=DEFAULT_SESSION)
    parser.add_argument('--limit-s', type=float)
    parser.add_argument('--log', type=Path, default=DEFAULT_LOG)
    parser.add_argument('--summary', type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument('--truth-score', type=Path,
                        help='optional post-replay truth score output; must be in /tmp')
    parser.add_argument('--truth', type=Path,
                        default=ROOT / 'field' / 'maze_truth_7x7.json',
                        help='read only for the optional post-replay score')
    args = parser.parse_args()
    summary = run_replay(args.session, args.log, args.summary, args.limit_s)
    if args.truth_score:
        summary['posthoc_truth_score'] = score_prior_walls(
            args.log, args.truth, args.truth_score)
        summary['truth_score_path'] = str(args.truth_score)
        args.summary.write_text(json.dumps(summary, indent=2) + '\n',
                                encoding='utf-8')
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
