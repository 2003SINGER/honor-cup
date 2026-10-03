#!/usr/bin/env python3
"""Offline scan-to-scan ICP versus wheel odometry for the joystick field bag.

STATUS: DIAGNOSTIC
Reason: relative scan matching is not an absolute localization input.

Uses NumPy/SciPy on the Mac for diagnostics only. It does not read maze truth,
write ROS state, alter the navigation map, or infer OPEN edges. Input scans are
the merged /scan_multi endpoint clouds; their original sensor origins and beam
times are unavailable, so estimates are geometric diagnostics only.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_scan_edge import decode_laser_scan, decode_odometry

DEFAULT_SESSION = ROOT / 'field_data' / '20261001_174719_joystick_full_maze'
DEFAULT_LOG = Path('/tmp/honor-cup-scan-match-odom.jsonl')
DEFAULT_SUMMARY = Path('/tmp/honor-cup-scan-match-odom-summary.json')

# Fixed conservative gates. These are diagnostic acceptance gates, not tuned
# against maze truth. In particular, weak geometry falls back to odometry.
MAX_ODOM_BRACKET_S = 0.15
MAX_SCAN_DT_S = 0.25
MAX_ODOM_STEP_M = 0.20
MAX_ODOM_STEP_YAW_RAD = math.radians(25)
MIN_POINTS = 60
MIN_MATCHES = 50
MAX_CORRESPONDENCE_M = 0.15
TRIM_FRACTION = 0.70
MAX_P90_RESIDUAL_M = 0.12
MAX_ICP_STEP_M = 0.20
MAX_ICP_STEP_YAW_RAD = math.radians(25)
MAX_ICP_ODOM_DELTA_M = 0.08
MAX_ICP_ODOM_DELTA_YAW_RAD = math.radians(12)
STATIONARY_ODOM_M = 0.0015
STATIONARY_ODOM_YAW_RAD = math.radians(0.1)
STATIONARY_ICP_M = 0.002
STATIONARY_ICP_YAW_RAD = math.radians(0.15)
MAX_ITERATIONS = 30


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c*b[0] - s*b[1],
            a[1] + s*b[0] + c*b[1], wrap_angle(a[2] + b[2]))


def inverse(t):
    c, s = math.cos(t[2]), math.sin(t[2])
    return (-c*t[0] - s*t[1], s*t[0] - c*t[1], wrap_angle(-t[2]))


def interpolate_odom(samples, stamp):
    times = [row[0] for row in samples]
    i = bisect.bisect_left(times, stamp)
    if i < len(samples) and times[i] == stamp:
        p = samples[i][1]
        return (p.x, p.y, p.yaw)
    if i == 0 or i == len(samples):
        raise ValueError('NO_ODOM_BRACKET')
    ta, a = samples[i-1]
    tb, b = samples[i]
    if tb - ta > MAX_ODOM_BRACKET_S:
        raise ValueError('ODOM_BRACKET_GAP')
    f = (stamp-ta)/(tb-ta)
    dyaw = wrap_angle(b.yaw-a.yaw)
    return (a.x+f*(b.x-a.x), a.y+f*(b.y-a.y),
            wrap_angle(a.yaw+f*dyaw))


def scan_cloud(scan, min_range=0.08, max_range=3.5):
    ranges = np.asarray(scan['ranges'], dtype=np.float64)
    angles = scan['angle_min'] + np.arange(len(ranges))*scan['angle_increment']
    valid = np.isfinite(ranges) & (ranges >= max(min_range, scan['range_min'])) & \
            (ranges <= min(max_range, scan['range_max']))
    r, a = ranges[valid], angles[valid]
    return np.column_stack((r*np.cos(a), r*np.sin(a)))


def _apply(points, transform):
    c, s = math.cos(transform[2]), math.sin(transform[2])
    rotation = np.array(((c, -s), (s, c)))
    return points @ rotation.T + np.array(transform[:2])


def _rigid_fit(source, target):
    src_mean = source.mean(axis=0)
    dst_mean = target.mean(axis=0)
    covariance = (source-src_mean).T @ (target-dst_mean)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = dst_mean - rotation @ src_mean
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return (float(translation[0]), float(translation[1]), yaw)


def trimmed_icp(source, target, initial):
    """Map current scan points into previous scan frame using trimmed ICP."""
    if len(source) < MIN_POINTS or len(target) < MIN_POINTS:
        return {'accepted': False, 'reason': 'TOO_FEW_SCAN_POINTS',
                'matches': 0, 'transform': None, 'p50_m': None, 'p90_m': None,
                'iterations': 0}
    tree = cKDTree(target)
    transform = tuple(initial)
    iterations = 0
    for iterations in range(1, MAX_ITERATIONS + 1):
        projected = _apply(source, transform)
        distances, indices = tree.query(projected, k=1, workers=1)
        candidates = np.flatnonzero(distances <= MAX_CORRESPONDENCE_M)
        if len(candidates) < MIN_MATCHES:
            return {'accepted': False, 'reason': 'ICP_INSUFFICIENT_MATCHES',
                    'matches': int(len(candidates)), 'transform': transform,
                    'p50_m': None, 'p90_m': None, 'iterations': iterations}
        keep_n = max(MIN_MATCHES, int(len(candidates)*TRIM_FRACTION))
        kept = candidates[np.argpartition(distances[candidates], keep_n-1)[:keep_n]]
        increment = _rigid_fit(projected[kept], target[indices[kept]])
        transform = compose(increment, transform)
        if math.hypot(increment[0], increment[1]) < 1e-5 and \
                abs(increment[2]) < 1e-5:
            break
    projected = _apply(source, transform)
    distances, _ = tree.query(projected, k=1, workers=1)
    candidates = np.flatnonzero(distances <= MAX_CORRESPONDENCE_M)
    if len(candidates) < MIN_MATCHES:
        return {'accepted': False, 'reason': 'ICP_INSUFFICIENT_MATCHES',
                'matches': int(len(candidates)), 'transform': transform,
                'p50_m': None, 'p90_m': None, 'iterations': iterations}
    keep_n = max(MIN_MATCHES, int(len(candidates)*TRIM_FRACTION))
    kept = candidates[np.argpartition(distances[candidates], keep_n-1)[:keep_n]]
    residuals = distances[kept]
    p50, p90 = (float(np.quantile(residuals, q)) for q in (.5, .9))
    reason = 'ACCEPTED' if p90 <= MAX_P90_RESIDUAL_M else 'ICP_HIGH_RESIDUAL'
    return {'accepted': reason == 'ACCEPTED', 'reason': reason,
            'matches': int(len(kept)), 'transform': transform,
            'p50_m': p50, 'p90_m': p90, 'iterations': iterations}


def load_session(session: Path):
    values = {}
    for line in (session/'session.yaml').read_text(encoding='utf-8').splitlines():
        if ':' in line:
            k, v = line.split(':', 1)
            values[k.strip()] = v.strip().strip('"\'')
    bags = sorted((session/'bag').rglob('*.db3'))
    if len(bags) != 1:
        raise ValueError(f'expected one SQLite bag, found {len(bags)}')
    db = sqlite3.connect(f'file:{bags[0]}?mode=ro', uri=True)
    topics = {name: (tid, typ) for tid, name, typ in
              db.execute('SELECT id,name,type FROM topics')}
    for name in (values['scan_topic'], values['odom_topic']):
        if name not in topics:
            db.close()
            raise ValueError(f'missing topic {name}')
    odoms = []
    for _, blob in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[values['odom_topic']][0],)):
        stamp, frame, child, pose = decode_odometry(blob)
        if (frame, child) != (values['odom_frame'], values['base_frame']):
            db.close()
            raise ValueError(f'odom frame mismatch: {frame}/{child}')
        odoms.append((stamp, pose))
    scans = [decode_laser_scan(blob) for _, blob in db.execute(
        'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
        (topics[values['scan_topic']][0],))]
    db.close()
    return values, odoms, scans


def _translation(t):
    return math.hypot(t[0], t[1])


def _close_stationary(odom_delta, icp_delta):
    return (_translation(odom_delta) <= STATIONARY_ODOM_M and
            abs(odom_delta[2]) <= STATIONARY_ODOM_YAW_RAD and
            _translation(icp_delta) <= STATIONARY_ICP_M and
            abs(icp_delta[2]) <= STATIONARY_ICP_YAW_RAD)


def replay(session: Path, log_path: Path, summary_path: Path,
           limit_s: float | None = None):
    if log_path.parent != Path('/tmp') or summary_path.parent != Path('/tmp'):
        raise ValueError('outputs must be directly under /tmp')
    values, odoms, scans = load_session(session)
    if len(scans) < 2:
        raise ValueError('need at least two scan frames')
    odom_poses = []
    clouds = []
    stamps = []
    for scan in scans:
        stamps.append(scan['stamp'])
        clouds.append(scan_cloud(scan))
        try:
            odom_poses.append(interpolate_odom(odoms, scan['stamp']))
        except ValueError as exc:
            odom_poses.append(None)

    start = stamps[0]
    trajectory = (0.0, 0.0, 0.0)
    odom_trajectory = (0.0, 0.0, 0.0)
    records = []
    reasons = Counter()
    accepted = fallback = held = 0
    max_correction = 0.0
    last_valid_index = 0 if odom_poses[0] is not None else None
    for i in range(1, len(scans)):
        stamp, prev_stamp = stamps[i], stamps[i-1]
        elapsed, dt = stamp-start, stamp-prev_stamp
        if limit_s is not None and elapsed > limit_s:
            break
        current = odom_poses[i]
        previous_index = last_valid_index
        previous = odom_poses[previous_index] if previous_index is not None else None
        row = {
            'pair_index': i-1, 'previous_stamp': prev_stamp, 'stamp': stamp,
            'source_scan_index': previous_index,
            'elapsed_s': elapsed, 'scan_dt_s': dt,
            'previous_points': int(len(clouds[i-1])),
            'current_points': int(len(clouds[i])),
            'odom_relative': None, 'icp_relative': None,
            'matches': 0, 'p50_residual_m': None, 'p90_residual_m': None,
            'iterations': 0, 'odom_difference_translation_m': None,
            'odom_difference_yaw_rad': None,
            'reason': None, 'fallback_reason': None,
            'accumulation': 'held', 'lidar_assisted_pose': list(trajectory),
            'odom_only_pose': list(odom_trajectory),
        }
        reason = None
        if current is None:
            reason = 'NO_ODOM_BRACKET'
        if reason is not None:
            row['reason'] = reason
            reasons[reason] += 1
            row['lidar_assisted_pose'] = list(trajectory)
            records.append(row)
            continue

        if previous is None:
            row['reason'] = 'NO_VALID_PREVIOUS_ODOM'
            reasons[row['reason']] += 1
            records.append(row)
            last_valid_index = i
            continue

        odom_delta = compose(inverse(previous), current)
        row['odom_relative'] = list(odom_delta)
        row['source_scan_index'] = previous_index
        source_dt = stamp - stamps[previous_index]
        row['scan_span_s'] = source_dt
        if _translation(odom_delta) > MAX_ODOM_STEP_M:
            reason = 'ODOM_TRANSLATION_JUMP'
        elif abs(odom_delta[2]) > MAX_ODOM_STEP_YAW_RAD:
            reason = 'ODOM_YAW_JUMP'
        elif dt <= 0 or source_dt > MAX_SCAN_DT_S:
            reason = 'SCAN_TIME_GAP'
            row['accumulation'] = 'odom_fallback'
            row['fallback_reason'] = reason
            trajectory = compose(trajectory, odom_delta)
            odom_trajectory = compose(odom_trajectory, odom_delta)
            fallback += 1
            reasons[reason] += 1
            row['reason'] = reason
            row['lidar_assisted_pose'] = list(trajectory)
            row['odom_only_pose'] = list(odom_trajectory)
            records.append(row)
            last_valid_index = i
            continue
        else:
            fit = trimmed_icp(clouds[i], clouds[previous_index], odom_delta)
            row['matches'] = fit['matches']
            row['iterations'] = fit['iterations']
            row['p50_residual_m'] = fit['p50_m']
            row['p90_residual_m'] = fit['p90_m']
            if fit['transform'] is not None:
                row['icp_relative'] = list(fit['transform'])
                diff = compose(inverse(odom_delta), fit['transform'])
                row['odom_difference_translation_m'] = _translation(diff)
                row['odom_difference_yaw_rad'] = diff[2]
            reason = fit['reason']
            if fit['accepted']:
                icp_delta = fit['transform']
                if (_translation(icp_delta) > MAX_ICP_STEP_M or
                        abs(icp_delta[2]) > MAX_ICP_STEP_YAW_RAD):
                    reason = 'ICP_LARGE_STEP'
                elif (_translation(compose(inverse(odom_delta), icp_delta)) >
                      MAX_ICP_ODOM_DELTA_M or
                      abs(compose(inverse(odom_delta), icp_delta)[2]) >
                      MAX_ICP_ODOM_DELTA_YAW_RAD):
                    reason = 'ICP_ODOM_DISAGREEMENT'
                elif _close_stationary(odom_delta, icp_delta):
                    reason = 'STATIONARY_NO_ACCUMULATION'
                    row['accumulation'] = 'held_stationary'
                    held += 1
                else:
                    trajectory = compose(trajectory, icp_delta)
                    row['accumulation'] = 'icp'
                    accepted += 1
                    max_correction = max(max_correction,
                        _translation(compose(inverse(odom_delta), icp_delta)))
            if reason != 'ACCEPTED' and row['accumulation'] != 'held_stationary':
                if reason in ('ODOM_TRANSLATION_JUMP', 'ODOM_YAW_JUMP'):
                    row['accumulation'] = 'held_jump'
                    held += 1
                else:
                    trajectory = compose(trajectory, odom_delta)
                    row['accumulation'] = 'odom_fallback'
                    row['fallback_reason'] = reason
                    fallback += 1
        if reason not in ('ODOM_TRANSLATION_JUMP', 'ODOM_YAW_JUMP'):
            odom_trajectory = compose(odom_trajectory, odom_delta)
        row['reason'] = reason
        reasons[reason] += 1
        row['lidar_assisted_pose'] = list(trajectory)
        row['odom_only_pose'] = list(odom_trajectory)
        records.append(row)
        last_valid_index = i

    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open('w', encoding='utf-8') as f:
        for row in records:
            f.write(json.dumps(row, separators=(',', ':'))+'\n')
    summary = summarize(session.name, values, records, reasons, accepted,
                        fallback, held, max_correction)
    summary_path.write_text(json.dumps(summary, indent=2)+'\n',
                            encoding='utf-8')
    return summary


def summarize(session_name, values, records, reasons, accepted, fallback,
              held, max_correction):
    def bucket(rows):
        icp = [r for r in rows if r['icp_relative'] is not None]
        valid = [r for r in icp if r['reason'] in ('ACCEPTED',
                  'STATIONARY_NO_ACCUMULATION')]
        p90s = [r['p90_residual_m'] for r in icp
                if r['p90_residual_m'] is not None]
        diffs = [r['odom_difference_translation_m'] for r in icp
                 if r['odom_difference_translation_m'] is not None]
        final_lidar = rows[-1]['lidar_assisted_pose'] if rows else None
        final_odom = rows[-1]['odom_only_pose'] if rows else None
        if final_lidar is not None and final_odom is not None:
            trajectory_difference = compose(inverse(final_odom), final_lidar)
            final_divergence = {
                'translation_m': _translation(trajectory_difference),
                'yaw_deg': math.degrees(trajectory_difference[2]),
            }
        else:
            final_divergence = None
        return {
            'pairs': len(rows),
            'icp_estimated_pairs': len(icp),
            'geometrically_accepted_pairs': len(valid),
            'fallback_pairs': sum(r['accumulation'] == 'odom_fallback' for r in rows),
            'held_pairs': sum(r['accumulation'].startswith('held') for r in rows),
            'reasons': dict(Counter(r['reason'] for r in rows)),
            'matches_p50': float(np.median([r['matches'] for r in icp])) if icp else None,
            'p90_residual_m_p50_p90': ([float(np.quantile(p90s,.5)),
                                       float(np.quantile(p90s,.9))] if p90s else None),
            'icp_odom_translation_difference_m_p50_p90': (
                [float(np.quantile(diffs,.5)),float(np.quantile(diffs,.9))]
                if diffs else None),
            'lidar_assisted_final_pose': final_lidar,
            'odom_only_final_pose': final_odom,
            'final_lidar_vs_odom_divergence': final_divergence,
        }
    early = [r for r in records if r['elapsed_s'] <= 30.0]
    return {
        'session': session_name,
        'scan_topic': values['scan_topic'], 'odom_topic': values['odom_topic'],
        'implementation': 'NumPy SVD + SciPy cKDTree trimmed point-to-point ICP; offline Mac diagnostic dependency',
        'estimator_boundary': 'No truth map input; no OPEN inference; no ROS/nav/map writes',
        'scan_geometry_limit': '/scan_multi merges sensors and loses original beam source/origin/time; this is not deskewed raw-lidar odometry',
        'gates': {
            'odom_bracket_s': MAX_ODOM_BRACKET_S, 'scan_dt_s': MAX_SCAN_DT_S,
            'odom_step_m': MAX_ODOM_STEP_M,
            'odom_step_yaw_deg': math.degrees(MAX_ODOM_STEP_YAW_RAD),
            'min_scan_points': MIN_POINTS, 'min_matches': MIN_MATCHES,
            'correspondence_gate_m': MAX_CORRESPONDENCE_M,
            'trim_fraction': TRIM_FRACTION, 'p90_residual_m': MAX_P90_RESIDUAL_M,
            'icp_odom_delta_m': MAX_ICP_ODOM_DELTA_M,
            'icp_odom_delta_yaw_deg': math.degrees(MAX_ICP_ODOM_DELTA_YAW_RAD),
            'stationary_hold_odom_m_per_pair': STATIONARY_ODOM_M,
            'stationary_hold_icp_m_per_pair': STATIONARY_ICP_M,
        },
        'all_pairs': bucket(records), 'first_30_seconds': bucket(early),
        'counts': {'icp_accumulated': accepted, 'odom_fallback': fallback,
                   'held': held, 'reason_counts': dict(reasons),
                   'max_accepted_odom_translation_correction_m': max_correction},
        'assessment': 'DIAGNOSTIC_FAILURE_NOT_A_STABLE_GEOMETRIC_PRIOR; cumulative ICP and odometry divergence is reported without independent pose truth, so neither track can be declared accurate. Low local residuals do not rule out corridor sliding or symmetric-scene ambiguity.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', nargs='?', type=Path, default=DEFAULT_SESSION)
    parser.add_argument('--limit-s', type=float)
    parser.add_argument('--log', type=Path, default=DEFAULT_LOG)
    parser.add_argument('--summary', type=Path, default=DEFAULT_SUMMARY)
    args = parser.parse_args()
    summary = replay(args.session, args.log, args.summary, args.limit_s)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
