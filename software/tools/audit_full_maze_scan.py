#!/usr/bin/env python3
"""Read-only full-maze scan audit, stratified by odometry motion and distance.

Example:
  python3 software/tools/audit_full_maze_scan.py \
    field_data/20261001_174719_joystick_full_maze

This compares projected returns with the supplied abstract 7x7 truth under the
session's cell/heading anchor. It does not infer a physical image transform or
apply any correction.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))

from analyze_scan_edge import (CDRReader, HEADING_RAD, CELL_SIZE_M,
                               Pose2D, Transform2D, compose, inverse,
                               decode_odometry, decode_tf_message,
                               static_transform, load_session_value,
                               quantile)
from audit_lidar_odom_calibration import edge_truth, scan_frame
from m3pro_nav.grid_association import GridAssociation, edge_id_for_line


def decode_scan(data: bytes) -> dict:
    """Decode LaserScan timing fields as well as the shared audit geometry."""
    r = CDRReader(data)
    stamp, frame_id = r.read_header()
    angle_min, angle_max, angle_increment = r.read('fff')
    time_increment, scan_time, range_min, range_max = r.read('ffff')
    count = r.read('I')
    if count > 1_000_000:
        raise ValueError(f'implausible LaserScan range count: {count}')
    ranges = tuple(r.read('f') for _ in range(count))
    intensity_count = r.read('I')
    for _ in range(intensity_count):
        r.read('f')
    return {'stamp': stamp, 'frame_id': frame_id,
            'angle_min': angle_min, 'angle_max': angle_max,
            'angle_increment': angle_increment,
            'time_increment': time_increment, 'scan_time': scan_time,
            'range_min': range_min, 'range_max': range_max,
            'ranges': ranges}


def truth_wall_segments(truth: dict) -> tuple[tuple[float, float, float, float], ...]:
    """Return each unique finite wall segment in the supplied 7x7 truth."""
    edges = edge_truth(truth)
    segments = {}
    c = float(truth['grid']['cell_size_m'])
    for y, row in enumerate(truth['open_directions_by_cell']):
        for x, directions in enumerate(row):
            for direction in 'NESW':
                if direction in directions:
                    continue
                if direction == 'N':
                    key = edge_id_for_line('H', y + 1, x)
                    segment = (x*c, (y+1)*c, (x+1)*c, (y+1)*c)
                elif direction == 'S':
                    key = edge_id_for_line('H', y, x)
                    segment = (x*c, y*c, (x+1)*c, y*c)
                elif direction == 'E':
                    key = edge_id_for_line('V', x + 1, y)
                    segment = ((x+1)*c, y*c, (x+1)*c, (y+1)*c)
                else:
                    key = edge_id_for_line('V', x, y)
                    segment = (x*c, y*c, x*c, (y+1)*c)
                if edges[key]:
                    segments[key] = segment
    return tuple(segments.values())


def point_to_segment_distance(point: tuple[float, float],
                              segment: tuple[float, float, float, float]) -> float:
    """Euclidean distance to a finite line segment, including its endpoints."""
    x, y = point
    ax, ay, bx, by = segment
    vx, vy = bx - ax, by - ay
    length2 = vx*vx + vy*vy
    if length2 <= 0:
        raise ValueError('wall segment must have positive length')
    t = max(0.0, min(1.0, ((x-ax)*vx + (y-ay)*vy) / length2))
    return math.hypot(x - (ax + t*vx), y - (ay + t*vy))


def motion_class(speed_mps: float, yaw_rate_radps: float, *,
                 static_speed_mps: float = 0.03,
                 static_yaw_rate_radps: float = 0.05) -> str:
    return ('static' if speed_mps < static_speed_mps and
            yaw_rate_radps < static_yaw_rate_radps else 'moving')


def summary(values: list[float]) -> dict:
    if not values:
        return {'n': 0}
    ordered = sorted(values)
    return {
        'n': len(ordered),
        'distance_mm_p50_p90_p95': [
            round(1000 * quantile(ordered, q), 2) for q in (.5, .9, .95)],
        'fraction_within_50mm': round(
            sum(v <= .05 for v in ordered) / len(ordered), 5),
        'fraction_within_100mm': round(
            sum(v <= .10 for v in ordered) / len(ordered), 5),
    }


def _bucket_report(records: list[dict], key: str, width: float) -> list[dict]:
    buckets = defaultdict(list)
    for record in records:
        buckets[int(record[key] // width)].append(record)
    result = []
    for index, rows in sorted(buckets.items()):
        distances = [d for row in rows for d in row['wall_distances']]
        result.append({
            'start': round(index * width, 3),
            'end': round((index + 1) * width, 3),
            'frames': len(rows),
            'speed_mps_p50': round(quantile([r['speed_mps'] for r in rows], .5), 4),
            'wall_point_distance': summary(distances),
            'unique_wall_truth_precision': _precision(rows, 'unique_wall_correct',
                                                       'unique_wall_false'),
            'open_truth_precision': _precision(rows, 'open_correct', 'open_false'),
        })
    return result


def _precision(rows: list[dict], good_key: str, bad_key: str) -> dict:
    good = sum(row[good_key] for row in rows)
    bad = sum(row[bad_key] for row in rows)
    n = good + bad
    return {'correct': good, 'incorrect': bad,
            'precision': round(good / n, 5) if n else None}


def audit_session(session: Path, truth: dict, *, static_speed_mps: float = .03,
                  static_yaw_rate_radps: float = .05) -> dict:
    cell_text = load_session_value(session / 'session.yaml', 'cell')
    cell = tuple(int(v.strip()) for v in cell_text.strip('[]').split(','))
    heading = load_session_value(session / 'session.yaml', 'heading')
    if heading not in HEADING_RAD:
        raise ValueError(f'unsupported session heading {heading!r}')
    scan_topic = load_session_value(session / 'session.yaml', 'scan_topic')
    odom_topic = load_session_value(session / 'session.yaml', 'odom_topic')
    laser_frame = load_session_value(session / 'session.yaml', 'laser_frame')
    odom_frame = load_session_value(session / 'session.yaml', 'odom_frame')
    base_frame = load_session_value(session / 'session.yaml', 'base_frame')
    bags = sorted((session / 'bag').rglob('*.db3'))
    if len(bags) != 1:
        raise ValueError(f'expected one SQLite bag, found {len(bags)}')

    db = sqlite3.connect(f'file:{bags[0]}?mode=ro', uri=True)
    topics = {name: (topic_id, msg_type) for topic_id, name, msg_type
              in db.execute('SELECT id,name,type FROM topics')}
    for name in (scan_topic, odom_topic, '/tf_static'):
        if name not in topics:
            raise ValueError(f'bag is missing {name}')

    odom_arrivals = []
    for bag_ns, blob in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[odom_topic][0],)):
        stamp, frame, child, pose = decode_odometry(blob)
        if (frame, child) != (odom_frame, base_frame):
            raise ValueError(f'odometry frame mismatch {frame}/{child}')
        odom_arrivals.append((stamp, pose, bag_ns * 1e-9))
    odoms = sorted((stamp, pose) for stamp, pose, _ in odom_arrivals)
    if len(odoms) < 2:
        raise ValueError('need at least two odometry messages')
    odom_times = [item[0] for item in odoms]
    odom_arrival_times = [item[2] for item in odom_arrivals]
    path_at_sample = [0.0]
    for (_, a), (_, b) in zip(odoms, odoms[1:]):
        path_at_sample.append(path_at_sample[-1] + math.hypot(b.x-a.x, b.y-a.y))

    static_edges = []
    for (blob,) in db.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics['/tf_static'][0],)):
        static_edges.extend(decode_tf_message(blob))
    base_from_laser = static_transform(static_edges, base_frame, laser_frame)
    first = odoms[0][1]
    anchor = Pose2D((cell[0] + .5) * CELL_SIZE_M,
                    (cell[1] + .5) * CELL_SIZE_M, HEADING_RAD[heading])
    maze_from_odom = compose(Transform2D(anchor.x, anchor.y, anchor.yaw),
                             inverse(Transform2D(first.x, first.y, first.yaw)))
    truths = edge_truth(truth)
    walls = truth_wall_segments(truth)
    association = GridAssociation()
    records = []
    skipped_time = 0
    scan_fields = []
    raw_scan_by_ms = {}

    for bag_ns, blob in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[scan_topic][0],)):
        scan = decode_scan(blob)
        raw_scan_by_ms[round(scan['stamp'], 3)] = {
            'valid': sum(scan['range_min'] <= r <= scan['range_max']
                         for r in scan['ranges'] if math.isfinite(r))}
        if scan['frame_id'] != laser_frame:
            raise ValueError(f'scan frame mismatch {scan["frame_id"]!r}')
        stamp = scan['stamp']
        i = bisect.bisect_left(odom_times, stamp)
        if i == 0 or i == len(odom_times):
            skipped_time += 1
            continue
        ta, pa = odoms[i-1]
        tb, pb = odoms[i]
        dt = tb - ta
        if dt <= 0 or dt > .5:
            skipped_time += 1
            continue
        fraction = (stamp - ta) / dt
        dyaw = math.atan2(math.sin(pb.yaw-pa.yaw), math.cos(pb.yaw-pa.yaw))
        pose = Pose2D(pa.x + fraction*(pb.x-pa.x),
                      pa.y + fraction*(pb.y-pa.y), pa.yaw + fraction*dyaw)
        speed = math.hypot(pb.x-pa.x, pb.y-pa.y) / dt
        yaw_rate = abs(dyaw) / dt
        mileage = path_at_sample[i-1] + fraction * (
            path_at_sample[i] - path_at_sample[i-1])
        elapsed = stamp - odom_times[0]
        maze_pose, rays, points = scan_frame(scan, pose, maze_from_odom,
                                             base_from_laser)
        distances = [min(point_to_segment_distance((p.x, p.y), wall)
                         for wall in walls) for p in points]
        observations = association.process(rays)
        wall_good = wall_bad = open_good = open_bad = 0
        outcome_counts = defaultdict(int)
        for observation in observations:
            outcome_counts[observation.outcome] += 1
            candidate = observation.candidate
            if observation.outcome == 'UNIQUE' and candidate is not None:
                expected = truths.get(candidate.edge_id)
                if expected is True:
                    wall_good += 1
                elif expected is False:
                    wall_bad += 1
            for edge in observation.open_edges:
                expected = truths.get(edge)
                if expected is False:
                    open_good += 1
                elif expected is True:
                    open_bad += 1
        nearest_skew = min(stamp-ta, tb-stamp)
        arrival_index = bisect.bisect_right(odom_arrival_times, bag_ns*1e-9)-1
        latest_arrival_skew = (stamp - odom_arrivals[arrival_index][0]
                               if arrival_index >= 0 else None)
        records.append({
            'stamp': stamp, 'elapsed_s': elapsed, 'mileage_m': mileage,
            'speed_mps': speed, 'yaw_rate_radps': yaw_rate,
            'motion': motion_class(speed, yaw_rate,
                                   static_speed_mps=static_speed_mps,
                                   static_yaw_rate_radps=static_yaw_rate_radps),
            'nearest_odom_skew_s': nearest_skew,
            'latest_arrival_odom_skew_s': latest_arrival_skew,
            'wall_distances': distances,
            'outcomes': dict(outcome_counts),
            'unique_wall_correct': wall_good, 'unique_wall_false': wall_bad,
            'open_correct': open_good, 'open_false': open_bad,
        })
        scan_fields.append((scan['time_increment'], scan['scan_time']))
    db.close()

    if not records:
        raise ValueError('no scan had odometry source-stamp coverage')
    speeds = [r['speed_mps'] for r in records]
    yaw_rates = [r['yaw_rate_radps'] for r in records]
    all_distances = [d for r in records for d in r['wall_distances']]
    classes = {}
    for label in ('static', 'moving'):
        rows = [r for r in records if r['motion'] == label]
        classes[label] = {
            'frames': len(rows),
            'speed_mps_p50_p90': [quantile([r['speed_mps'] for r in rows], q)
                                  for q in (.5, .9)] if rows else [],
            'yaw_rate_radps_p50_p90': [quantile([r['yaw_rate_radps'] for r in rows], q)
                                       for q in (.5, .9)] if rows else [],
            'wall_point_distance': summary(
                [d for r in rows for d in r['wall_distances']]),
            'unique_wall_truth_precision': _precision(
                rows, 'unique_wall_correct', 'unique_wall_false'),
            'open_truth_precision': _precision(rows, 'open_correct', 'open_false'),
            'nearest_odom_skew_ms_p50_p95': [
                round(1000*quantile([r['nearest_odom_skew_s'] for r in rows], q), 2)
                for q in (.5, .95)] if rows else [],
        }
    odom_intervals = [b-a for a, b in zip(odom_times, odom_times[1:])]
    abs_latest_lag = [abs(r['latest_arrival_odom_skew_s']) for r in records
                      if r['latest_arrival_odom_skew_s'] is not None]
    legacy_path = session / 'frames.jsonl'
    legacy_frames = ([json.loads(line) for line in legacy_path.read_text(
        encoding='utf-8').splitlines() if line.strip()]
        if legacy_path.exists() else [])
    by_stamp = {round(r['stamp'], 3): r for r in records}
    matched_legacy = [f for f in legacy_frames
                      if round(float(f['stamp']), 3) in raw_scan_by_ms]
    valid_matches = [f for f in matched_legacy
                     if f.get('n_valid') == raw_scan_by_ms[
                         round(float(f['stamp']), 3)]['valid']]
    outcome_matches = [f for f in matched_legacy
                       if round(float(f['stamp']), 3) in by_stamp and all(
                           f.get('outcomes', {}).get(k, 0) ==
                           by_stamp[round(float(f['stamp']), 3)]['outcomes'].get(k, 0)
                           for k in ('UNIQUE', 'AMBIGUOUS', 'NONE'))]
    return {
        'session': session.name,
        'anchor_assumption': {'cell': list(cell), 'heading': heading,
                              'cell_size_m': CELL_SIZE_M,
                              'physical_axis_transform_resolved': False},
        'frame_counts': {'scan_messages': len(records) + skipped_time,
                         'projected': len(records), 'skipped_no_source_bracket': skipped_time,
                         'wall_segments': len(walls),
                         'finite_returns': len(all_distances)},
        'scan_timing': {
            'unique_time_increment_s': sorted(set(x[0] for x in scan_fields)),
            'unique_scan_time_s': sorted(set(x[1] for x in scan_fields)),
            'recorded_scan_topics': sorted(name for name in topics
                                           if name.startswith('/scan')),
            'interpretation': 'zero fields provide no per-beam timing; deskew cannot be reconstructed from this merged bag',
        },
        'trajectory': {
            'odom_samples': len(odoms),
            'path_length_m': round(path_at_sample[-1], 4),
            'max_displacement_from_start_m': round(max(
                math.hypot(p.x-first.x, p.y-first.y) for _, p in odoms), 4),
            'local_speed_mps_p50_p90_p95_max': [
                round(quantile(speeds, q), 4) for q in (.5, .9, .95)] +
                [round(max(speeds), 4)],
            'local_yaw_rate_radps_p50_p90_p95_max': [
                round(quantile(yaw_rates, q), 4) for q in (.5, .9, .95)] +
                [round(max(yaw_rates), 4)],
            'odom_interval_s_p50_p95_max': [
                round(quantile(odom_intervals, q), 5) for q in (.5, .95)] +
                [round(max(odom_intervals), 5)],
        },
        'wall_comparison': {
            'distance_to_nearest_finite_truth_wall_segment': summary(all_distances),
            'motion_classes': classes,
            'by_elapsed_30s': _bucket_report(records, 'elapsed_s', 30.0),
            'by_odom_mileage_5m': _bucket_report(records, 'mileage_m', 5.0),
        },
        'scan_odom_timing': {
            'nearest_source_stamp_offset_ms_p50_p95_max': [
                round(1000*quantile([r['nearest_odom_skew_s'] for r in records], q), 2)
                for q in (.5, .95)] +
                [round(1000*max(r['nearest_odom_skew_s'] for r in records), 2)],
            'latest_arrived_odom_source_stamp_lag_abs_ms_p50_p95_max': [
                round(1000*quantile(abs_latest_lag, q), 2)
                for q in (.5, .95)] + [round(1000*max(abs_latest_lag), 2)],
            'arrival_lag_is_bag_record_order_proxy': True,
        },
        'legacy_frames_jsonl_comparison': {
            'records': len(legacy_frames),
            'matched_to_raw_scan_by_rounded_header_stamp': len(matched_legacy),
            'valid_return_count_exact_matches': len(valid_matches),
            'current_projection_outcome_count_exact_matches': len(outcome_matches),
            'outcome_comparison': 'counts UNIQUE/AMBIGUOUS/NONE only; OPEN is a separate edge-vote tally',
            'interpretation': 'Matching header stamp and finite-range count confirms same raw scan records; differing outcomes indicate odometry timing or code-version projection differences.',
        },
        'method_limits': [
            'A hit matches a truth wall when its endpoint is within the reported finite-segment distance; thresholds are descriptive, not calibrated sensor confidence.',
            'Static is defined by local odom speed < configured threshold and absolute yaw rate < configured threshold.',
            'The source-stamp interpolation treats scan as one instant because time_increment and scan_time are zero; within-scan motion distortion cannot be separated.',
            'All geometry depends on the session cell/heading anchor and unresolved physical axis transform.',
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir', type=Path)
    parser.add_argument('--truth', type=Path,
                        default=ROOT / 'field' / 'maze_truth_7x7.json')
    parser.add_argument('--static-speed-mps', type=float, default=.03)
    parser.add_argument('--static-yaw-rate-radps', type=float, default=.05)
    args = parser.parse_args()
    truth = json.loads(args.truth.read_text(encoding='utf-8'))
    report = audit_session(args.session_dir, truth,
                           static_speed_mps=args.static_speed_mps,
                           static_yaw_rate_radps=args.static_yaw_rate_radps)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
