#!/usr/bin/env python3
"""Read-only rosbag2 audit of scan/odom geometry and wall associations.

Uses the standard-library CDR readers in analyze_scan_edge.py and the live
GridAssociation/pose_correction implementations. The bag and field data are
opened read-only. Coordinates assume session.yaml's manual cell/heading anchor
and the ideal 0.4 m maze grid; outputs are diagnostic, never applied online.

Example:
  python3 software/tools/audit_lidar_odom_calibration.py field_data/20260930_212853_junction_NE_open_SW_wall
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2' / 'm3pro_nav'))

from analyze_scan_edge import (CELL_SIZE_M, HEADING_RAD, Pose2D, Transform2D,
                               compose, decode_laser_scan, decode_odometry,
                               decode_tf_message, interpolate_pose, inverse, load_session_value,
                               quantile, static_transform)

from m3pro_nav.frame_projector import WorldRay
from m3pro_nav.grid_association import (AMBIGUOUS, NONE, UNIQUE,
                                        GridAssociation, edge_id_for_line)
from m3pro_nav.pose_correction import (KnownWallSegment,
                                       ProjectedEndpoint,
                                       propose_pose_correction)


def edge_truth(truth: dict) -> dict[tuple, bool]:
    """Canonical edge key -> is wall, derived from supplied topology."""
    opened = truth['open_directions_by_cell']
    result = {}
    n = truth['grid']['width']
    for y, row in enumerate(opened):
        for x, directions in enumerate(row):
            for d, orient, k, j in (
                    ('N', 'H', y + 1, x), ('S', 'H', y, x),
                    ('E', 'V', x + 1, y), ('W', 'V', x, y)):
                key = edge_id_for_line(orient, k, j)
                value = d not in directions
                if key in result and result[key] != value:
                    raise ValueError(f'inconsistent truth for edge {key}')
                result[key] = value
    if len(result) != 2 * n * (n + 1):
        raise ValueError(f'unexpected canonical truth edge count: {len(result)}')
    return result


def scan_frame(scan: dict, pose: Pose2D, anchor: Transform2D,
               base_from_laser: Transform2D) -> tuple[Pose2D, list[WorldRay], list[ProjectedEndpoint]]:
    """Project raw scan rays through odom and static TF into assumed maze frame."""
    maze_from_laser = compose(anchor, compose(
        Transform2D(pose.x, pose.y, pose.yaw), base_from_laser))
    c, s = math.cos(maze_from_laser.yaw), math.sin(maze_from_laser.yaw)
    rays, points = [], []
    for i, distance in enumerate(scan['ranges']):
        angle = scan['angle_min'] + i * scan['angle_increment']
        ca, sa = math.cos(angle), math.sin(angle)
        dx, dy = c * ca - s * sa, s * ca + c * sa
        valid = (math.isfinite(distance) and
                 scan['range_min'] <= distance <= scan['range_max'])
        if valid:
            hx = maze_from_laser.x + distance * dx
            hy = maze_from_laser.y + distance * dy
            rays.append(WorldRay(i, maze_from_laser.x, maze_from_laser.y,
                                 dx, dy, hx, hy, distance, None))
            points.append(ProjectedEndpoint(hx, hy, True))
        else:
            reason = 'INVALID_RANGE' if not math.isfinite(distance) else 'OUT_OF_RANGE'
            rays.append(WorldRay(i, maze_from_laser.x, maze_from_laser.y,
                                 dx, dy, math.nan, math.nan, distance, reason))
    maze_pose = compose(anchor, Transform2D(pose.x, pose.y, pose.yaw))
    return Pose2D(maze_pose.x, maze_pose.y, maze_pose.yaw), rays, points


def local_confirmed_walls(cell: tuple[int, int]) -> tuple[KnownWallSegment, ...]:
    """Documented local walked-truth walls for the (1,2) junction capture."""
    if cell != (1, 2):
        return ()
    # Session-specific walked-path notes in docs/reports/2026-09-30-lidar-static-calibration.md:
    # (1,2) S/W walls and the dead-end north wall at (1,4). Although the
    # supplied topology marks (0,4) N as a wall, its physical segment mapping
    # is ambiguous in prior notes, so it is deliberately excluded.
    return (
        KnownWallSegment((0.4, 0.8), (0.8, 0.8), True, 'cell_1_2_S'),
        KnownWallSegment((0.4, 0.8), (0.4, 1.2), True, 'cell_1_2_W'),
        KnownWallSegment((0.4, 2.0), (0.8, 2.0), True, 'cell_1_4_N'),
    )


def evaluate_wall_residuals(endpoints: list[ProjectedEndpoint], pose: Pose2D,
                            walls: tuple[KnownWallSegment, ...],
                            delta: tuple[float, float, float]
                            ) -> list[float]:
    """Residuals on frozen nearest-wall matches, used for temporal holdout."""
    pairs = []
    for point in endpoints:
        candidates = []
        for wall in walls:
            ax, ay = wall.start
            vx, vy = wall.end[0] - ax, wall.end[1] - ay
            length = math.hypot(vx, vy)
            tx, ty = vx / length, vy / length
            nx, ny = -ty, tx
            along = (point.x - ax) * tx + (point.y - ay) * ty
            residual = (point.x - ax) * nx + (point.y - ay) * ny
            if -0.12 <= along <= length + 0.12 and abs(residual) <= 0.12:
                candidates.append((abs(residual), wall, tx, ty, nx, ny))
        candidates.sort(key=lambda row: row[0])
        if (not candidates or (len(candidates) > 1 and
                candidates[1][0] - candidates[0][0] < 0.005)):
            continue
        _, wall, tx, ty, nx, ny = candidates[0]
        dx, dy, dyaw = delta
        rx, ry = point.x - pose.x, point.y - pose.y
        c, s = math.cos(dyaw), math.sin(dyaw)
        qx = pose.x + dx + c * rx - s * ry
        qy = pose.y + dy + s * rx + c * ry
        ax, ay = wall.start
        along = (qx - ax) * tx + (qy - ay) * ty
        length = math.hypot(wall.end[0] - ax, wall.end[1] - ay)
        if -0.12 <= along <= length + 0.12:
            residual = (qx - ax) * nx + (qy - ay) * ny
            pairs.append(residual)
    return pairs


def residual_summary(values: list[float]) -> dict:
    absolute = [abs(v) for v in values]
    return {'n': len(absolute), 'abs_m_p50_p90_p95': [
        quantile(absolute, q) for q in (.5, .9, .95)],
        'within_35mm_fraction': (sum(v <= .035 for v in absolute) / len(absolute)
                                 if absolute else None)}


def process_session(session: Path, truths: dict[tuple, bool],
                    external_correction: tuple[float, float, float] | None = None,
                    correction_source: str | None = None) -> dict:
    meta = session / 'session.yaml'
    cell_text = load_session_value(meta, 'cell').strip('[]').split(',')
    cell = tuple(int(v.strip()) for v in cell_text)
    heading = load_session_value(meta, 'heading')
    if heading not in HEADING_RAD:
        raise ValueError(f'unsupported heading: {heading}')
    scan_topic = load_session_value(meta, 'scan_topic')
    odom_topic = load_session_value(meta, 'odom_topic')
    laser_frame = load_session_value(meta, 'laser_frame')
    odom_frame = load_session_value(meta, 'odom_frame')
    base_frame = load_session_value(meta, 'base_frame')
    bag_files = sorted((session / 'bag').glob('**/*.db3'))
    if len(bag_files) != 1:
        raise ValueError(f'{session}: expected one db3 bag, found {len(bag_files)}')

    db = sqlite3.connect(f'file:{bag_files[0]}?mode=ro', uri=True)
    topics = {name: (tid, typ) for tid, name, typ in
              db.execute('SELECT id,name,type FROM topics')}
    for required in (scan_topic, odom_topic, '/tf_static'):
        if required not in topics:
            raise ValueError(f'{session}: missing topic {required}')
    odoms = []
    odom_arrival = []
    for (bag_stamp, blob) in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[odom_topic][0],)):
        stamp, frame, child, pose = decode_odometry(blob)
        if (frame, child) != (odom_frame, base_frame):
            raise ValueError(f'{session}: odometry frames {frame}/{child}, expected {odom_frame}/{base_frame}')
        odoms.append((stamp, pose))
        odom_arrival.append((bag_stamp * 1e-9, stamp, pose))
    odoms.sort(key=lambda item: item[0])
    if not odoms:
        raise ValueError(f'{session}: no odometry')

    tf_edges = []
    for (_bag_stamp, blob) in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics['/tf_static'][0],)):
        tf_edges.extend(decode_tf_message(blob))
    base_from_laser = static_transform(tf_edges, base_frame, laser_frame)
    first = odoms[0][1]
    maze_anchor = Pose2D((cell[0] + 0.5) * CELL_SIZE_M,
                         (cell[1] + 0.5) * CELL_SIZE_M, HEADING_RAD[heading])
    maze_from_odom = compose(Transform2D(maze_anchor.x, maze_anchor.y,
                                         maze_anchor.yaw),
                             inverse(Transform2D(first.x, first.y, first.yaw)))

    association = GridAssociation()
    outcomes = Counter()
    wall_correct = wall_false = open_correct = open_false = 0
    unique_residuals, ambiguous_margins = [], []
    line_points: dict[tuple, list[tuple[float, float]]] = {}
    corr_reasons = Counter()
    corr_values = []
    holdout_frames = []
    cross_baseline, cross_adjusted = [], []
    walls = local_confirmed_walls(cell)
    n_scans = skipped = valid_returns = 0
    latest_odom_skews = []
    latest_odom_pos_error = []
    latest_odom_yaw_error = []
    arrival_ns = [x[0] for x in odom_arrival]
    for bag_stamp, blob in db.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[scan_topic][0],)):
        scan = decode_laser_scan(blob)
        if scan['frame_id'] != laser_frame:
            raise ValueError(f'{session}: scan frame {scan["frame_id"]}, expected {laser_frame}')
        try:
            odom_pose = interpolate_pose(odoms, scan['stamp'])
        except ValueError:
            skipped += 1
            continue
        maze_pose, rays, endpoints = scan_frame(
            scan, odom_pose, maze_from_odom, base_from_laser)
        n_scans += 1
        scan_arrival = bag_stamp * 1e-9
        oi = bisect.bisect_right(arrival_ns, scan_arrival) - 1
        if oi >= 0:
            _arrival, last_stamp, last_pose = odom_arrival[oi]
            skew = scan['stamp'] - last_stamp
            latest_odom_skews.append(skew)
            if len(odom_arrival) > 1:
                left = max(0, min(oi, len(odom_arrival) - 2))
                _a0, t0, p0 = odom_arrival[left]
                _a1, t1, p1 = odom_arrival[left + 1]
                dt = t1 - t0
                if dt > 0:
                    speed = math.hypot(p1.x - p0.x, p1.y - p0.y) / dt
                    dyaw = math.atan2(math.sin(p1.yaw - p0.yaw),
                                      math.cos(p1.yaw - p0.yaw))
                    yaw_rate = abs(dyaw) / dt
                    latest_odom_pos_error.append(speed * abs(skew))
                    latest_odom_yaw_error.append(yaw_rate * abs(skew))
        valid_returns += len(endpoints)
        observations = association.process(rays)
        unique_by_wall = {}
        for obs in observations:
            outcomes[obs.outcome] += 1
            if obs.outcome == UNIQUE and obs.candidate:
                edge = obs.candidate.edge_id
                unique_residuals.append(obs.candidate.residual)
                is_wall = truths.get(edge)
                if is_wall is True:
                    wall_correct += 1
                elif is_wall is False:
                    wall_false += 1
                unique_by_wall[edge] = obs.candidate.residual
                along = obs.candidate.along_edge
                line_points.setdefault(edge, []).append((along, obs.candidate.residual))
            elif obs.outcome == AMBIGUOUS and len(obs.candidates) > 1:
                ambiguous_margins.append(obs.candidates[1].residual - obs.candidates[0].residual)
            for edge in obs.open_edges:
                is_wall = truths.get(edge)
                if is_wall is False:
                    open_correct += 1
                elif is_wall is True:
                    open_false += 1

        if walls:
            if external_correction is not None:
                cross_baseline.extend(evaluate_wall_residuals(
                    endpoints, maze_pose, walls, (0.0, 0.0, 0.0)))
                cross_adjusted.extend(evaluate_wall_residuals(
                    endpoints, maze_pose, walls, external_correction))
            if n_scans % 2 == 0:
                result = propose_pose_correction(endpoints, walls, maze_pose)
                corr_reasons[result.reason] += 1
                if result.accepted:
                    corr_values.append([result.dx, result.dy, result.dyaw,
                                        result.median_residual_m,
                                        result.p90_residual_m, result.n_inliers,
                                        result.n_walls])
            else:
                holdout_frames.append((maze_pose, endpoints))
    db.close()

    fit_residuals = []
    for edge, pts in line_points.items():
        if len(pts) >= 3:
            # Residuals to the canonical expected edge line, grouped by edge.
            fit_residuals.extend(abs(v) for _, v in pts)
    accepted_delta_stats = {}
    if corr_values:
        for i, name in enumerate(('dx_m', 'dy_m', 'dyaw_rad', 'median_m',
                                  'p90_m', 'inliers', 'walls')):
            vals = sorted(row[i] for row in corr_values)
            accepted_delta_stats[name] = [quantile(vals, q) for q in (.1, .5, .9)]
    holdout = {'split': 'even-numbered scans fit per-frame proposal; odd-numbered scans evaluate shared median fit',
               'fit_frames': len(corr_values), 'evaluation_frames': len(holdout_frames)}
    if corr_values and holdout_frames:
        delta = tuple(quantile(sorted(row[i] for row in corr_values), .5)
                      for i in range(3))
        baseline, adjusted = [], []
        for pose, endpoints in holdout_frames:
            baseline.extend(evaluate_wall_residuals(endpoints, pose, walls,
                                                    (0.0, 0.0, 0.0)))
            adjusted.extend(evaluate_wall_residuals(endpoints, pose, walls,
                                                    delta))
        holdout['shared_median_delta'] = list(delta)
        holdout['baseline'] = residual_summary(baseline)
        holdout['after_proposal'] = residual_summary(adjusted)
    cross_session = None
    if external_correction is not None:
        cross_session = {
            'source_session': correction_source,
            'shared_delta': list(external_correction),
            'assumption': 'source and target session used the same physical base pose, anchor, maze frame, and extrinsic',
            'baseline': residual_summary(cross_baseline),
            'after_source_proposal': residual_summary(cross_adjusted),
        }
    odom_x = [pose.x for _, pose in odoms]
    odom_y = [pose.y for _, pose in odoms]
    yaw_deltas = [math.atan2(math.sin(pose.yaw - odoms[0][1].yaw),
                             math.cos(pose.yaw - odoms[0][1].yaw))
                  for _, pose in odoms]
    path_length = sum(math.hypot(b[1].x - a[1].x, b[1].y - a[1].y)
                      for a, b in zip(odoms, odoms[1:]))
    skew_abs = [abs(v) for v in latest_odom_skews]
    known_assoc = wall_correct + wall_false
    known_open = open_correct + open_false
    return {
        'session': session.name,
        'anchor_assumption': {'cell': list(cell), 'heading': heading,
                              'cell_size_m': CELL_SIZE_M},
        'frames': {'scans_with_pose': n_scans, 'skipped_pose': skipped,
                   'odom_samples': len(odoms), 'valid_returns': valid_returns},
        'odometry_consistency': {
            'odom_path_length_m': path_length,
            'odom_end_minus_start_m': [odom_x[-1] - odom_x[0],
                                       odom_y[-1] - odom_y[0]],
            'odom_max_displacement_from_start_m': max(
                math.hypot(x - odom_x[0], y - odom_y[0])
                for x, y in zip(odom_x, odom_y)),
            'odom_yaw_delta_rad_p05_p50_p95': [
                quantile(yaw_deltas, q) for q in (.05, .5, .95)],
            'scan_stamp_minus_latest_arrived_odom_stamp_s': {
                'n': len(latest_odom_skews),
                'signed_p05_p50_p95': [
                    quantile(sorted(latest_odom_skews), q) for q in (.05, .5, .95)],
                'abs_p50_p90_p95_max': [
                    quantile(sorted(skew_abs), q) for q in (.5, .9, .95)] +
                    [max(skew_abs) if skew_abs else None],
                'abs_skew_exceedance_counts_s': {
                    str(threshold): sum(v > threshold for v in skew_abs)
                    for threshold in (.02, .05, .10, .20)},
                'estimated_position_error_m_p50_p90_p95_max': [
                    quantile(sorted(latest_odom_pos_error), q)
                    for q in (.5, .9, .95)] +
                    [max(latest_odom_pos_error)
                     if latest_odom_pos_error else None],
                'estimated_yaw_error_rad_p50_p90_p95_max': [
                    quantile(sorted(latest_odom_yaw_error), q)
                    for q in (.5, .9, .95)] +
                    [max(latest_odom_yaw_error)
                     if latest_odom_yaw_error else None],
                'method': 'approximate bag arrival order; latest odom sample available by scan bag timestamp; error estimate uses adjacent odometry finite differences',
            },
        },
        'tf': {'base_frame': base_frame, 'laser_frame': laser_frame,
               'base_from_laser': [base_from_laser.x, base_from_laser.y,
                                   base_from_laser.yaw]},
        'association': {
            'outcomes': dict(outcomes),
            'unique_hit_votes_against_supplied_topology': {
                'wall_correct': wall_correct, 'hit_on_topology_open': wall_false,
                'n': known_assoc,
                'precision_if_topology_alignment_is_correct':
                    wall_correct / known_assoc if known_assoc else None},
            'open_votes_against_supplied_topology': {
                'open_correct': open_correct, 'open_on_topology_wall': open_false,
                'n': known_open,
                'precision_if_topology_alignment_is_correct':
                    open_correct / known_open if known_open else None},
            'unique_residual_m_p50_p90_p95': [
                quantile(unique_residuals, q) for q in (.5, .9, .95)],
            'ambiguous_margin_m_p50_p90': [
                quantile(ambiguous_margins, q) for q in (.5, .9)],
        },
        'canonical_line_residuals': {
            'point_count_on_edges_with_3plus_hits': len(fit_residuals),
            'absolute_m_p50_p90_p95': [
                quantile(fit_residuals, q) for q in (.5, .9, .95)],
            'edge_counts_top10': [
                {'edge': repr(k), 'n': len(v)}
                for k, v in sorted(line_points.items(), key=lambda kv: -len(kv[1]))[:10]],
        },
        'pose_correction': {
            'wall_source': 'walk-confirmed local walls; only provided for session cell (1,2)',
            'proposal_rejections': dict(corr_reasons),
            'accepted_candidate_count': len(corr_values),
            'candidate_values_p10_p50_p90': accepted_delta_stats,
            'temporal_holdout': holdout,
            'cross_session_evaluation': cross_session,
            'limit': 'diagnostic proposals only; static repeated scans are correlated and wall geometry/anchor remain assumptions',
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dirs', nargs='+', type=Path)
    parser.add_argument('--truth', type=Path,
                        default=Path('field/maze_truth_7x7.json'))
    parser.add_argument('--output', type=Path,
                        help='write JSON report to this path; default stdout')
    parser.add_argument('--cross-evaluate-from', type=Path,
                        help='fit an even-frame shared correction on this listed session and evaluate it on other listed sessions at the same recorded cell/heading')
    args = parser.parse_args()
    truth = json.loads(args.truth.read_text(encoding='utf-8'))
    truths = edge_truth(truth)
    sessions = args.session_dirs
    if args.cross_evaluate_from:
        source = args.cross_evaluate_from
        if source not in sessions:
            parser.error('--cross-evaluate-from must also appear in session_dirs')
        ordered_sessions = [source] + [path for path in sessions if path != source]
    else:
        source, ordered_sessions = None, sessions
    session_reports = []
    source_delta = None
    for path in ordered_sessions:
        # Parse the small anchor fields before deciding whether a shared rigid
        # correction is even applicable to this target capture.
        external = None
        source_name = None
        if source_delta is not None and path != source:
            same_pose_label = (
                load_session_value(path / 'session.yaml', 'cell') ==
                load_session_value(source / 'session.yaml', 'cell') and
                load_session_value(path / 'session.yaml', 'heading') ==
                load_session_value(source / 'session.yaml', 'heading'))
            if same_pose_label:
                external, source_name = source_delta, source.name
        item = process_session(path, truths, external, source_name)
        session_reports.append(item)
        if path == source:
            delta = item['pose_correction']['temporal_holdout'].get(
                'shared_median_delta')
            if delta is not None:
                source_delta = tuple(delta)
    report = {
        'schema_version': 1,
        'method': 'read-only rosbag2 SQLite/CDR replay; session manual anchor; supplied maze topology; ideal axis-aligned 0.4 m grid',
        'limitations': [
            'field/maze_truth_7x7.json leaves physical image transform unresolved; session cell/heading semantics are an assumption',
            'session laser_frame is base_link while base_frame is base_footprint; static TF is applied, but fused /scan_multi calibration provenance is unrecorded',
            'bag frame residuals are repeated correlated scans, not independent confidence samples',
            'pose correction output is a proposal only and is never applied to odometry or control',
        ],
        'cross_session_note': ('Cross-session correction was evaluated only where recorded cell and heading match the source; this checks repeatability at the nominal pose, not independent spatial generalization.' if source else None),
        'sessions': session_reports,
    }
    rendered = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding='utf-8')
    else:
        print(rendered, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
