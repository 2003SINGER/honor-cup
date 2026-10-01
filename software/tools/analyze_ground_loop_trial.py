#!/usr/bin/env python3
"""Compare fixed-template ground_loop_trial CSVs without modifying inputs."""

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path


def number(row, key):
    try:
        value = float(row.get(key, ''))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_parameters(reason):
    params = {}
    for key, value in re.findall(r'([A-Za-z][A-Za-z0-9_]*)=([^;]+)', reason or ''):
        value = value.strip()
        try:
            numeric = float(value)
            params[key] = numeric if math.isfinite(numeric) else value
        except ValueError:
            params[key] = value
    return params


def _positions_in_start_frame(rows, start_x, start_y, start_yaw):
    c, s = math.cos(start_yaw), math.sin(start_yaw)
    points = []
    for row in rows:
        x, y = number(row, 'pose_x_m'), number(row, 'pose_y_m')
        if x is None or y is None:
            continue
        dx, dy = x - start_x, y - start_y
        # Grid north is body-forward; positive left is counter-clockwise.
        forward = c * dx + s * dy
        left = -s * dx + c * dy
        points.append((number(row, 'monotonic_s'), forward, left))
    return points


def _rate_summary(rows, stamp_key):
    stamps = [number(row, stamp_key) for row in rows]
    stamps = [stamp for stamp in stamps if stamp is not None]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    positive = [gap for gap in gaps if gap > 0]
    duplicates = sum(gap == 0 for gap in gaps)
    median_gap = sorted(positive)[len(positive) // 2] if positive else None
    if median_gap is not None and len(positive) % 2 == 0:
        ordered = sorted(positive)
        median_gap = (ordered[len(ordered)//2 - 1] + ordered[len(ordered)//2]) / 2
    elapsed = stamps[-1] - stamps[0] if len(stamps) > 1 else None
    return {
        'sample_count': len(stamps),
        'median_hz': (1.0 / median_gap if median_gap and median_gap > 0 else None),
        'mean_hz': ((len(stamps) - 1) / elapsed
                    if elapsed is not None and elapsed > 0 else None),
        'median_gap_s': median_gap,
        'max_gap_s': max(positive) if positive else None,
        'nonadvancing_intervals': duplicates + sum(gap < 0 for gap in gaps),
    }


def _hold_duration(rows, phase):
    starts = [number(row, 'monotonic_s') for row in rows
              if row.get('record_type') == 'phase_hold_start'
              and row.get('phase') == phase]
    completes = [number(row, 'monotonic_s') for row in rows
                 if row.get('record_type') == 'phase_hold_complete'
                 and row.get('phase') == phase]
    starts = [value for value in starts if value is not None]
    completes = [value for value in completes if value is not None]
    if not starts or not completes:
        return None
    duration = completes[0] - starts[0]
    return duration if duration >= 0 else None


def _phase_points(points, sample_rows, all_rows, phase):
    """Select samples by explicit phase, falling back to logged event times."""
    explicit = [point for point, row in zip(points, sample_rows)
                if row.get('phase') == phase]
    if explicit:
        return explicit
    start_rows = [row for row in all_rows if row.get('record_type') == 'trial_start']
    outbound_complete = [number(row, 'monotonic_s') for row in all_rows
        if row.get('record_type') == 'phase_hold_complete'
        and row.get('phase') == 'outbound']
    outbound_hold = [number(row, 'monotonic_s') for row in all_rows
        if row.get('record_type') == 'phase_hold_start'
        and row.get('phase') == 'outbound']
    stop_rows = [row for row in all_rows if row.get('record_type') == 'trial_stop']
    start_t = number(start_rows[0], 'monotonic_s') if start_rows else None
    complete_t = next((value for value in outbound_complete if value is not None), None)
    hold_t = next((value for value in outbound_hold if value is not None), None)
    stop_t = number(stop_rows[-1], 'monotonic_s') if stop_rows else None
    # For outbound peak, retain samples through midpoint settle. If it never
    # completed, use the first hold-entry time as the only known boundary.
    if phase == 'outbound':
        end_t = complete_t if complete_t is not None else hold_t
        return [point for point in points if point[0] is not None
                and (start_t is None or point[0] >= start_t)
                and (end_t is None or point[0] <= end_t)]
    if phase == 'return' and complete_t is not None:
        return [point for point in points if point[0] is not None
                and point[0] >= complete_t
                and (stop_t is None or point[0] <= stop_t)]
    return []


def analyze_csv(path):
    """Compute route geometry and actual per-callback cadence for one run."""
    path = Path(path)
    with path.open(newline='', encoding='utf-8-sig') as stream:
        rows = list(csv.DictReader(stream))
    starts = [row for row in rows if row.get('record_type') == 'trial_start']
    if not starts:
        raise ValueError(f'{path}: missing trial_start record')
    start = starts[0]
    start_x = number(start, 'pose_x_m')
    start_y = number(start, 'pose_y_m')
    start_yaw = number(start, 'pose_yaw_rad')
    if None in (start_x, start_y, start_yaw):
        raise ValueError(f'{path}: trial_start is missing a finite pose/yaw')

    odom_rows = [row for row in rows if row.get('record_type') == 'odom_sample']
    sample_rows = odom_rows if odom_rows else [
        row for row in rows if row.get('record_type') == 'control_sample']
    positions = _positions_in_start_frame(sample_rows, start_x, start_y, start_yaw)
    # Keep row/point alignment after rows lacking valid pose were discarded.
    valid_sample_rows = [row for row in sample_rows
                         if number(row, 'pose_x_m') is not None
                         and number(row, 'pose_y_m') is not None]
    outbound = _phase_points(positions, valid_sample_rows, rows, 'outbound')
    returning = _phase_points(positions, valid_sample_rows, rows, 'return')

    # The fixed template midpoint is one cell left and one cell forward from
    # the start; these axes are expressed in the actual trial_start body frame.
    midpoint_left_m = 0.4
    midpoint_forward_m = 0.4
    peak_mid_left = max((point[2] for point in outbound), default=None)
    peak_mid_forward = max((point[1] for point in outbound), default=None)
    peak_return_right = max((-point[2] for point in returning), default=None)
    peak_return_back = max((-point[1] for point in returning), default=None)
    midpoint_left_overrun = (max(0.0, peak_mid_left - midpoint_left_m)
                             if peak_mid_left is not None else None)
    midpoint_forward_overrun = (max(0.0, peak_mid_forward - midpoint_forward_m)
                                if peak_mid_forward is not None else None)
    return_right_overrun = max(0.0, peak_return_right) if peak_return_right is not None else None
    return_back_overrun = max(0.0, peak_return_back) if peak_return_back is not None else None

    stop_rows = [row for row in rows if row.get('record_type') == 'trial_stop']
    terminal = stop_rows[-1] if stop_rows else (sample_rows[-1] if sample_rows else {})
    terminal_x = number(terminal, 'pose_x_m')
    terminal_y = number(terminal, 'pose_y_m')
    terminal_yaw = number(terminal, 'pose_yaw_rad')
    terminal_pose = None
    if None not in (terminal_x, terminal_y, terminal_yaw):
        dx, dy = terminal_x - start_x, terminal_y - start_y
        terminal_pose = {
            'world_x_m': terminal_x,
            'world_y_m': terminal_y,
            'relative_forward_m': math.cos(start_yaw) * dx + math.sin(start_yaw) * dy,
            'relative_left_m': -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy,
            'position_error_m': math.hypot(dx, dy),
            'relative_yaw_rad': wrapped_angle(terminal_yaw - start_yaw),
        }

    if odom_rows:
        source_cadence = _rate_summary(odom_rows, 'source_stamp_s')
        receipt_cadence = _rate_summary(odom_rows, 'receipt_monotonic_s')
        cadence_source = 'odom_sample callbacks'
    else:
        source_cadence = receipt_cadence = None
        cadence_source = 'unavailable: CSV has no odom_sample callback records'
    stop = stop_rows[-1] if stop_rows else {}
    return {
        'file': str(path),
        'start_pose_world': {'x_m': start_x, 'y_m': start_y, 'yaw_rad': start_yaw},
        'midpoint_target_in_start_body': {
            'left_m': midpoint_left_m, 'forward_m': midpoint_forward_m},
        'outbound_peak_in_start_body': {
            'left_m': peak_mid_left, 'forward_m': peak_mid_forward},
        'outbound_overrun': {
            'left_m': midpoint_left_overrun, 'forward_m': midpoint_forward_overrun},
        'return_peak_overrun': {
            'right_m': return_right_overrun, 'back_m': return_back_overrun},
        'terminal_pose': terminal_pose,
        'hold_duration_s': {
            'outbound': _hold_duration(rows, 'outbound'),
            'return': _hold_duration(rows, 'return')},
        'odom_callback_cadence_source': cadence_source,
        'odom_source_stamp_cadence': source_cadence,
        'odom_receipt_monotonic_cadence': receipt_cadence,
        'odom_callback_count': len(odom_rows) if odom_rows else None,
        'stop_reason': stop.get('reason') or 'missing trial_stop record',
        'parameters': parse_parameters(start.get('reason', '')),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', type=Path,
                        help='one or more ground_loop_trial CSV files')
    parser.add_argument('--json', action='store_true',
                        help='print machine-readable JSON')
    args = parser.parse_args(argv)
    try:
        reports = [analyze_csv(path) for path in args.csv]
    except (OSError, ValueError, csv.Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(reports, indent=2, allow_nan=False))
        return 0
    print('file | midpoint overrun L/F (cm) | return overrun R/B (cm) | endpoint error/yaw | hold out/back (s) | odom source/receipt Hz | stop')
    for report in reports:
        mid = report['outbound_overrun']
        ret = report['return_peak_overrun']
        terminal = report['terminal_pose'] or {}
        hold = report['hold_duration_s']
        source = report['odom_source_stamp_cadence']
        receipt = report['odom_receipt_monotonic_cadence']
        if source is None:
            cadence = 'unavailable'
        else:
            cadence = f"{_fmt(source['median_hz'])}/{_fmt(receipt['median_hz'] if receipt else None)}"
        print(' | '.join((Path(report['file']).name,
            f"{_cm(mid['left_m'])}/{_cm(mid['forward_m'])}",
            f"{_cm(ret['right_m'])}/{_cm(ret['back_m'])}",
            f"{_cm(terminal.get('position_error_m'))}/{_fmt(terminal.get('relative_yaw_rad'))}",
            f"{_fmt(hold['outbound'])}/{_fmt(hold['return'])}",
            cadence, report['stop_reason'])))
        print(f"  odom rows={report['odom_callback_count']}; cadence basis={report['odom_callback_cadence_source']}; parameters={report['parameters']}")
    return 0


def _fmt(value):
    return 'n/a' if value is None else f'{value:.3f}'


def _cm(value):
    return 'n/a' if value is None else f'{100.0 * value:.1f}'


if __name__ == '__main__':
    raise SystemExit(main())
