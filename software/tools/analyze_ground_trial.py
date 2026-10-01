#!/usr/bin/env python3
"""Summarize one or more control_probe --ground-odom-straight CSV runs."""

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

DEFAULT_FRESHNESS_LIMIT_S = 0.5


def _number(row, key):
    value = row.get(key, '')
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _bool(value):
    return str(value).strip().lower() in {'1', 'true', 'yes'}


def _start_config(reason):
    return {key: value for key, value in re.findall(
        r'([a-z_]+)=([^;]+)', reason or '')}


def analyze_csv(path, freshness_limit_s=DEFAULT_FRESHNESS_LIMIT_S):
    """Return run metrics using the existing control_probe CSV fields."""
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        rows = list(csv.DictReader(stream))
    starts = [r for r in rows if r.get('record_type') == 'ground_trial_start']
    samples = [r for r in rows if r.get('record_type') == 'ground_trial_sample']
    stops = [r for r in rows if r.get('record_type') == 'ground_trial_stop']
    if not starts:
        raise ValueError(f'{path}: no ground_trial_start row')
    if not samples:
        raise ValueError(f'{path}: no ground_trial_sample rows')

    start, last = starts[0], samples[-1]
    config = _start_config(start.get('reason', ''))
    target = _number(start, 'ref_x_m')
    start_x = _number(start, 'pose_x_m')
    target_y = _number(start, 'ref_y_m')
    start_y = _number(start, 'pose_y_m')
    distance = _number(start, 'distance') or _number(start, 'target_distance_m')
    if distance is None and 'distance' in config:
        try:
            distance = float(config['distance'])
        except ValueError:
            distance = None
    if distance is None:
        distance = math.hypot((target or 0) - (start_x or 0),
                              (target_y or 0) - (start_y or 0))
    else:
        distance = abs(distance)

    progress_values = [_number(r, 'odom_forward_path_m') for r in samples]
    progress_values = [v for v in progress_values if v is not None]
    # Older/partial logs can lack path fields; derive signed along-track from poses.
    if not progress_values:
        dx, dy = (target or start_x or 0) - (start_x or 0), (target_y or start_y or 0) - (start_y or 0)
        norm = math.hypot(dx, dy)
        ux, uy = ((dx / norm, dy / norm) if norm else (1.0, 0.0))
        progress_values = []
        for row in samples:
            x, y = _number(row, 'pose_x_m'), _number(row, 'pose_y_m')
            if x is not None and y is not None and start_x is not None and start_y is not None:
                progress_values.append((x - start_x) * ux + (y - start_y) * uy)

    overshoot = max((max(0.0, p - distance) for p in progress_values), default=None)
    cross_values = [_number(r, 'odom_lateral_path_m') for r in samples]
    cross_values = [abs(v) for v in cross_values if v is not None]
    yaw_values = [_number(r, 'yaw_err_rad') for r in samples]
    yaw_values = [abs(v) for v in yaw_values if v is not None]

    receive_times = [_number(r, 'received_monotonic_s') for r in samples]
    receive_times = [v for v in receive_times if v is not None]
    receive_gaps = [b - a for a, b in zip(receive_times, receive_times[1:]) if b >= a]
    source_stamps = [_number(r, 'source_stamp_s') for r in samples]
    source_stamps = [v for v in source_stamps if v is not None]
    source_gaps = [b - a for a, b in zip(source_stamps, source_stamps[1:]) if b > a]
    max_source_gap = max(source_gaps, default=None)
    paired_stamps = [(_number(r, 'source_stamp_s'), _number(r, 'received_monotonic_s'))
                     for r in samples]
    paired_stamps = [(stamp, received) for stamp, received in paired_stamps
                     if stamp is not None and received is not None]
    hold_durations = []
    hold_stamp = hold_start = hold_last = None
    for stamp, received in paired_stamps:
        if stamp != hold_stamp:
            if hold_stamp is not None:
                hold_durations.append(max(0.0, hold_last - hold_start))
            hold_stamp, hold_start, hold_last = stamp, received, received
        else:
            hold_last = received
    if hold_stamp is not None:
        hold_durations.append(max(0.0, hold_last - hold_start))
    max_stamp_hold = max(hold_durations, default=None)
    # This is a cadence/stall indicator, not a direct receipt-age measurement:
    # the CSV does not record the odometry callback receipt timestamp per sample.
    freshness_ok = (max_source_gap is not None and
                    max_source_gap <= freshness_limit_s and
                    max_stamp_hold is not None and
                    max_stamp_hold <= freshness_limit_s and
                    (not receive_gaps or max(receive_gaps) <= freshness_limit_s))

    stop = stops[-1] if stops else {}
    stop_x, stop_y = _number(stop, 'pose_x_m'), _number(stop, 'pose_y_m')
    target_x, target_y = _number(start, 'ref_x_m'), _number(start, 'ref_y_m')
    if None not in (stop_x, stop_y, target_x, target_y):
        final_error = math.hypot(stop_x - target_x, stop_y - target_y)
        final_error_source = 'ground_trial_stop pose'
    else:
        final_error = _number(last, 'pos_err_m')
        final_error_source = 'last ground_trial_sample pos_err_m'
    command_integrals = [_number(r, 'command_integral_m') for r in rows]
    command_integrals = [v for v in command_integrals if v is not None]
    result = {
        'file': str(path),
        'target_distance_m': distance,
        'max_forward_overshoot_m': overshoot,
        'final_position_error_m': final_error,
        'final_position_error_source': final_error_source,
        'max_cross_track_m': max(cross_values, default=None),
        'max_yaw_error_rad': max(yaw_values, default=None),
        'settled': (bool(stops) and stop.get('reason') == 'settled' and
                    any(_bool(r.get('settled', '')) for r in samples)),
        'stop_reason': stop.get('reason') or 'missing ground_trial_stop row',
        'command_integral_m': command_integrals[-1] if command_integrals else None,
        'sample_count': len(samples),
        'max_sample_receive_gap_s': max(receive_gaps, default=None),
        'max_source_stamp_advance_gap_s': max_source_gap,
        'max_source_stamp_hold_s': max_stamp_hold,
        'source_stamp_duplicate_or_stalled_samples': max(0, len(source_stamps) - len(source_gaps) - 1),
        'odom_freshness_cadence_ok': freshness_ok,
        'freshness_limit_s': freshness_limit_s,
        'parameters': config,
    }
    return result


def _format(value, digits=4):
    return 'n/a' if value is None else f'{value:.{digits}f}'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', type=Path, help='one or more control_probe CSV files')
    parser.add_argument('--freshness-limit', type=float, default=DEFAULT_FRESHNESS_LIMIT_S,
                        help='maximum allowed gap between advancing odometry source stamps (default: 0.5 s)')
    parser.add_argument('--json', action='store_true', help='print machine-readable JSON')
    args = parser.parse_args(argv)
    if not math.isfinite(args.freshness_limit) or args.freshness_limit <= 0:
        parser.error('--freshness-limit must be positive and finite')
    try:
        results = [analyze_csv(path, args.freshness_limit) for path in args.csv]
    except (OSError, ValueError, csv.Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(results, indent=2))
        return 0
    print('file | target m | overshoot m | final error m | cross-track m | yaw rad | settled | stop reason | command integral m | odom cadence')
    for r in results:
        print(' | '.join((Path(r['file']).name, _format(r['target_distance_m']),
            _format(r['max_forward_overshoot_m']), _format(r['final_position_error_m']),
            _format(r['max_cross_track_m']), _format(r['max_yaw_error_rad']),
            str(r['settled']), str(r['stop_reason']), _format(r['command_integral_m']),
            ('ok' if r['odom_freshness_cadence_ok'] else 'check'))))
        print(f"  samples={r['sample_count']}, max receive gap={_format(r['max_sample_receive_gap_s'])} s, max advancing source-stamp gap={_format(r['max_source_stamp_advance_gap_s'])} s, max same-stamp hold={_format(r['max_source_stamp_hold_s'])} s, stalled/duplicate stamps={r['source_stamp_duplicate_or_stalled_samples']}, params={r['parameters']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
