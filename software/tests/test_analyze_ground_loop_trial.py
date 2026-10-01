import csv
import math

import pytest

from software.tools.analyze_ground_loop_trial import analyze_csv


def _write_synthetic_trial(path, *, with_odom=True):
    fields = ('monotonic_s', 'record_type', 'phase', 'source_stamp_s',
              'receipt_monotonic_s', 'pose_x_m', 'pose_y_m', 'pose_yaw_rad',
              'world_vx_mps', 'world_vy_mps', 'odom_wz_radps', 'reason')
    start_x, start_y, start_yaw = 1.0, 2.0, math.pi / 2
    rows = [
        {'monotonic_s': '0.0', 'record_type': 'trial_start',
         'pose_x_m': str(start_x), 'pose_y_m': str(start_y),
         'pose_yaw_rad': str(start_yaw),
         'reason': 'speed=0.5;kp_pos=1.4;kd_vel=0.2'},
        {'monotonic_s': '0.2', 'record_type': 'phase_hold_start',
         'phase': 'outbound'},
        {'monotonic_s': '0.6', 'record_type': 'phase_hold_complete',
         'phase': 'outbound'},
        {'monotonic_s': '2.4', 'record_type': 'phase_hold_start',
         'phase': 'return'},
        {'monotonic_s': '2.7', 'record_type': 'phase_hold_complete',
         'phase': 'return'},
    ]

    if with_odom:
        for index in range(41):
            t = index * 0.09
            if t <= 0.18:
                ratio = t / 0.18
                left, forward = 0.42 * ratio, 0.43 * ratio
            elif t <= 0.60:
                left, forward = 0.42, 0.43
            elif t <= 1.80:
                ratio = (t - 0.60) / 1.20
                left = 0.42 + (-0.025 - 0.42) * ratio
                forward = 0.43 + (-0.04 - 0.43) * ratio
            elif t <= 2.70:
                ratio = (t - 1.80) / 0.90
                left = -0.025 + 0.015 * ratio
                forward = -0.04 + 0.035 * ratio
            else:
                left, forward = -0.01, -0.005
            # At yaw=+pi/2, body-forward is world +Y and body-left is world -X.
            rows.append({
                'monotonic_s': f'{t:.2f}', 'record_type': 'odom_sample',
                'source_stamp_s': f'{100.0 + index * 0.09:.2f}',
                'receipt_monotonic_s': f'{10.0 + index * 0.09:.2f}',
                'pose_x_m': f'{start_x - left:.8f}',
                'pose_y_m': f'{start_y + forward:.8f}',
                'pose_yaw_rad': f'{start_yaw:.8f}',
                'world_vx_mps': '0.0', 'world_vy_mps': '0.0',
                'odom_wz_radps': '0.0'})
    else:
        rows.extend([
            {'monotonic_s': '0.1', 'record_type': 'control_sample',
             'source_stamp_s': '100.0', 'phase': 'outbound',
             'pose_x_m': str(start_x), 'pose_y_m': str(start_y + 0.2),
             'pose_yaw_rad': str(start_yaw)},
            {'monotonic_s': '0.2', 'record_type': 'control_sample',
             'source_stamp_s': '100.1', 'phase': 'outbound',
             'pose_x_m': str(start_x - 0.1), 'pose_y_m': str(start_y + 0.2),
             'pose_yaw_rad': str(start_yaw)},
        ])

    rows.append({'monotonic_s': '3.6', 'record_type': 'trial_stop',
        'pose_x_m': f'{start_x + 0.01:.8f}',
        'pose_y_m': f'{start_y - 0.005:.8f}',
        'pose_yaw_rad': f'{start_yaw + 0.02:.8f}', 'reason': 'settled'})
    rows.sort(key=lambda row: float(row['monotonic_s']))
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_synthetic_route_checks_body_frame_hold_and_callback_cadence(tmp_path):
    path = tmp_path / 'synthetic.csv'
    _write_synthetic_trial(path)
    report = analyze_csv(path)

    assert report['stop_reason'] == 'settled'
    assert report['outbound_overrun']['left_m'] == pytest.approx(0.02, abs=1e-6)
    assert report['outbound_overrun']['forward_m'] == pytest.approx(0.03, abs=1e-6)
    assert report['return_peak_overrun']['right_m'] == pytest.approx(0.025, abs=1e-6)
    assert report['return_peak_overrun']['back_m'] == pytest.approx(0.04, abs=1e-6)
    assert report['terminal_pose']['position_error_m'] == pytest.approx(math.hypot(.01, .005))
    assert report['terminal_pose']['relative_yaw_rad'] == pytest.approx(0.02, abs=1e-6)
    assert report['hold_duration_s']['outbound'] == pytest.approx(0.4)
    assert report['hold_duration_s']['return'] == pytest.approx(0.3)
    assert report['odom_callback_count'] == 41
    assert report['odom_source_stamp_cadence']['median_hz'] == pytest.approx(1 / .09)
    assert report['odom_receipt_monotonic_cadence']['median_hz'] == pytest.approx(1 / .09)
    assert report['parameters'] == {'speed': 0.5, 'kp_pos': 1.4, 'kd_vel': 0.2}


def test_old_control_sample_csv_marks_odom_callback_rate_unavailable(tmp_path):
    path = tmp_path / 'legacy.csv'
    _write_synthetic_trial(path, with_odom=False)
    # The synthetic legacy control rows above must not be counted as odometry.
    report = analyze_csv(path)
    assert report['odom_callback_count'] is None
    assert report['odom_source_stamp_cadence'] is None
    assert report['odom_receipt_monotonic_cadence'] is None
    assert report['odom_callback_cadence_source'].startswith('unavailable:')

    # A minimal older schema also exercises parser tolerance for missing fields.
    path = tmp_path / 'legacy-minimal.csv'
    fields = ('monotonic_s', 'record_type', 'source_stamp_s', 'phase',
              'pose_x_m', 'pose_y_m', 'pose_yaw_rad', 'reason')
    rows = [
        {'monotonic_s': '1.0', 'record_type': 'trial_start',
         'pose_x_m': '2', 'pose_y_m': '3', 'pose_yaw_rad': '1.57079632679',
         'reason': 'speed=0.5;kp_pos=1.0'},
        {'monotonic_s': '1.1', 'record_type': 'control_sample',
         'source_stamp_s': '10.0', 'phase': 'outbound',
         'pose_x_m': '2', 'pose_y_m': '3.2', 'pose_yaw_rad': '1.57079632679'},
        {'monotonic_s': '1.2', 'record_type': 'control_sample',
         'source_stamp_s': '10.1', 'phase': 'outbound',
         'pose_x_m': '1.9', 'pose_y_m': '3.2', 'pose_yaw_rad': '1.57079632679'},
        {'monotonic_s': '1.3', 'record_type': 'trial_stop',
         'pose_x_m': '1.9', 'pose_y_m': '3', 'pose_yaw_rad': '1.57079632679',
         'reason': 'settled'},
    ]
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    report = analyze_csv(path)
    assert report['odom_callback_count'] is None
    assert report['odom_source_stamp_cadence'] is None
    assert report['odom_receipt_monotonic_cadence'] is None
    assert report['odom_callback_cadence_source'].startswith('unavailable:')
    # Control samples provide motion data but are never mislabeled as the
    # odometry publisher or callback rate.
    assert report['outbound_peak_in_start_body']['left_m'] == pytest.approx(0.1)
