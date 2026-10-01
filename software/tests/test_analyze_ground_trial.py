import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import analyze_ground_trial as analysis


FIELDS = [
    'received_monotonic_s', 'record_type', 'source_stamp_s', 'reason',
    'pose_x_m', 'pose_y_m', 'pose_yaw_rad', 'world_vx_mps', 'world_vy_mps',
    'command_integral_m', 'odom_forward_path_m', 'odom_lateral_path_m',
    'ref_x_m', 'ref_y_m', 'pos_err_m', 'yaw_err_rad', 'settled',
]


def make_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return path


class GroundTrialAnalysisTests(unittest.TestCase):
    def test_analyzes_metrics_and_uses_stop_integral(self):
        with tempfile.TemporaryDirectory() as directory:
            path = make_csv(Path(directory) / 'trial.csv', [
                {'record_type': 'ground_trial_start', 'reason': 'axis=x;distance=0.4000;speed=0.150;a_dec=0.2;kp_pos=1.000;kd_vel=0.000',
                 'pose_x_m': 2.0, 'pose_y_m': 1.0, 'ref_x_m': 2.4, 'ref_y_m': 1.0},
                {'record_type': 'ground_trial_sample', 'received_monotonic_s': 1.0,
                 'source_stamp_s': 11.0, 'odom_forward_path_m': .39,
                 'odom_lateral_path_m': .01, 'pos_err_m': .02, 'yaw_err_rad': .02,
                 'command_integral_m': .31, 'settled': 'false'},
                {'record_type': 'ground_trial_sample', 'received_monotonic_s': 1.1,
                 'source_stamp_s': 11.1, 'odom_forward_path_m': .43,
                 'odom_lateral_path_m': -.03, 'pos_err_m': .01, 'yaw_err_rad': -.04,
                 'command_integral_m': .38, 'settled': 'true'},
                {'record_type': 'ground_trial_stop', 'reason': 'settled',
                 'pose_x_m': 2.39, 'pose_y_m': 1.0,
                 'command_integral_m': .39},
            ])
            got = analysis.analyze_csv(path)
        self.assertAlmostEqual(got['target_distance_m'], .4)
        self.assertAlmostEqual(got['max_forward_overshoot_m'], .03)
        self.assertAlmostEqual(got['final_position_error_m'], .01)
        self.assertEqual(got['final_position_error_source'], 'ground_trial_stop pose')
        self.assertAlmostEqual(got['max_cross_track_m'], .03)
        self.assertAlmostEqual(got['max_yaw_error_rad'], .04)
        self.assertTrue(got['settled'])
        self.assertEqual(got['stop_reason'], 'settled')
        self.assertAlmostEqual(got['command_integral_m'], .39)
        self.assertTrue(got['odom_freshness_cadence_ok'])
        self.assertEqual(got['parameters']['kp_pos'], '1.000')

    def test_marks_stalled_odom_cadence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = make_csv(Path(directory) / 'stale.csv', [
                {'record_type': 'ground_trial_start', 'reason': 'axis=y;distance=0.2',
                 'pose_x_m': 0, 'pose_y_m': 0, 'ref_x_m': 0, 'ref_y_m': .2},
                {'record_type': 'ground_trial_sample', 'received_monotonic_s': 1.0,
                 'source_stamp_s': 5.0, 'odom_forward_path_m': 0, 'pos_err_m': .2},
                {'record_type': 'ground_trial_sample', 'received_monotonic_s': 1.6,
                 'source_stamp_s': 5.0, 'odom_forward_path_m': 0, 'pos_err_m': .2},
                {'record_type': 'ground_trial_stop', 'reason': 'wall_clock_limit',
                 'command_integral_m': .1},
            ])
            got = analysis.analyze_csv(path)
        self.assertFalse(got['odom_freshness_cadence_ok'])
        self.assertEqual(got['source_stamp_duplicate_or_stalled_samples'], 1)
        self.assertAlmostEqual(got['max_source_stamp_hold_s'], .6)
        self.assertEqual(got['stop_reason'], 'wall_clock_limit')
        self.assertAlmostEqual(got['max_forward_overshoot_m'], 0)

    def test_truncated_log_with_settled_sample_is_not_success(self):
        with tempfile.TemporaryDirectory() as directory:
            path = make_csv(Path(directory) / 'partial.csv', [
                {'record_type': 'ground_trial_start', 'reason': 'axis=x;distance=0.4',
                 'pose_x_m': 0, 'pose_y_m': 0, 'ref_x_m': .4, 'ref_y_m': 0},
                {'record_type': 'ground_trial_sample', 'received_monotonic_s': 1.0,
                 'source_stamp_s': 5.0, 'odom_forward_path_m': .4,
                 'pos_err_m': .01, 'yaw_err_rad': 0, 'settled': 'true'},
            ])
            got = analysis.analyze_csv(path)
        self.assertFalse(got['settled'])
        self.assertEqual(got['stop_reason'], 'missing ground_trial_stop row')
        self.assertEqual(got['final_position_error_source'],
                         'last ground_trial_sample pos_err_m')


if __name__ == '__main__':
    unittest.main()
