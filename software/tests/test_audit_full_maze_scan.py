import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
import audit_full_maze_scan as audit


class AuditFullMazeScanTest(unittest.TestCase):
    def test_truth_wall_segments_are_unique_and_finite(self):
        truth = json.loads((ROOT / 'field' / 'maze_truth_7x7.json').read_text())
        walls = audit.truth_wall_segments(truth)
        self.assertEqual(len(walls), 62)
        self.assertTrue(all(a != c or b != d for a, b, c, d in walls))

    def test_point_distance_uses_finite_segment_endpoints(self):
        wall = (0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(audit.point_to_segment_distance((0.5, 0.03), wall),
                               0.03)
        self.assertAlmostEqual(audit.point_to_segment_distance((1.04, 0.0), wall),
                               0.04)

    def test_motion_class_uses_both_translation_and_yaw_thresholds(self):
        self.assertEqual(audit.motion_class(.02, .04), 'static')
        self.assertEqual(audit.motion_class(.04, .01), 'moving')
        self.assertEqual(audit.motion_class(.01, .06), 'moving')


if __name__ == '__main__':
    unittest.main()
