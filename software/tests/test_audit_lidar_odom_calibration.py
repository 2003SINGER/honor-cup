"""Geometry guards for the read-only bag calibration audit."""

import json
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
import audit_lidar_odom_calibration as audit


class AuditLidarOdomCalibrationTest(unittest.TestCase):
    def test_truth_maps_every_grid_edge_once_and_respects_open_passage(self):
        truth = json.loads((ROOT / 'field' / 'maze_truth_7x7.json').read_text())
        edges = audit.edge_truth(truth)
        self.assertEqual(len(edges), 112)
        self.assertFalse(edges[((1, 2), 'N')])
        self.assertTrue(edges[((1, 1), 'N')])
        self.assertTrue(edges[((0, 4), 'N')])

    def test_scan_projection_applies_base_from_laser_translation(self):
        _, rays, points = audit.scan_frame(
            {'angle_min': 0.0, 'angle_increment': 0.0,
             'range_min': 0.05, 'range_max': 3.0, 'ranges': (1.0,)},
            audit.Pose2D(0.0, 0.0, 0.0),
            audit.Transform2D(0.0, 0.0, 0.0),
            audit.Transform2D(0.1, 0.0, 0.0))
        self.assertEqual(len(rays), 1)
        self.assertAlmostEqual(points[0].x, 1.1)
        self.assertAlmostEqual(points[0].y, 0.0)

    def test_holdout_geometry_metric_improves_for_known_translation(self):
        walls = (audit.KnownWallSegment((0.0, 0.8), (1.0, 0.8), True, 'h'),)
        pose = audit.Pose2D(0.5, 0.5, 0.0)
        points = [audit.ProjectedEndpoint(x, 0.82) for x in (0.2, 0.5, 0.8)]
        before = audit.evaluate_wall_residuals(points, pose, walls,
                                               (0.0, 0.0, 0.0))
        after = audit.evaluate_wall_residuals(points, pose, walls,
                                              (0.0, -0.02, 0.0))
        self.assertAlmostEqual(sum(abs(v) for v in before) / len(before), 0.02)
        self.assertAlmostEqual(sum(abs(v) for v in after) / len(after), 0.0)


if __name__ == '__main__':
    unittest.main()
