"""Synthetic invariants for continuous wall coordinates and pose correction."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))

from replay_continuous_wall_geometry import (EdgeGeometry,
    interpolate_bounded_pose)
from m3pro_nav.pose import Pose2D
from m3pro_nav.pose_correction import (KnownWallSegment,
    ProjectedEndpoint, PoseCorrectionConfig, propose_pose_correction)


class ContinuousWallGeometryTests(unittest.TestCase):
    def test_two_centimeter_wall_offset_is_not_forced_into_pose(self):
        # Gauge/pose is exact. Physical geometry is deliberately 20 mm away
        # from the nominal grid coordinate; correction should preserve it.
        pose = Pose2D(.60, .60, 0.0)
        walls = (
            KnownWallSegment((.42, .20), (.42, 1.00), True, 'V-offset'),
            KnownWallSegment((.20, .38), (1.00, .38), True, 'H-offset'))
        points = [ProjectedEndpoint(.42, y) for y in
                  (.28, .34, .43, .51, .67, .74, .83, .91)]
        points += [ProjectedEndpoint(x, .38) for x in
                   (.28, .34, .43, .51, .67, .74, .83, .91)]
        result = propose_pose_correction(points, walls, pose,
            PoseCorrectionConfig(max_translation_m=.10))
        self.assertTrue(result.accepted, result.reason)
        self.assertLess(math.hypot(result.dx, result.dy), .002)
        self.assertLess(abs(result.dyaw), math.radians(.1))

    def test_historical_continuous_walls_correct_a_later_odom_offset(self):
        # This scan's projected endpoints include a known odom translation
        # error. The historical wall geometry remains at its measured offset.
        true_pose = Pose2D(.60, .60, 0.0)
        predicted_pose = Pose2D(.62, .59, 0.0)
        walls = (
            KnownWallSegment((.42, .20), (.42, 1.00), True, 'V-offset'),
            KnownWallSegment((.20, .38), (1.00, .38), True, 'H-offset'))
        error = (predicted_pose.x - true_pose.x,
                 predicted_pose.y - true_pose.y)
        points = [ProjectedEndpoint(.42 + error[0], y + error[1]) for y in
                  (.28, .34, .43, .51, .67, .74, .83, .91)]
        points += [ProjectedEndpoint(x + error[0], .38 + error[1]) for x in
                   (.28, .34, .43, .51, .67, .74, .83, .91)]
        result = propose_pose_correction(points, walls, predicted_pose,
            PoseCorrectionConfig(max_translation_m=.10))
        self.assertTrue(result.accepted, result.reason)
        self.assertAlmostEqual(result.dx, -error[0], delta=.002)
        self.assertAlmostEqual(result.dy, -error[1], delta=.002)

    def test_same_pose_and_time_repeated_scans_are_one_view_cluster(self):
        edge = EdgeGeometry(('cell', 'east'), 'V', .4)
        for stamp in (0.0, .2, .4, .6, .8, 1.0):
            edge.observe(.42, Pose2D(.5, .5, 0.0), stamp)
        self.assertFalse(edge.stable)
        self.assertEqual(len(edge.views), 1)

    def test_four_separated_views_promote_offset_and_conflict_does_not_move_it(self):
        edge = EdgeGeometry(('cell', 'east'), 'V', .4)
        views = (Pose2D(.0, .0, 0.0), Pose2D(.3, .0, 0.0),
                 Pose2D(.0, .3, 0.0), Pose2D(.3, .3, 0.0))
        for i, pose in enumerate(views[:3]):
            edge.observe(.42, pose, i * 2.0)
            self.assertFalse(edge.stable)
        edge.observe(.42, views[3], 6.0)
        self.assertTrue(edge.stable)
        self.assertAlmostEqual(edge.coordinate, .42, places=12)
        edge.observe(.48, Pose2D(.7, .4, 0.0), 8.0)
        self.assertAlmostEqual(edge.coordinate, .42, places=12)
        self.assertEqual(edge.conflicts, 1)

    def test_odometry_interpolation_rejects_bracket_over_150ms(self):
        samples = [(0.0, Pose2D(0.0, 0.0, 0.0)),
                   (.20, Pose2D(.2, 0.0, 0.0))]
        with self.assertRaisesRegex(ValueError, 'exceeds 0.150s'):
            interpolate_bounded_pose(samples, .10)
        tight = [(0.0, Pose2D(0.0, 0.0, 0.0)),
                 (.10, Pose2D(.1, 0.0, 0.0))]
        result = interpolate_bounded_pose(tight, .05)
        self.assertAlmostEqual(result.x, .05)


if __name__ == '__main__':
    unittest.main()
