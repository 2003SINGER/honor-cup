"""Small geometry checks for the offline scan matching diagnostic."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import replay_scan_match_odom as replay


class ScanMatchGeometryTests(unittest.TestCase):
    def test_recovers_known_rigid_transform(self):
        points = np.random.default_rng(7).uniform(-0.6, 0.6, (180, 2))
        expected = (0.035, -0.025, math.radians(1.2))
        moved = replay._apply(points, replay.inverse(expected))

        fit = replay.trimmed_icp(moved, points, (0.0, 0.0, 0.0))

        self.assertTrue(fit['accepted'])
        self.assertGreaterEqual(fit['matches'], replay.MIN_MATCHES)
        for actual, target in zip(fit['transform'], expected):
            self.assertAlmostEqual(actual, target, places=8)

    def test_real_low_speed_step_is_not_held_as_stationary(self):
        odom_delta = (0.008, 0.0, 0.0)
        icp_delta = (0.007, 0.0, 0.0)
        self.assertFalse(replay._close_stationary(odom_delta, icp_delta))


if __name__ == '__main__':
    unittest.main()
