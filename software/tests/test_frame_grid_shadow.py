"""The online diagnostic recurrence must not alter navigation state."""

import math
from dataclasses import dataclass
import unittest

from m3pro_nav.frame_grid_shadow import FrameGridShadow
from m3pro_nav.frame_projector import LaserExtrinsic, ManualMazeAnchor
from m3pro_nav.pose import Pose2D, norm_angle


@dataclass(frozen=True)
class Ray:
    angle: float
    range: float
    valid: bool = True


@dataclass(frozen=True)
class Frame:
    stamp: float
    rays: tuple[Ray, ...]


def _frame(stamp):
    points = [(x, y) for x in (0.4, 0.8)
              for y in (0.04 + i * 0.02 for i in range(20))]
    return Frame(stamp, tuple(Ray(math.atan2(y, x), math.hypot(x, y))
                              for x, y in points))


class FrameGridShadowTests(unittest.TestCase):
    def assert_pose_close(self, actual, expected):
        for measured, target in zip(actual, expected):
            self.assertAlmostEqual(measured, target, places=8)

    def test_shadow_is_causal_and_leaves_manual_anchor_unchanged(self):
        wheel0 = Pose2D(1.0, 2.0, 0.2)
        wheel1 = Pose2D(1.03, 2.01, 0.25)
        anchor = ManualMazeAnchor((3, 0), 'N', wheel0)
        transform_before = anchor.transform
        shadow = FrameGridShadow()
        ext = LaserExtrinsic.from_yaml(0, 0, 0)
        first = shadow.process(_frame(1.0), wheel0, ext, anchor)
        self.assertEqual(first['reason'], 'FIRST_FRAME_ANCHOR')
        self.assert_pose_close(first['corrected_pose'], [1.4, 0.2, math.pi / 2])
        prior = shadow.previous_corrected.copy()
        second = shadow.process(_frame(1.14), wheel1, ext, anchor)
        self.assertGreaterEqual(second['fitted_wall_count'], 1)
        self.assertGreaterEqual(second['elapsed_ms'], 0)
        # The prediction is previous corrected pose composed with the SE(2)
        # relative wheel displacement, including the wheel yaw increment.
        dx, dy = wheel1.x - wheel0.x, wheel1.y - wheel0.y
        c0, s0 = math.cos(-wheel0.yaw), math.sin(-wheel0.yaw)
        local_x, local_y = c0 * dx - s0 * dy, s0 * dx + c0 * dy
        cp, sp = math.cos(prior.yaw), math.sin(prior.yaw)
        self.assert_pose_close(second['predicted_pose'], [
            prior.x + cp * local_x - sp * local_y,
            prior.y + sp * local_x + cp * local_y,
            norm_angle(prior.yaw + wheel1.yaw - wheel0.yaw)])
        self.assertIs(anchor.transform, transform_before)
        previous_stamp = shadow.previous_stamp
        rejected = shadow.process(_frame(1.14), wheel1, ext, anchor)
        self.assertEqual(rejected['reason'], 'NONMONOTONIC_SCAN_STAMP')
        self.assertEqual(shadow.previous_stamp, previous_stamp)


    def test_shadow_reanchors_diagnostic_only_after_large_scan_gap(self):
        wheel0 = Pose2D(0, 0, 0)
        anchor = ManualMazeAnchor((3, 0), 'N', wheel0)
        shadow = FrameGridShadow(max_gap_s=0.8)
        ext = LaserExtrinsic.from_yaml(0, 0, 0)
        shadow.process(_frame(1.0), wheel0, ext, anchor)
        result = shadow.process(_frame(2.0), Pose2D(.1, 0, 0), ext, anchor)
        self.assertEqual(result['reason'], 'FIRST_FRAME_ANCHOR')
        self.assert_pose_close(result['corrected_pose'], [1.4, .3, math.pi / 2])


if __name__ == '__main__':
    unittest.main()
