import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))

from m3pro_nav.wall_trust_prior import evaluate_wall_trust


def hit(tangent=(.05, .10, .15, .20, .25, .30), *, distance=.5,
        mad=.005, heading_proxy=1.5, incidence=None):
    result = {
        'distance_m': distance,
        'normal_mad_m': mad,
        'heading_to_wall_normal_rad': heading_proxy,
        'roi': {'5': {'longest_contiguous_support_points': len(tangent),
                      'longest_contiguous_span_m': max(tangent)-min(tangent)}},
        'roi_tangent_m': list(tangent),
        'roi_signed_normal_m': [0.] * len(tangent),
    }
    if incidence is not None:
        result['beam_incidence_rad'] = incidence
    return result


class WallTrustPriorTests(unittest.TestCase):
    def side_prior(self, cell_x, *, yaw_rate=None):
        edge = ((cell_x, 0), 'N')
        h = hit(tuple(cell_x * .4 + x for x in (.05, .10, .15, .20, .25, .30)))
        return evaluate_wall_trust(edge, h, (0., 0., 0.), yaw_rate_rad_s=yaw_rate)

    def test_measured_side_visibility_has_high_medium_and_strict_bands(self):
        # Endpoint support centers span .2, .6, 1.0, 1.4, ... metres.
        near = self.side_prior(0)
        measured_limit = self.side_prior(2)
        extra_half_cell = evaluate_wall_trust(
            ((2, 0), 'N'),
            hit((.90, 1.00, 1.10, 1.15, 1.18, 1.20)),
            (0., 0., 0.))
        farther = self.side_prior(4)
        self.assertEqual(near.wall_side, 'side')
        self.assertAlmostEqual(near.side_visibility_weight, 1.0)
        self.assertAlmostEqual(measured_limit.side_visibility_weight, 1.0)
        self.assertGreater(extra_half_cell.side_visibility_weight, .5)
        self.assertEqual(farther.side_visibility_weight, .12)

    def test_front_wall_does_not_use_side_visibility_envelope(self):
        edge = ((0, 0), 'E')
        h = hit((.05, .10, .15, .20, .25, .30))
        prior = evaluate_wall_trust(edge, h, (0., 0., 0.))
        self.assertEqual(prior.wall_side, 'front')
        self.assertEqual(prior.side_visibility_weight, 1.0)

    def test_behind_side_support_is_lower_confidence(self):
        edge = ((0, 0), 'N')
        prior = evaluate_wall_trust(
            edge, hit((.05, .10, .15, .20, .25, .30)), (1.5, 0., 0.))
        self.assertEqual(prior.wall_side, 'side')
        self.assertEqual(prior.side_visibility_weight, .18)

    def test_heading_proxy_is_not_beam_incidence(self):
        edge = ((0, 0), 'N')
        a = evaluate_wall_trust(edge, hit(heading_proxy=0.), (0., 0., 0.))
        b = evaluate_wall_trust(edge, hit(heading_proxy=1.5), (0., 0., 0.))
        self.assertEqual(a.incidence_weight, b.incidence_weight)
        c = evaluate_wall_trust(edge, hit(incidence=math.pi / 3), (0., 0., 0.))
        self.assertLess(c.incidence_weight, a.incidence_weight)

    def test_turning_reduces_only_wall_promotion_weight(self):
        calm = self.side_prior(0)
        turning = self.side_prior(0, yaw_rate=.7)
        self.assertEqual(turning.geometry_weight, calm.geometry_weight)
        self.assertLess(turning.promotion_weight, calm.promotion_weight)
        self.assertEqual(turning.pose_correction_weight, 1.0)
        self.assertEqual(turning.turn_promotion_weight, .25)

    def test_range_roi_and_mad_change_prior_continuously(self):
        edge = ((0, 0), 'N')
        near = evaluate_wall_trust(edge, hit(distance=.4, mad=.003), (0., 0., 0.))
        far = evaluate_wall_trust(edge, hit(distance=2., mad=.02), (0., 0., 0.))
        self.assertGreater(near.range_weight, far.range_weight)
        self.assertGreater(near.mad_weight, far.mad_weight)
        self.assertGreater(near.promotion_weight, far.promotion_weight)
        self.assertTrue(near.diagnostic_only)


if __name__ == '__main__':
    unittest.main()
