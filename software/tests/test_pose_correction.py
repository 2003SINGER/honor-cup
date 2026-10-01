import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.pose import Pose2D
from m3pro_nav.pose_correction import (KnownWallSegment,
                                      ProjectedEndpoint,
                                      PoseCorrectionConfig,
                                      propose_pose_correction)
import m3pro_nav.pose_correction as pose_correction


def _case(delta=(.025, -.018, math.radians(1.5)), outliers=0):
    current = Pose2D(.53, .47, .13)
    walls = [
        KnownWallSegment((1.0, .1), (1.0, 1.4), confirmed=True,
                         wall_id='east-boundary'),
        KnownWallSegment((.1, 1.0), (1.4, 1.0), confirmed=True,
                         wall_id='north-boundary'),
    ]
    dx, dy, dyaw = delta
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    endpoints = []
    # Construct endpoints that become exact wall observations when the
    # expected correction is applied around the current base position.
    for i in range(10):
        truth = (1.0, .25 + i * .11)
        truth = (truth, )
        truth_pt = truth[0]
        endpoints.append((truth_pt[0] - dx, truth_pt[1] - dy))
    for i in range(10):
        truth_pt = (.25 + i * .11, 1.0)
        endpoints.append((truth_pt[0] - dx, truth_pt[1] - dy))
    # Apply inverse rotation around the pose after translation, yielding the
    # scan coordinates before correction.
    observed = []
    for x, y in endpoints:
        rx, ry = x - current.x, y - current.y
        observed.append(ProjectedEndpoint(
            current.x + c * rx - s * ry,
            current.y + s * rx + c * ry))
    # Nearby spurious returns stay inside the association gate and exercise
    # Huber down-weighting rather than only the correspondence filter.
    observed.extend(ProjectedEndpoint(1.07, .35 + i * .12)
                    for i in range(outliers))
    return current, walls, observed, delta


def test_recovers_bounded_se2_delta_from_two_confirmed_wall_directions():
    current, walls, endpoints, expected = _case(outliers=3)
    result = propose_pose_correction(endpoints, walls, current)
    assert result.accepted, result
    assert result.dx == pytest.approx(expected[0], abs=2e-4)
    assert result.dy == pytest.approx(expected[1], abs=2e-4)
    assert result.dyaw == pytest.approx(expected[2], abs=2e-4)
    assert result.corrected_pose.x == pytest.approx(current.x + expected[0])
    assert result.n_walls == 2


def test_abstains_without_nonparallel_confirmed_walls():
    current, _, endpoints, _ = _case()
    one_direction = [KnownWallSegment((1.0, .1), (1.0, 1.4), True)]
    result = propose_pose_correction(endpoints, one_direction, current)
    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_WALL_DIRECTIONS'


def test_unconfirmed_wall_is_rejected_before_fitting():
    current, _, endpoints, _ = _case()
    walls = [KnownWallSegment((1.0, .1), (1.0, 1.4), True),
             KnownWallSegment((.1, 1.0), (1.4, 1.0), False)]
    result = propose_pose_correction(endpoints, walls, current)
    assert not result.accepted
    assert result.reason == 'UNCONFIRMED_WALL'


def test_abstains_when_solution_exceeds_configured_translation_bound():
    current, walls, endpoints, _ = _case(delta=(.11, 0.0, 0.0))
    result = propose_pose_correction(
        endpoints, walls, current, PoseCorrectionConfig())
    assert not result.accepted
    assert result.reason == 'CORRECTION_EXCEEDS_BOUND'
    assert result.corrected_pose is None


def test_insufficient_hits_abstains():
    result = propose_pose_correction(
        [ProjectedEndpoint(1.0, .5)],
        [KnownWallSegment((1, 0), (1, 1), True),
         KnownWallSegment((0, 1), (1, 1), True)],
        Pose2D(.5, .5, 0.0))
    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_HITS'


def test_wall_extension_near_corner_cannot_support_a_pose_correction():
    walls = [KnownWallSegment((1.0, .2), (1.0, .6), True),
             KnownWallSegment((.2, 1.0), (.6, 1.0), True)]
    # The first group lies 8 cm beyond a finite wall endpoint. Its distance
    # from the infinite line is only 5 mm, but it must not count as a wall hit.
    endpoints = [ProjectedEndpoint(1.005, .68) for _ in range(8)]
    endpoints += [ProjectedEndpoint(.26 + .04 * i, 1.0)
                  for i in range(8)]
    result = propose_pose_correction(endpoints, walls, Pose2D(.5, .5, 0.0))
    assert not result.accepted
    assert result.n_associated == 8
    assert result.reason == 'INSUFFICIENT_WALL_SUPPORT'


def test_wall_direction_observability_is_independent_of_wall_order():
    walls = [KnownWallSegment((0, 0),
                              (math.cos(math.radians(a)),
                               math.sin(math.radians(a))), True)
             for a in (10, 0, 20)]
    assert pose_correction._diverse_normals(walls, 15)
    assert pose_correction._diverse_normals(tuple(reversed(walls)), 15)


def test_final_inliers_must_still_support_two_wall_directions(monkeypatch):
    walls = [KnownWallSegment((1.0, .1), (1.0, 1.4), True, 'vertical'),
             KnownWallSegment((.1, 1.0), (1.4, 1.0), True, 'horizontal')]
    endpoints = [ProjectedEndpoint(1.0, .2 + i * .06) for i in range(10)]
    endpoints += [ProjectedEndpoint(.2 + i * .06, 1.08) for i in range(10)]
    # Hold the fitted pose fixed: vertical hits are inliers; the initially
    # associated but 8 cm-offset horizontal points are all final outliers.
    monkeypatch.setattr(pose_correction, '_solve3', lambda _a, _b: [0, 0, 0])
    cfg = PoseCorrectionConfig(max_iterations=1,
                               max_median_residual_m=.10,
                               max_p90_residual_m=.10,
                               min_inlier_fraction=.4)
    result = propose_pose_correction(endpoints, walls,
                                    Pose2D(.5, .5, 0.0), cfg)
    assert not result.accepted
    assert result.reason == 'INSUFFICIENT_FINAL_WALL_SUPPORT'
    assert result.n_walls == 1


def test_minimum_inlier_fraction_is_checked_per_wall(monkeypatch):
    walls = [KnownWallSegment((1.0, .1), (1.0, 1.4), True, 'vertical'),
             KnownWallSegment((.1, 1.0), (1.4, 1.0), True, 'horizontal')]
    endpoints = [ProjectedEndpoint(1.0, .2 + i * .06) for i in range(10)]
    endpoints += [ProjectedEndpoint(.2 + i * .06,
                                    1.08 if i < 4 else 1.0)
                  for i in range(10)]
    monkeypatch.setattr(pose_correction, '_solve3', lambda _a, _b: [0, 0, 0])
    cfg = PoseCorrectionConfig(max_iterations=1,
                               min_wall_inlier_fraction=.7)
    result = propose_pose_correction(endpoints, walls,
                                    Pose2D(.5, .5, 0.0), cfg)
    assert not result.accepted
    assert result.reason == 'POOR_WALL_SUPPORT'
    assert result.n_walls == 2


@pytest.mark.parametrize('kwargs', [
    {'inlier_gate_m': .13, 'association_gate_m': .12},
    {'max_median_residual_m': .05, 'max_p90_residual_m': .04},
    {'min_wall_angle_deg': 90},
    {'endpoint_guard_m': 0},
])
def test_rejects_internally_inconsistent_fit_thresholds(kwargs):
    with pytest.raises(ValueError):
        PoseCorrectionConfig(**kwargs)
