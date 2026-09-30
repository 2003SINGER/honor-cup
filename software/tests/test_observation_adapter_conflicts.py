"""Frame-level conflict handling for real lidar observations."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.frame_projector import WorldRay  # noqa: E402
from m3pro_nav.observation_adapter import RealObservationAdapter  # noqa: E402
from m3pro_nav.pose import Pose2D  # noqa: E402


def _ray(index, origin, endpoint):
    ox, oy = origin
    hx, hy = endpoint
    distance = math.hypot(hx - ox, hy - oy)
    return WorldRay(index, ox, oy, (hx - ox) / distance,
                    (hy - oy) / distance, hx, hy, distance, None)


def test_wall_hit_vetoes_open_from_another_ray_for_same_edge_only():
    # Ray 0 hits the north edge of cell (5, 0). Ray 1 independently crosses
    # that same edge before its own farther hit, creating contradictory OPEN
    # evidence in the frame. Ray 2 crosses a different edge, which must remain.
    rays = [
        _ray(0, (2.2, 0.2), (2.2, 0.399)),
        _ray(1, (2.2, 0.2), (2.2, 0.801)),
        # Slightly outside the nominal field: OPEN path evidence is retained
        # for the interior edge while the physical perimeter is suppressed.
        _ray(2, (2.2, 0.2), (2.81, 0.2)),
    ]

    hits, opens, stats = RealObservationAdapter().to_nav_observation(
        rays, Pose2D(2.2, 0.2, 0.0))

    wall_edge = ((5, 0), 'N')
    unrelated_open_edge = ((5, 0), 'E')
    assert wall_edge in hits
    open_edges = {(cell, direction) for cell, direction, _, _ in opens}
    assert wall_edge not in open_edges
    assert unrelated_open_edge in open_edges
    assert stats.n_open_edges == len(opens)
