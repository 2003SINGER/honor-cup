import sys
from pathlib import Path

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
import analyze_conservative_open as conservative
from analyze_scan_edge import Transform2D


def test_common_candidate_ray_edge_is_required():
    # Both plausible origins reach the same vertical edge before the hit.
    assert conservative.crossing_edge((.5, .5), (1.1, .5), .01) == frozenset({('V', 2, 1)})


def test_corner_and_near_hit_crossings_abstain():
    assert conservative.crossing_edge((.5, .5), (.86, .5), .01) == frozenset()
    assert conservative.crossing_edge((.5, .5), (1.1, 1.1), .01) == frozenset()


def test_multiple_interior_edges_are_retained():
    crossed = conservative.crossing_edge((.5, .5), (1.7, .5), .01)
    assert crossed == frozenset({('V', 2, 1), ('V', 3, 1), ('V', 4, 1)})


def test_same_frame_wall_candidate_is_detected_for_open_conflict_veto():
    assert conservative.hit_edge_candidates((.8, .55)) == {('V', 2, 1)}


def test_synthetic_center_ray_is_not_a_candidate_origin():
    origins = conservative.laser_origins_from_tf([
        ('base_link', 'laser0_frame', Transform2D(-.11617, .09156, 0)),
        ('base_link', 'laser1_frame', Transform2D(.10766, -.09078, 0)),
    ])
    assert origins == ((-.11617, .09156), (.10766, -.09078))
    assert (0.0, 0.0) not in origins
