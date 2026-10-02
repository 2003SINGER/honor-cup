import math
import sys
from pathlib import Path

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
import replay_dynamic_wall_snap as replay
from analyze_scan_edge import Transform2D
from m3pro_nav.edge_map import EdgeMap, WALL
from m3pro_nav.pose import Pose2D
from m3pro_nav.pose_correction import PoseCorrectionResult


def _scan(stamp):
    # Rays from cell center toward one finite vertical grid edge at x=0.8 m.
    angles = [-0.6, -0.4, -0.2, 0.2, 0.4, 0.6]
    ranges = [0.2 / math.cos(angle) for angle in angles]
    return {
        'stamp': stamp,
        'frame_id': 'base_link',
        'angle_min': angles[0],
        'angle_increment': (angles[-1] - angles[0]) / (len(angles) - 1),
        'range_min': 0.05,
        'range_max': 4.0,
        'ranges': tuple(ranges),
    }


def test_wall_needs_distinct_frames_and_is_only_available_next_frame(monkeypatch):
    state = replay.ReplayState(Transform2D(0.0, 0.0, 0.0))
    odom = Pose2D(0.6, 0.6, 0.0)
    calls = []

    def fail_if_truth_is_read(_truth):
        raise AssertionError('estimator tried to read maze truth')

    monkeypatch.setattr(replay.audit, 'edge_truth', fail_if_truth_is_read)

    def reject_proposal(endpoints, walls, pose, config=None):
        calls.append(tuple(walls))
        return PoseCorrectionResult(False, 'INSUFFICIENT_WALL_DIRECTIONS',
                                    n_input_hits=len(endpoints))

    monkeypatch.setattr(replay, 'propose_pose_correction', reject_proposal)
    tf = Transform2D(0.0, 0.0, 0.0)
    replay.step_scan(state, _scan(1.0), odom, tf)
    assert calls[-1] == ()
    assert not replay.confirmed_soft_walls(state.edge_map)

    replay.step_scan(state, _scan(2.0), odom, tf)
    # The second frame confirms the wall only after its own proposal ran.
    assert calls[-1] == ()
    walls = replay.confirmed_soft_walls(state.edge_map)
    assert len(walls) == 1
    assert walls[0].confirmed
    key = state.edge_map._pk(walls[0].wall_id)
    assert state.edge_map.soft[key]['state'] == WALL

    replay.step_scan(state, _scan(3.0), odom, tf)
    assert len(calls[-1]) == 1


def test_applied_transform_maps_same_odom_pose_to_corrected_pose(monkeypatch):
    state = replay.ReplayState(Transform2D(0.3, -0.2, 0.4))
    odom = Pose2D(1.2, -0.7, -0.3)
    corrected = Pose2D(1.55, -0.42, 0.6)
    state.edge_map.soft[state.edge_map.edge_key((1, 1), 'W')] = {
        'score': 2.0, 'state': WALL}

    def accepted(endpoints, walls, pose, config=None):
        assert len(walls) == 1
        return PoseCorrectionResult(True, 'ACCEPTED', corrected,
                                    dx=0.1, dy=0.2, dyaw=0.1,
                                    n_input_hits=len(endpoints), n_inliers=8,
                                    n_walls=2, median_residual_m=0.01,
                                    p90_residual_m=0.02)

    monkeypatch.setattr(replay, 'propose_pose_correction', accepted)
    replay.step_scan(state, _scan(1.0), odom, Transform2D(0.0, 0.0, 0.0))
    mapped = replay.audit.compose(state.maze_from_odom,
                                  replay._pose_transform(odom))
    assert math.isclose(mapped.x, corrected.x, abs_tol=1e-12)
    assert math.isclose(mapped.y, corrected.y, abs_tol=1e-12)
    assert math.isclose(mapped.yaw, corrected.yaw, abs_tol=1e-12)


def test_odometry_interpolation_requires_short_bracketing_interval():
    samples = [(0.0, Pose2D(0, 0, 0)), (0.12, Pose2D(1.2, 0, 0))]
    midpoint = replay.interpolate_bounded_pose(samples, 0.06)
    assert math.isclose(midpoint.x, 0.6)
    try:
        replay.interpolate_bounded_pose(
            [(0.0, Pose2D(0, 0, 0)), (0.16, Pose2D(1, 0, 0))], 0.08)
    except ValueError as exc:
        assert 'exceeds 0.150s' in str(exc)
    else:
        raise AssertionError('long odometry bracket was accepted')
