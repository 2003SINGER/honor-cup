"""Small geometry checks for the read-only known-map replay."""

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'software' / 'tools'
sys.path.insert(0, str(TOOLS))
spec = importlib.util.spec_from_file_location(
    'replay_truth_pose_correction', TOOLS / 'replay_truth_pose_correction.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def test_truth_walls_and_local_selection_are_deterministic():
    truth = json.loads((ROOT / 'field' / 'maze_truth_7x7.json').read_text())
    walls, edges = replay.truth_walls(truth)
    assert len(walls) == 62
    assert len(edges) == 112
    pose = replay.Pose2D(1.4, 0.2, 1.57)
    near = replay.local_walls(walls, pose, 0.8)
    assert near
    assert len(near) < len(walls)
    assert near == replay.local_walls(walls, pose, 0.8)
