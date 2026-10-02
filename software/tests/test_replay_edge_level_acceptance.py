import json
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))

import replay_edge_level_acceptance as accept
import replay_frame_grid_snap as snap
from analyze_scan_edge import Pose2D
from replay_pointcloud_wall_match import Segment


class EdgeLevelAcceptanceTests(unittest.TestCase):
    def test_truth_is_loaded_only_after_all_sensor_votes(self):
        hit = {'edge_id': "((1, 0), 'E')", 'cell': [1, 0],
               'direction': 'E', 'segment_index': 0, 'support_points': 12,
               'span_m': .22, 'normal_residual_m': 0.0, 'distance_m': .5}
        frame = {'stamp': 1.0, 'elapsed_s': 0.0,
                 'corrected_pose': [0.0, 0.0, 0.0],
                 'wall_edge_hits': [hit]}
        original_truth = accept.edge_truth
        original_observe = accept.EdgeMap.observe_wall
        observations = []

        def observe(*args, **kwargs):
            observations.append(kwargs.get('stamp'))
            return original_observe(*args, **kwargs)

        def load_truth(data):
            self.assertTrue(observations, 'truth loaded before EdgeMap replay')
            return original_truth(data)

        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            log = root / 'replay.jsonl'
            log.write_text(json.dumps(frame) + '\n')
            truth = root / 'truth.json'
            truth.write_text((ROOT / 'field' / 'maze_truth_7x7.json').read_text())
            with mock.patch.object(accept.EdgeMap, 'observe_wall', observe), \
                    mock.patch.object(accept, 'edge_truth', load_truth):
                accept.replay(log, truth, root / 'out.json', root / 'out.csv')

    def test_duplicate_segments_are_one_vote_per_edge_per_frame(self):
        hit = {'edge_id': "((1, 0), 'E')", 'cell': [1, 0],
               'direction': 'E', 'segment_index': 0, 'support_points': 12,
               'span_m': .22, 'normal_residual_m': 0.0, 'distance_m': .5,
               'same_frame_piece_count': 2}
        frame = {'stamp': 1.0, 'elapsed_s': 0.0,
                 'predicted_pose': [0.0, 0.0, 0.0],
                 'corrected_pose': [0.0, 0.0, 0.0],
                 'wall_edge_hits': [hit]}
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            log = root / 'replay.jsonl'
            log.write_text(json.dumps(frame) + '\n' + json.dumps({
                **frame, 'stamp': 2.0, 'elapsed_s': 1.0,
                'predicted_pose': [0.01, 0.01, 0.01],
                'corrected_pose': [0.01, 0.01, 0.01],
            }) + '\n')
            truth = root / 'truth.json'
            truth.write_text((ROOT / 'field' / 'maze_truth_7x7.json').read_text())
            result = accept.replay(log, truth, root / 'out.json', root / 'out.csv')
        # Two frames each with two fit pieces still produce only two votes.
        # Nearby poses are one diagnostic view.
        edge = next(row for row in result['edges'] if row['edge_id'] == 'E(1,0)')
        self.assertEqual(edge['wall_hits'], 2)
        self.assertEqual(edge['independent_view_hits'], 1)
        self.assertEqual(edge['final_state'], 'WALL')
        self.assertFalse(result['sensor_model']['open_evidence_available'])
        self.assertIsNone(result['sensor_model']['open_hits'])
        self.assertEqual(result['canonical_edges'], 112)

    def test_long_segment_does_not_fill_cell_without_six_support_hits(self):
        # Two observed 20 cm spans flank an empty middle cell. The fitted line
        # crosses it geometrically, but no actual support points land there.
        support = tuple([(0.10 + i * .035, .4) for i in range(6)] +
                        [(0.85 + i * .035, .4) for i in range(6)])
        segment = Segment((support[0]), (support[-1]), len(support), 0.0, support)
        hits = snap._wall_edge_hits([segment], Pose2D(0, 0, 0), Pose2D(0, 0, 0))
        cell_j = {hit['cell_j'] for hit in hits}
        self.assertIn(0, cell_j)
        self.assertIn(2, cell_j)
        self.assertNotIn(1, cell_j)
        self.assertTrue(all(hit['support_points'] >= 6 and hit['span_m'] >= .08
                            for hit in hits))
        self.assertTrue(all(abs(hit['distance_m'] - .4) < 1e-9 for hit in hits))

    def test_single_cell_piece_still_requires_six_hits_and_eight_cm(self):
        few_points = tuple((.10 + i * .025, .4) for i in range(5))
        too_short = Segment(few_points[0], few_points[-1], 5, 0.0, few_points)
        self.assertEqual(snap._wall_edge_hits(
            [too_short], Pose2D(0, 0, 0), Pose2D(0, 0, 0)), [])

    def test_view_clusters_require_position_and_heading_change(self):
        samples = [
            {'frame_index': 0, 'pose': [0.0, 0.0, 0.0]},
            {'frame_index': 1, 'pose': [0.1, 0.1, 0.1]},
            {'frame_index': 2, 'pose': [0.4, 0.1, 0.1]},
        ]
        clusters = accept._view_clusters(samples)
        self.assertEqual([c['frames'] for c in clusters], [[0, 1], [2]])


if __name__ == '__main__':
    unittest.main()
