import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))

import sweep_wall_quality as sweep
from m3pro_nav.edge_map import UNKNOWN, WALL
from m3pro_nav.wall_evidence_quality import add_temporal_evidence


def vote(edge, frame, pose, *, roi_count=8, roi_span=.12):
    return {'frame_index': frame, 'stamp': float(frame + 1),
            'elapsed_s': float(frame), 'pose': pose,
            'yaw_rate': .01,
            'hit': {'edge_id': repr(edge), 'distance_m': .35,
                    'support_points': roi_count, 'span_m': roi_span,
                    'roi': {'2': {'support_points': roi_count, 'span_m': roi_span},
                            '3': {'support_points': roi_count, 'span_m': roi_span},
                            '4': {'support_points': roi_count, 'span_m': roi_span},
                            '5': {'support_points': roi_count, 'span_m': roi_span}},
                    'normal_mad_m': .005,
                    'heading_to_wall_normal_rad': .1}}


def config(views=1):
    return {'roi_half_width_cm': 2, 'min_support_points': 6,
            'min_span_m': .08, 'max_distance_m': .6,
            'max_grazing_deg': 20, 'max_yaw_rate_rad_s': .08,
            'independent_view_confirmations': views,
            'max_normal_mad_m': .02}


class WallQualitySweepTests(unittest.TestCase):
    @staticmethod
    def temporal_vote(edge, frame, tangent_points, pose=None):
        sample = vote(edge, frame, pose or [0.0, 0.0, 0.0])
        sample['edge'] = edge
        sample['stamp'] = frame * .1 + 1.0
        sample['hit'].update({
            'normal_residual_m': .01,
            'roi_tangent_m': tangent_points,
            'roi_signed_normal_m': [.01] * len(tangent_points),
        })
        return sample

    def test_same_canonical_edge_tracks_across_adjacent_frames(self):
        edge = ((1, 0), 'E')
        samples = [self.temporal_vote(edge, i, [.10, .13, .16]) for i in (0, 1)]
        add_temporal_evidence({edge: samples})
        self.assertEqual([s['temporal_streak'] for s in samples], [1, 2])
        self.assertFalse(samples[0]['temporal_continuity'])
        self.assertTrue(samples[1]['temporal_continuity'])

    def test_adjacent_edge_id_jump_only_abstains_for_matching_cluster(self):
        old_edge, new_edge = ((1, 0), 'E'), ((1, 1), 'E')
        before = self.temporal_vote(old_edge, 0, [.36, .38, .39])
        after = self.temporal_vote(new_edge, 1, [.41, .42, .44])
        add_temporal_evidence({old_edge: [before], new_edge: [after]})
        self.assertTrue(after['edge_id_jump_abstain'])

        unrelated = self.temporal_vote(new_edge, 1, [.65, .68, .70])
        add_temporal_evidence({old_edge: [before], new_edge: [unrelated]})
        self.assertFalse(unrelated['edge_id_jump_abstain'])

    def test_temporal_persistence_does_not_create_view_credits(self):
        edge = ((1, 0), 'E')
        samples = [self.temporal_vote(edge, i, [.10, .13, .16]) for i in range(30)]
        votes = {edge: samples}
        add_temporal_evidence(votes)
        self.assertEqual(samples[-1]['temporal_streak'], 30)
        self.assertEqual(len(sweep._view_clusters(samples)), 1)

    def test_stationary_temporal_streak_still_gets_only_one_map_vote(self):
        edge = ((1, 0), 'E')
        samples = [self.temporal_vote(edge, i, [.10, .13, .16])
                   for i in range(30)]
        cfg = {**sweep._fixed_baseline_config(), 'roi_half_width_cm': 5,
               'use_roi_support': True, 'independent_view_confirmations': 1,
               'min_temporal_streak': 2}
        result = sweep._simulate({edge: samples}, cfg)[edge]
        self.assertEqual(result['max_temporal_streak'], 30)
        self.assertEqual(result['independent_views'], 1)
        self.assertEqual(result['wall_hits'], 1)
        self.assertEqual(result['final_state'], UNKNOWN)

    def test_contiguous_roi_rejects_two_clusters_with_large_global_span(self):
        hit = {'roi': {'2': {'support_points': 10, 'span_m': .38}},
               'roi_tangent_m': [.00, .01, .02, .03, .30, .32, .34, .36, .38],
               'roi_signed_normal_m': [.005] * 9}
        self.assertEqual(sweep._roi_measure(hit, 2), (10, .38))
        count, span = sweep._roi_contiguous_measure(hit, 2)
        self.assertEqual(count, 5)
        self.assertAlmostEqual(span, .08)

    def test_contiguous_half_wall_can_confirm_from_two_scans(self):
        edge = ((1, 1), 'E')
        items = [vote(edge, i, [0, 0, 0], roi_span=.18) for i in (0, 1)]
        for item in items:
            item['hit']['roi']['2'].update({
                'longest_contiguous_support_points': 7,
                'longest_contiguous_span_m': .16})
        cfg = sweep._fixed_baseline_config()
        cfg.update({'contiguous_roi_half_width_cm': 2,
                    'min_contiguous_support_points': 6,
                    'min_contiguous_span_m': .15})
        result = sweep._simulate({edge: items}, cfg)[edge]
        self.assertEqual(result['final_state'], WALL)

    def test_physical_normal_offset_is_reported_without_deciding_edge_id(self):
        edge = ((1, 1), 'E')
        items = [vote(edge, i, [0, 0, 0]) for i in (0, 1)]
        items[0]['hit']['normal_residual_m'] = .07
        items[1]['hit']['normal_residual_m'] = .09
        result = sweep._simulate({edge: items}, sweep._fixed_baseline_config())[edge]
        self.assertAlmostEqual(result['physical_normal_offset_median_m'], .08)
        self.assertAlmostEqual(result['physical_normal_offset_mad_m'], .01)
        self.assertEqual(result['final_state'], WALL)

    def test_wide_roi_recomputed_from_retained_endpoint_offsets(self):
        hit = {'roi': {'5': {'support_points': 2, 'span_m': .02}},
               'roi_tangent_m': [.00, .02, .04, .06, .08, .10, .12],
               'roi_signed_normal_m': [.01, -.02, .03, .06, -.07, .09, .11],
               'roi_tangential_interval_m': [0.0, .12]}
        self.assertEqual(sweep._roi_measure(hit, 5), (2, .02))
        self.assertEqual(sweep._roi_measure(hit, 8), (5, .08))
        self.assertEqual(sweep._roi_measure(hit, 10), (6, .10))
        self.assertEqual(sweep._roi_measure(hit, 12), (7, .12))

    def test_wide_side_wall_is_not_rejected_when_heading_gate_is_disabled(self):
        edge = ((1, 1), 'E')
        items = [vote(edge, 0, [0, 0, 0]), vote(edge, 1, [.25, 0, 0])]
        for item in items:
            item['hit']['heading_to_wall_normal_rad'] = 1.45
        cfg = config(views=1)
        cfg.update({'max_grazing_deg': None, 'roi_half_width_cm': 2,
                    'max_distance_m': None, 'max_yaw_rate_rad_s': None,
                    'max_normal_mad_m': None})
        result = sweep._simulate({edge: items}, cfg)[edge]
        self.assertEqual(result['final_state'], WALL)

    def test_fixed_baseline_uses_original_cell_support_and_span(self):
        edge = ((1, 1), 'E')
        a, b = vote(edge, 0, [0, 0, 0]), vote(edge, 1, [.01, 0, 0])
        for item in (a, b):
            item['hit']['roi']['2'] = {'support_points': 1, 'span_m': .01}
        result = sweep._simulate({edge: [a, b]}, sweep._fixed_baseline_config())[edge]
        self.assertEqual(result['quality_hits'], 2)
        self.assertEqual(result['final_state'], WALL)

    def test_independent_view_confirmation_time_is_causal(self):
        edge = ((1, 1), 'E')
        votes = {edge: [vote(edge, 0, [0, 0, 0]),
                        vote(edge, 3, [.02, 0, 0]),  # same view, ignored
                        vote(edge, 5, [.25, 0, 0]),
                        vote(edge, 8, [.50, 0, 0])]}
        result = sweep._simulate(votes, config(views=2))[edge]
        self.assertEqual(result['final_state'], WALL)
        self.assertEqual(result['independent_views'], 3)
        self.assertEqual(result['wall_hits'], 3)
        self.assertEqual(result['first_wall']['frame_index'], 5)
        self.assertEqual(result['first_wall']['stamp'], 6.0)

    def test_half_wall_with_local_roi_support_is_accepted(self):
        # Partial wall span is valid when observed points support this canonical
        # edge; it does not require an entire maze-cell length of returns.
        edge = ((1, 1), 'E')
        votes = {edge: [vote(edge, 0, [0, 0, 0]),
                        vote(edge, 1, [.25, 0, 0])]}
        result = sweep._simulate(votes, config(views=2))[edge]
        self.assertEqual(result['final_state'], WALL)
        self.assertEqual(result['wall_hits'], 2)

    def test_original_support_half_wall_span_18cm_passes_targeted_gate(self):
        edge = ((1, 1), 'E')
        items = [vote(edge, 0, [0, 0, 0], roi_span=.18),
                 vote(edge, 1, [.25, 0, 0], roi_span=.18)]
        cfg = sweep._fixed_baseline_config()
        cfg.update({'min_span_m': .18, 'independent_view_confirmations': 2})
        result = sweep._simulate({edge: items}, cfg)[edge]
        self.assertEqual(result['quality_hits'], 2)
        self.assertEqual(result['final_state'], WALL)
        self.assertEqual(result['first_wall']['frame_index'], 1)

    def test_repeated_stationary_frames_do_not_self_confirm(self):
        edge = ((1, 1), 'E')
        votes = {edge: [vote(edge, i, [0, 0, 0], roi_span=.18) for i in range(12)]}
        cfg = sweep._fixed_baseline_config()
        cfg.update({'min_span_m': .18, 'independent_view_confirmations': 2})
        result = sweep._simulate(votes, cfg)[edge]
        self.assertEqual(result['quality_hits'], 12)
        self.assertEqual(result['independent_views'], 1)
        self.assertEqual(result['wall_hits'], 0)
        self.assertEqual(result['final_state'], UNKNOWN)

    def test_unobserved_middle_cell_of_long_wall_stays_unknown(self):
        # A fitted long wall may touch neighboring cells, but this tool only
        # evolves evidence for explicitly emitted per-edge hits.
        left, middle, right = ((1, 2), 'E'), ((2, 2), 'E'), ((3, 2), 'E')
        votes = {left: [vote(left, 0, [0, 0, 0]), vote(left, 1, [.3, 0, 0])],
                 right: [vote(right, 2, [0, 0, 0]), vote(right, 3, [.3, 0, 0])]}
        states = sweep._simulate(votes, config(views=2))
        self.assertEqual(states[left]['final_state'], WALL)
        self.assertEqual(states[right]['final_state'], WALL)
        self.assertEqual(states[middle]['final_state'], UNKNOWN)
        self.assertEqual(states[middle]['quality_hits'], 0)


if __name__ == '__main__':
    unittest.main()
