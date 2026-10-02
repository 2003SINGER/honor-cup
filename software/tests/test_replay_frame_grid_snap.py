"""Synthetic acceptance cases for the per-frame Manhattan grid snap.

These tests use only synthetic wall geometry. They deliberately do not import
the maze truth map: the estimator must infer the shared correction from the
grid prior and current-frame segments alone.
"""
import math
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))

from analyze_scan_edge import Pose2D
from replay_pointcloud_wall_match import Segment, split_by_grid_cells
import replay_frame_grid_snap as snap


def _segment(a, b, inliers=12):
    points = tuple((a[0] + (b[0] - a[0]) * i / (inliers - 1),
                    a[1] + (b[1] - a[1]) * i / (inliers - 1))
                   for i in range(inliers))
    return Segment(a, b, inliers, 0.0, points)


def _distorted_grid(*, dx=0.0, dy=0.0, yaw=0.0, x_noise=(), y_noise=()):
    """Ideal grid lines seen through one shared rigid pose error."""
    c, s = math.cos(yaw), math.sin(yaw)

    def transform(point):
        x, y = point
        return (c * x - s * y + dx, s * x + c * y + dy)

    segments = []
    for i, x in enumerate((0.4, 0.8, 1.2)):
        x += x_noise[i] if i < len(x_noise) else 0.0
        segments.append(_segment(transform((x, 0.12)),
                                 transform((x, 0.68))))
    for i, y in enumerate((0.4, 0.8)):
        y += y_noise[i] if i < len(y_noise) else 0.0
        segments.append(_segment(transform((0.12, y)),
                                 transform((1.28, y))))
    return segments


class FrameGridSnapTests(unittest.TestCase):
  def test_joint_consensus_recovers_shared_xy_and_yaw_error(self):
    segments = _distorted_grid(dx=0.070, dy=-0.045,
                               yaw=math.radians(3.0),
                               x_noise=(0.006, -0.004, 0.003),
                               y_noise=(-0.005, 0.004))
    result = snap.solve_frame_correction(segments, Pose2D(0.0, 0.0, 0.0))

    self.assertTrue(result.accepted, result.reason)
    self.assertAlmostEqual(result.dx, -0.070, delta=0.015)
    self.assertAlmostEqual(result.dy, 0.045, delta=0.015)
    self.assertAlmostEqual(result.dyaw, math.radians(-3.0),
                           delta=math.radians(0.7))
    self.assertGreaterEqual(result.inlier_wall_count, 4)
    self.assertLess(result.post_residual_m, result.pre_residual_m)


  def test_vertical_only_walls_do_not_correct_unobservable_y(self):
    segments = _distorted_grid(dx=0.065, dy=0.0,
                               yaw=math.radians(2.0))[:3]
    predicted = Pose2D(0.15, 0.25, 0.0)

    result = snap.solve_frame_correction(segments, predicted)

    self.assertTrue(result.accepted, result.reason)
    self.assertEqual(result.mode, 'X_ONLY')
    self.assertAlmostEqual(result.dx, -0.065, delta=0.02)
    self.assertAlmostEqual(result.dy, 0.0, delta=1e-9)
    self.assertAlmostEqual(result.corrected_pose.y, predicted.y, delta=1e-9)


  def test_equally_plausible_adjacent_grid_lines_abstain(self):
    # A single wall exactly halfway between x=.4 and x=.8 has no defensible
    # local grid identity. The solver must not manufacture a correction.
    segments = [_segment((0.6, 0.1), (0.6, 0.7))]
    predicted = Pose2D(0.0, 0.0, 0.0)

    result = snap.solve_frame_correction(segments, predicted)

    self.assertFalse(result.accepted)
    self.assertEqual(result.associated_wall_count, 0)
    self.assertEqual(result.corrected_pose, predicted)


  def test_local_association_and_correction_bounds_prevent_grid_jump(self):
    # A valid small common offset can be corrected, but it must stay within
    # the fixed first-version search window and keep each wall near its nearby
    # line identity.
    segments = _distorted_grid(dx=0.085, dy=-0.055,
                               yaw=math.radians(2.0))
    result = snap.solve_frame_correction(segments, Pose2D(0.0, 0.0, 0.0))

    self.assertTrue(result.accepted, result.reason)
    self.assertLessEqual(abs(result.dx), 0.12)
    self.assertLessEqual(abs(result.dy), 0.12)
    self.assertLessEqual(abs(result.dyaw), math.radians(6.0))
    self.assertLess(abs(result.corrected_pose.x), 0.12)
    self.assertLess(abs(result.corrected_pose.y), 0.12)

  def test_correction_larger_than_limit_abstains_instead_of_clamping(self):
    # Isolate the observable x correction so yaw and y cannot trade off the
    # amount of translation. Widen association only; retain the 12 cm hard
    # correction bound.
    segments = _distorted_grid(dx=0.15)[:3]
    config = snap.GridSnapConfig(association_gate=0.20, max_dyaw=0.0)
    predicted = Pose2D(0.0, 0.0, 0.0)

    result = snap.solve_frame_correction(segments, predicted, config=config)

    self.assertFalse(result.accepted)
    self.assertEqual(result.corrected_pose, predicted)
    self.assertEqual(result.mode, 'REJECTED')

  def test_out_of_limit_translation_cannot_be_disguised_as_yaw(self):
    # An exact-axis frame displaced by 15 cm must not invent a 6 degree turn
    # to bring its translation components inside the 12 cm limit.
    segments = _distorted_grid(dx=0.15, dy=-0.15)
    result = snap.solve_frame_correction(
        segments, Pose2D(0.0, 0.0, 0.0),
        config=snap.GridSnapConfig(association_gate=0.20))
    self.assertFalse(result.accepted)
    self.assertAlmostEqual(result.dyaw, 0.0)

  def test_duplicate_fragments_on_one_grid_line_are_not_two_votes(self):
    duplicate = _segment((0.46, 0.10), (0.46, 0.70))
    result = snap.solve_frame_correction([duplicate, duplicate],
                                         Pose2D(0.0, 0.0, 0.0))

    self.assertFalse(result.accepted)
    self.assertEqual(result.mode, 'REJECTED')
    self.assertEqual(result.reason, 'INSUFFICIENT_INDEPENDENT_WALLS')


  def test_long_wall_emits_only_cells_with_local_point_support(self):
    # The fitted support has points on cells 0 and 2, with no hits in cell 1.
    # Splitting by the grid must never synthesize a middle-cell wall.
    support = tuple((x, 0.4)
                    for lo, hi in ((0.04, 0.36), (0.84, 1.16))
                    for x in (lo + (hi - lo) * i / 9 for i in range(10)))
    segment = Segment(support[0], support[-1], len(support), 0.0, support)

    pieces = split_by_grid_cells(segment, min_points=6, min_span=0.08)

    cells = {math.floor((piece.a[0] + piece.b[0]) / 2 / 0.4)
             for piece in pieces}
    self.assertEqual(cells, {0, 2})
    self.assertTrue(all(piece.inliers >= 6 for piece in pieces))

  def test_posthoc_fp_diagnostics_keep_edge_samples_and_frame_runs(self):
    def detail(label, fp_edges, pieces):
      return {'label':label,'segment_index':2,'fp_edge_ids':fp_edges,
              'support_points':18,'span_m':0.72,'pieces':pieces}
    def piece(edge, label='FP'):
      return {'edge_id':edge,'label':label,'truth':'open','truth_wall':False,
              'orientation':'V','line_k':0,'cell_j':2,
              'support_points':8,'span_m':0.31,'line_residual_m':0.02}
    frames=[]
    boundary=repr(('B',(0,2),'W'))
    interior=repr(((2,1),'E'))
    other=repr(('B',(3,6),'N'))
    for index, edges in enumerate(([boundary,interior], [boundary], [other])):
      frames.append({'frame_index':index,'stamp':100+index,'elapsed_s':index*0.1,
          'corrected_pose':[0.1,0.2,0.0],
          'anchor_plus_raw_odom_pose':[0.0,0.0,0.0],
          'corrected':{'details':[detail('FP',edges,[piece(e) for e in edges])]},
          'anchor_plus_raw_odom':{'details':[detail('FP',edges,[piece(e) for e in edges])]}})

    scored=snap._fp_edge_diagnostics(frames)['corrected']

    edge=scored['segment_fp'][boundary]
    self.assertEqual(edge['count'],2)
    self.assertEqual(edge['consecutive_frame_runs'],[
        {'start_frame_index':0,'end_frame_index':1,'frame_count':2}])
    sample=edge['samples'][0]
    self.assertEqual((sample['stamp'],sample['elapsed_s'],sample['pose']),
                     (100,0.0,[0.1,0.2,0.0]))
    self.assertEqual(sample['support_points'],18)
    self.assertEqual(scored['piece_fp'][boundary]['count'],2)
    self.assertEqual(scored['segment_fp'][interior]['count'],1)
    self.assertEqual([s['segment_index'] for s in scored['piece_fp'][interior]['samples']],
                     [2])
    self.assertEqual(scored['piece_fp'][other]['samples'][0]['line_residual_m'],0.02)
    self.assertEqual(scored['piece_fp'][boundary]['samples'][0]['orientation'],'V')
    self.assertEqual(scored['piece_fp'][boundary]['samples'][0]['line_k'],0)
    self.assertEqual(json.loads(json.dumps(scored))['segment_fp'][boundary]['count'],2)


if __name__ == '__main__':
    unittest.main()
