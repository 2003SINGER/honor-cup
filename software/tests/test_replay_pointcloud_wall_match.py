"""Bounded synthetic checks for the endpoint-only wall matcher."""
import math
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'software'/'tools'))
from replay_pointcloud_wall_match import (Segment, associate_segment,
    History, extract_segments, segment_error, split_by_grid_cells)
from m3pro_nav.pose import Pose2D


class PointCloudWallMatchTests(unittest.TestCase):
    def test_tls_recovers_oblique_direction_and_finite_span(self):
        # Endpoint coordinates, not ray origins/directions.
        pts=[]
        for i in range(20):
            t=i*.015
            pts.append((.5+t, .3+.2*t + (.001 if i%2 else -.001)))
        pts.extend(((.1+i*.07, .9) for i in range(8)))
        segs=extract_segments(pts,threshold=.008,min_inliers=6,min_span=.12)
        self.assertGreaterEqual(len(segs),2)
        fitted=min(segs,key=lambda s:abs(s.length-.285))
        tx,ty=fitted.tangent
        self.assertAlmostEqual(abs(ty/tx),.2,delta=.025)
        self.assertAlmostEqual(fitted.length,.285,delta=.035)

    def test_gap_splits_disconnected_collinear_walls(self):
        pts=[(x,y) for lo in (.0,.24) for x in [lo+i*.012 for i in range(12)]
             for y in (.7,.701)]
        segs=extract_segments(pts,threshold=.006,min_inliers=6,min_span=.08,max_gap=.06)
        self.assertGreaterEqual(len(segs),2)

    def test_grid_association_abstains_on_non_manhattan_line(self):
        seg=Segment((.1,.2),(.35,.38),20,.002)
        match,reason,_=associate_segment(seg)
        self.assertIsNone(match)
        self.assertIn(reason,('LINE_DIRECTION','GRID_LINE_DISTANCE'))

    def test_grid_association_keeps_edge_identity_and_continuous_span(self):
        seg=Segment((.41,.21),(.412,.37),10,.001)
        match,reason,_=associate_segment(seg)
        self.assertEqual(reason,'UNIQUE')
        self.assertEqual(match[1],((0,0),'E'))
        self.assertAlmostEqual(seg.length,.16,delta=.003)

    def test_line_angle_error_is_undirected(self):
        a=Segment((0.,0.),(1.,0.),10,.0)
        b=Segment((1.,.01),(0.,.01),10,.0)
        normal,angle,overlap=segment_error(b,a)
        self.assertAlmostEqual(normal,.01,places=12)
        self.assertAlmostEqual(angle,0.,places=12)
        self.assertAlmostEqual(overlap,1.,places=12)

    def test_long_line_is_split_into_hit_supported_cell_pieces(self):
        pts=tuple((.015+i*.0194,.41+(.001 if i%2 else -.001)) for i in range(38))
        seg=Segment((pts[0][0],.41),(pts[-1][0],.41),len(pts),.001,pts)
        pieces=split_by_grid_cells(seg)
        self.assertEqual(len(pieces),2)
        self.assertTrue(all(piece.inliers>=6 and piece.length<.4 for piece in pieces))
        self.assertEqual(sum(piece.inliers for piece in pieces),len(pts))

    def test_independent_pose_clusters_do_not_require_one_second_gap(self):
        h=History(((0,0),'E'))
        views=(Pose2D(0.,0.,0.),Pose2D(.3,0.,0.),
               Pose2D(0.,.3,0.),Pose2D(.3,.3,0.))
        seg=Segment((.4,.1),(.4,.4),8,.001)
        for i,p in enumerate(views): h.add(seg,p,i*.2)
        self.assertTrue(h.stable)


if __name__=='__main__': unittest.main()
