import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))

from replay_physical_wall_tracks import (WallTrack, line_geometry, overlap,
    _topology_hint)
from replay_pointcloud_wall_match import Segment, split_by_grid_cells


def line(x0, x1, y, n=8):
    pts=tuple((x0+(x1-x0)*i/(n-1),y) for i in range(n))
    return Segment(pts[0],pts[-1],n,0.,pts)


class PhysicalWallTrackTests(unittest.TestCase):
  def test_undirected_line_normal_is_sign_canonical_near_angle_wrap(self):
    a=Segment((.5,1.01),(.9,1.00),8,0.)
    b=Segment((.9,1.00),(.5,1.01),8,0.)
    ga,gb=line_geometry(a),line_geometry(b)
    self.assertAlmostEqual(ga[1],gb[1],places=9)
    self.assertAlmostEqual(overlap(ga[2],gb[2]),math.dist(a.a,a.b),places=8)
    self.assertEqual(_topology_hint(a),('H',3,1))

  def test_repeated_same_view_cannot_promote_seed_wall(self):
    track=WallTrack(0)
    for i in range(40):
        track.add(line(0.1,.38,.4),(0.,0.,0.),i*.1)
    self.assertFalse(track.stable)
    self.assertEqual(len(track.views),1)

  def test_independent_views_preserve_measured_two_centimeter_offset(self):
    track=WallTrack(0)
    poses=[(0.,0.,0.),(.18,0.,0.),(.18,.02,math.radians(18))]
    for i,pose in enumerate(poses):
        track.add(line(.1,.38,.42),pose,float(i+1)*2)
    self.assertTrue(track.stable)
    self.assertLess(abs((track.geometry.a[1]+track.geometry.b[1])/2-.42),1e-9)

  def test_long_fitted_wall_only_emits_cells_with_own_hit_support(self):
    support=tuple((x,.4) for lo,hi in ((.04,.36),(.84,1.16))
                  for x in [lo+i*(hi-lo)/9 for i in range(10)])
    seg=Segment(support[0],support[-1],len(support),0.,support)
    pieces=split_by_grid_cells(seg,min_points=6,min_span=.08)
    cells={math.floor((p.a[0]+p.b[0])/2/.4) for p in pieces}
    self.assertEqual(cells,{0,2})
    self.assertTrue(all(p.inliers>=6 for p in pieces))


if __name__=='__main__':
    unittest.main()
