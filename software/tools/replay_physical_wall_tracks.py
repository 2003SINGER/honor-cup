#!/usr/bin/env python3
"""Offline causal replay using continuous finite physical-wall tracks.

STATUS: SUPERSEDED
SUPERSEDED_BY: replay_frame_grid_snap.py (offline pose candidate)
Reason: promoted track identities conflicted in later scans; keep for replay.

Tracks are associated by measured direction, normal distance, and finite
tangential overlap. Grid geometry is never used to create or move a track.
Only prior stable tracks may propose pose corrections. Truth is consulted
after replay for a coarse topology score. This tool is read-only with respect
to bags and has no ROS/control side effects.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))

import audit_lidar_odom_calibration as audit
from analyze_scan_edge import CELL_SIZE_M, Pose2D
from replay_continuous_wall_geometry import _read_bag, _pose, _tf, interpolate_bounded_pose
from replay_pointcloud_wall_match import (Segment, extract_segments,
    _global_segment, _repose_segment, segment_error)
from m3pro_nav.pose_correction import (KnownWallSegment, PoseCorrectionConfig,
    ProjectedEndpoint, propose_pose_correction)
from m3pro_nav.grid_association import edge_id_for_line


def angle_delta(a, b):
    return abs(math.atan2(math.sin(2*(a-b)), math.cos(2*(a-b)))/2)


def line_geometry(seg):
    raw_tx, raw_ty = seg.tangent
    # Canonical tangent: +x for mostly-horizontal lines, +y for
    # mostly-vertical lines. This is invariant to endpoint order and avoids
    # moving a small negative slope into the pi-near direction.
    if ((abs(raw_tx) >= abs(raw_ty) and raw_tx < 0) or
            (abs(raw_ty) > abs(raw_tx) and raw_ty < 0)):
        raw_tx, raw_ty = -raw_tx, -raw_ty
    tx, ty = raw_tx, raw_ty
    angle = math.atan2(ty, tx)
    nx, ny = -math.sin(angle), math.cos(angle)
    # Canonicalize the undirected line normal so near-0 and near-pi tangents
    # yield the same signed normal coordinate.
    if nx < -1e-9 or (abs(nx) <= 1e-9 and ny < 0):
        nx, ny = -nx, -ny
    mid = ((seg.a[0]+seg.b[0])/2, (seg.a[1]+seg.b[1])/2)
    normal = mid[0]*nx + mid[1]*ny
    lo = min(seg.a[0]*tx+seg.a[1]*ty, seg.b[0]*tx+seg.b[1]*ty)
    hi = max(seg.a[0]*tx+seg.a[1]*ty, seg.b[0]*tx+seg.b[1]*ty)
    return angle, normal, (lo, hi)


def overlap(a, b):
    return max(0.0, min(a[1], b[1])-max(a[0], b[0]))


def view_independent(a, b):
    dx, dy = a[0]-b[0], a[1]-b[1]
    dyaw = abs(math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2])))
    return math.hypot(dx, dy) >= .15 or dyaw >= math.radians(15)


@dataclass
class WallTrack:
    track_id: int
    views: list = field(default_factory=list)  # (stamp, viewpoint, segment)
    stable: bool = False
    geometry: Segment | None = None
    conflicts: int = 0

    def add(self, seg, pose, stamp):
        for old_stamp, old_pose, _ in self.views:
            if stamp-old_stamp < 1.0 or not view_independent(pose, old_pose):
                return False
        if self.stable:
            da, dn, ov = segment_error(seg, self.geometry)
            if da > math.radians(4) or dn > .035 or ov < .04:
                self.conflicts += 1
            return True
        self.views.append((stamp, (pose[0], pose[1], pose[2]), seg))
        self._try_promote()
        return True

    def _try_promote(self):
        if len(self.views) < 3:
            return
        segs = [v[2] for v in self.views]
        base_angle = line_geometry(segs[0])[0]
        angles = [base_angle + math.atan2(math.sin(2*(line_geometry(s)[0]-base_angle)),
                                         math.cos(2*(line_geometry(s)[0]-base_angle)))/2
                  for s in segs]
        angle = sum(angles)/len(angles)
        if max(angles)-min(angles) > math.radians(4):
            return
        tx, ty = math.cos(angle), math.sin(angle)
        nx, ny = -ty, tx
        offsets, intervals = [], []
        for s in segs:
            mx, my = (s.a[0]+s.b[0])/2, (s.a[1]+s.b[1])/2
            offsets.append(mx*nx+my*ny)
            q = [p[0]*tx+p[1]*ty for p in (s.a, s.b)]
            intervals.append((min(q), max(q)))
        med = sorted(offsets)[len(offsets)//2]
        if max(abs(x-med) for x in offsets) > .035:
            return
        # Preserve only tangential support that was observed by at least two
        # independent views. This prevents a line fit extending across gaps.
        cuts = sorted({x for interval in intervals for x in interval})
        supported = []
        for lo, hi in zip(cuts, cuts[1:]):
            mid = (lo+hi)/2
            if sum(a <= mid <= b for a,b in intervals) >= 2 and hi-lo >= .08:
                if supported and lo-supported[-1][1] <= .02:
                    supported[-1] = (supported[-1][0], hi)
                else:
                    supported.append((lo, hi))
        if not supported:
            return
        lo, hi = max(supported, key=lambda x:x[1]-x[0])
        if hi-lo < .10:
            return
        cx, cy = med*nx+(lo+hi)/2*tx, med*ny+(lo+hi)/2*ty
        self.geometry = Segment((cx-(hi-lo)/2*tx, cy-(hi-lo)/2*ty),
                                (cx+(hi-lo)/2*tx, cy+(hi-lo)/2*ty),
                                sum(s.inliers for s in segs),
                                max(s.rms for s in segs))
        self.stable = True


class TrackMap:
    def __init__(self):
        self.tracks = []

    def match(self, seg, gate_normal=.09, gate_angle=math.radians(12), min_overlap=.035):
        ga, gn, gi = line_geometry(seg)
        choices = []
        for track in self.tracks:
            ref = track.geometry if track.stable else track.views[-1][2]
            ra, rn, ri = line_geometry(ref)
            da = angle_delta(ga, ra)
            dn = abs(gn-rn)
            ov = overlap(gi, ri)
            if da <= gate_angle and dn <= gate_normal and ov >= min_overlap:
                choices.append((dn + .10*da + .02/max(ov,.01), track))
        choices.sort(key=lambda x:x[0])
        if not choices or (len(choices)>1 and choices[1][0]-choices[0][0] < .01):
            return None
        return choices[0][1]

    def update(self, segs, pose, stamp):
        used = set(); ids=[]
        for seg in sorted(segs, key=lambda s:(-s.inliers, -s.length)):
            track = self.match(seg)
            if track is None or track.track_id in used:
                track = WallTrack(len(self.tracks))
                self.tracks.append(track)
            track.add(seg, pose, stamp)
            used.add(track.track_id); ids.append(track.track_id)
        return ids

    def stable_walls(self):
        return [KnownWallSegment(t.geometry.a, t.geometry.b, True, f'wall:{t.track_id}')
                for t in self.tracks if t.stable and t.conflicts == 0]


def _topology_hint(seg, field_n=7):
    """Post-run label only: return nearest ideal wall-edge hint, never identity."""
    angle, normal, interval = line_geometry(seg)
    midq = sum(interval)/2
    candidates=[]
    vertical=abs(abs(angle)-math.pi/2) < math.radians(15)
    horizontal=abs(angle) < math.radians(15)
    if vertical:
        k=round(normal/CELL_SIZE_M); j=math.floor(midq/CELL_SIZE_M)
        if 0<=k<=field_n and 0<=j<field_n:
            candidates.append(('V',k,j))
    if horizontal:
        k=round(normal/CELL_SIZE_M); i=math.floor(midq/CELL_SIZE_M)
        if 0<=k<=field_n and 0<=i<field_n:
            candidates.append(('H',k,i))
    return candidates[0] if len(candidates)==1 else None


def replay(session, log_path, summary_path, truth_path=None):
    if log_path.parent != Path('/tmp') or summary_path.parent != Path('/tmp'):
        raise ValueError('outputs must be directly under /tmp')
    db, topics, values, odoms, extrinsic, anchor = _read_bag(session)
    maps = {'corrected': TrackMap(), 'raw_odom': TrackMap()}
    counts=Counter(); rows=[]; prev_odom=prev_corrected=None; first_odom=None
    stamp0=None; corrections=[]; correction_after_orthogonal=None
    try:
        for _,blob in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                                 (topics[values['scan_topic']][0],)):
            scan=audit.decode_laser_scan(blob)
            try: odom=interpolate_bounded_pose(odoms,scan['stamp'])
            except ValueError: counts['no_odom_bracket']+=1; continue
            if first_odom is None:
                first_odom=odom; pred=anchor; base=anchor; stamp0=scan['stamp']
            else:
                delta=audit.compose(audit.inverse(_tf(prev_odom)),_tf(odom))
                pred=_pose(audit.compose(_tf(prev_corrected),delta))
                base=_pose(audit.compose(_tf(anchor),audit.compose(audit.inverse(_tf(first_odom)),_tf(odom))))
            _,_,local_pts=audit.scan_frame(scan,Pose2D(0.,0.,0.),_tf(Pose2D(0.,0.,0.)),extrinsic)
            c,s=math.cos(pred.yaw),math.sin(pred.yaw)
            pts=[ProjectedEndpoint(pred.x+c*p.x-s*p.y,pred.y+s*p.x+c*p.y)
                 for p in local_pts]
            prior=maps['corrected'].stable_walls()
            proposal=propose_pose_correction(pts,prior,pred,PoseCorrectionConfig())
            current=proposal.corrected_pose if proposal.accepted else pred
            if proposal.accepted:
                counts['accepted_corrections']+=1
                corrections.append((scan['stamp'], proposal.dx, proposal.dy, proposal.dyaw))
            # Fit local endpoints once; reproject the same measured geometry
            # under corrected and raw-odom poses for the two causal maps.
            basepts=[(p.x,p.y) for p in local_pts]
            local_segments=extract_segments(basepts)
            corr_segments=[_global_segment(seg,current) for seg in local_segments]
            # Baseline uses its own causal map built from uncorrected anchor+odom.
            bsegs=[_global_segment(seg,base) for seg in local_segments]
            c_matches=[maps['corrected'].match(s) for s in corr_segments]
            b_matches=[maps['raw_odom'].match(s) for s in bsegs]
            c_assoc=sum(t is not None for t in c_matches); b_assoc=sum(t is not None for t in b_matches)
            prior_angles=[math.atan2(w.end[1]-w.start[1],w.end[0]-w.start[0])%math.pi for w in prior]
            independent=any(angle_delta(a,b)>math.radians(25)
                            for i,a in enumerate(prior_angles) for b in prior_angles[i+1:])
            if independent and correction_after_orthogonal is None: correction_after_orthogonal=scan['stamp']
            rows.append({'stamp':scan['stamp'],'elapsed_s':scan['stamp']-stamp0,
                'corrected_pose':[current.x,current.y,current.yaw], 'raw_pose':[base.x,base.y,base.yaw],
                'fit_segments':len(local_segments),'prior_stable_tracks':len(prior),
                'prior_orthogonal_tracks':independent,'correction':{'accepted':proposal.accepted,'reason':proposal.reason,
                    'dx':proposal.dx,'dy':proposal.dy,'dyaw':proposal.dyaw,'inliers':proposal.n_inliers},
                'corrected_track_associations':c_assoc,'baseline_track_associations':b_assoc,
                'corrected_track_ids':[t.track_id if t else None for t in c_matches],
                'baseline_track_ids':[t.track_id if t else None for t in b_matches]})
            counts['frames']+=1; counts['segments']+=len(local_segments)
            maps['corrected'].update(corr_segments,(current.x,current.y,current.yaw),scan['stamp'])
            maps['raw_odom'].update(bsegs,(base.x,base.y,base.yaw),scan['stamp'])
            prev_odom,prev_corrected=odom,current
    finally:
        db.close()
    # Stability is measured by how often each causal map uniquely reuses a
    # prior physical track, summarized in the second half of the replay.
    half=max((r['elapsed_s'] for r in rows),default=0)/2
    second=[r for r in rows if r['elapsed_s']>=half]
    summary={'status':'OFFLINE_EXPERIMENT_ONLY','session':str(session),
      'model':'continuous finite physical wall tracks; angle+normal+finite overlap association; no grid coordinate snapping',
      'counts':dict(counts),'stable_tracks':{
        name:sum(t.stable and t.conflicts==0 for t in m.tracks) for name,m in maps.items()},
      'track_counts':{name:len(m.tracks) for name,m in maps.items()},
      'stable_track_details':{name:[{'track_id':t.track_id,'independent_view_count':len(t.views),
          'conflicts':t.conflicts,'normal_coordinate_m':line_geometry(t.geometry)[1],
          'angle_rad':line_geometry(t.geometry)[0],'supported_span_m':t.geometry.length}
          for t in m.tracks if t.stable] for name,m in maps.items()},
      'orthogonal_stable_track_pairs':{name:sum(
          angle_delta(line_geometry(a.geometry)[0],line_geometry(b.geometry)[0])>math.radians(25)
          for i,a in enumerate(m.tracks) if a.stable and a.conflicts==0
          for b in m.tracks[i+1:] if b.stable and b.conflicts==0)
          for name,m in maps.items()},
      'second_half_association':{
        name:{'frames':len(second),'matched_segment_fraction':sum(r[f'{key}_track_associations'] for r in second)/max(1,sum(r['fit_segments'] for r in second)),
              'mean_associated_segments_per_frame':sum(r[f'{key}_track_associations'] for r in second)/max(1,len(second))}
        for name,key in (('corrected','corrected'),('raw_odom','baseline'))},
      'orthogonal_stable_tracks_first_seen_elapsed_s':None if correction_after_orthogonal is None else correction_after_orthogonal-stamp0,
      'correction_count':len(corrections),
      'accepted_correction_abs_p50_p90_max':{
          axis:{'p50':audit.quantile([abs(v[i]) for v in corrections],.5),
                'p90':audit.quantile([abs(v[i]) for v in corrections],.9),
                'max':max(abs(v[i]) for v in corrections)}
          for axis,i in (('dx_m',1),('dy_m',2),('dyaw_rad',3))} if corrections else None,
      'limitations':['viewpoint independence uses wheel odometry, so odometry drift can still counterfeit spatial diversity',
        'continuous track ID association is greedy and can fragment or merge tracks after large drift',
        'no independently surveyed pose or wall coordinates; topology only used post-run for conditional scoring',
        'segment extraction/replay is offline and never controls the robot']}
    if truth_path:
        truth=json.loads(truth_path.read_text())
        expected=audit.edge_truth(truth)
        track_labels={}
        for name,m in maps.items():
            hints=[]
            truth_tracks=[]
            for t in m.tracks:
                if t.stable and t.geometry:
                    hint=_topology_hint(t.geometry)
                    hints.append(hint)
                    edge=(edge_id_for_line(hint[0],hint[1],hint[2]) if hint else None)
                    truth_value=expected.get(edge) if edge is not None else None
                    truth_tracks.append({'track_id':t.track_id,'view_count':len(t.views),
                        'conflicts':t.conflicts,'topology_hint':hint,
                        'posthoc_truth_wall':truth_value})
                    if t.conflicts==0:
                        track_labels[(name,t.track_id)]=truth_value
            summary.setdefault('post_run_topology_hints',{})[name]={
                'hint_count':sum(h is not None for h in hints),'hints':[h for h in hints if h is not None],
                'stable_track_truth':truth_tracks,
                'truth_use':'post-run only; hint values never enter track association, promotion, or correction'}
        for name,key in (('corrected','corrected'),('raw_odom','baseline')):
            true_samples=false_samples=0
            for row in second:
                for tid in row[f'{key}_track_ids']:
                    val=track_labels.get((name,tid)) if tid is not None else None
                    true_samples += (val is True)
                    false_samples += (val is False)
            summary['second_half_association'][name].update({
                'stable_track_true_samples_posthoc':true_samples,
                'stable_track_false_samples_posthoc':false_samples,
                'stable_track_conditional_precision_posthoc':true_samples/(true_samples+false_samples)
                    if true_samples+false_samples else None})
    log_path.write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in rows))
    summary_path.write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,default=ROOT/'field_data/20261001_174719_joystick_full_maze')
    p.add_argument('--log',type=Path,default=Path('/tmp/honor-cup-physical-wall-tracks.jsonl'))
    p.add_argument('--summary',type=Path,default=Path('/tmp/honor-cup-physical-wall-tracks-summary.json'))
    p.add_argument('--truth-score',type=Path,default=ROOT/'field/maze_truth_7x7.json')
    a=p.parse_args()
    print(json.dumps(replay(a.session,a.log,a.summary,a.truth_score),indent=2,ensure_ascii=False))

if __name__=='__main__': main()
