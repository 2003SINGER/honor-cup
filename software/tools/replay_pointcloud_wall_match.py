#!/usr/bin/env python3
"""Offline experiment: fit wall segments directly from /scan_multi endpoints.

STATUS: DIAGNOSTIC
Reason: point-cloud baseline; current replays also import its geometry helpers.

Unlike LaserScan-ray association, this treats finite ranges as a base-frame
point cloud. No free-space/open inference is made. Grid truth is consulted
only by the optional post-replay score.
"""
from __future__ import annotations

import argparse
import ast
import json
import math
import random
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))

import audit_lidar_odom_calibration as audit
from analyze_scan_edge import CELL_SIZE_M as CELL, Pose2D
from replay_continuous_wall_geometry import (_read_bag, _tf, _pose,
    interpolate_bounded_pose)
from m3pro_nav.pose_correction import (KnownWallSegment, PoseCorrectionConfig,
    ProjectedEndpoint, propose_pose_correction)


@dataclass(frozen=True)
class Segment:
    """Finite line segment with unit tangent/normal, span, and fit residual."""
    a: tuple[float, float]
    b: tuple[float, float]
    inliers: int
    rms: float
    support: tuple[tuple[float, float], ...] = ()

    @property
    def tangent(self):
        dx, dy = self.b[0]-self.a[0], self.b[1]-self.a[1]
        n = math.hypot(dx, dy)
        return dx/n, dy/n

    @property
    def length(self):
        return math.dist(self.a, self.b)


def _fit_tls(points):
    mx = sum(p[0] for p in points)/len(points)
    my = sum(p[1] for p in points)/len(points)
    xx = sum((p[0]-mx)**2 for p in points)
    yy = sum((p[1]-my)**2 for p in points)
    xy = sum((p[0]-mx)*(p[1]-my) for p in points)
    theta = .5*math.atan2(2*xy, xx-yy)
    tx, ty = math.cos(theta), math.sin(theta)
    nx, ny = -ty, tx
    residuals = [(p[0]-mx)*nx+(p[1]-my)*ny for p in points]
    lo = min((p[0]-mx)*tx+(p[1]-my)*ty for p in points)
    hi = max((p[0]-mx)*tx+(p[1]-my)*ty for p in points)
    return ((mx+lo*tx, my+lo*ty), (mx+hi*tx, my+hi*ty),
            math.sqrt(sum(r*r for r in residuals)/len(residuals)))


def extract_segments(points, *, threshold=.018, min_inliers=6,
                     min_span=.12, max_gap=.075, seed=17, max_lines=16):
    """Deterministic pair-RANSAC, TLS refit, then split at point-cloud gaps."""
    remaining = list(dict.fromkeys((float(x), float(y)) for x, y in points))
    rng = random.Random(seed)
    result = []
    while len(remaining) >= min_inliers and len(result) < max_lines:
        best = []
        pairs = [(i, j) for i in range(len(remaining))
                 for j in range(i+1, len(remaining))]
        if len(pairs) > 1200:
            pairs = rng.sample(pairs, 1200)
        for i, j in pairs:
            ax, ay = remaining[i]; bx, by = remaining[j]
            dx, dy = bx-ax, by-ay
            norm = math.hypot(dx, dy)
            if norm < min_span: continue
            nx, ny = -dy/norm, dx/norm
            inliers = [p for p in remaining
                       if abs((p[0]-ax)*nx+(p[1]-ay)*ny) <= threshold]
            if len(inliers) > len(best): best = inliers
        if len(best) < min_inliers: break
        a, b, _ = _fit_tls(best)
        tx, ty = (b[0]-a[0])/math.dist(a,b), (b[1]-a[1])/math.dist(a,b)
        mx = sum(p[0] for p in best)/len(best); my = sum(p[1] for p in best)/len(best)
        ordered = sorted(best, key=lambda p: (p[0]-mx)*tx+(p[1]-my)*ty)
        groups, group = [], [ordered[0]]
        for p in ordered[1:]:
            if math.dist(p, group[-1]) > max_gap:
                groups.append(group); group = []
            group.append(p)
        groups.append(group)
        consumed = set(best)
        remaining = [p for p in remaining if p not in consumed]
        for group in groups:
            if len(group) < min_inliers: continue
            a, b, rms = _fit_tls(group)
            seg = Segment(a, b, len(group), rms, tuple(group))
            if seg.length >= min_span and rms <= threshold:
                result.append(seg)
    return sorted(result, key=lambda s: (-s.inliers, s.a))


def _global_segment(seg, pose):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    def tr(p): return (pose.x+c*p[0]-s*p[1], pose.y+s*p[0]+c*p[1])
    return Segment(tr(seg.a), tr(seg.b), seg.inliers, seg.rms,
                   tuple(tr(p) for p in seg.support))


def _repose_segment(seg, source, target):
    """Rigidly change a maze-frame segment from source pose to target pose."""
    cs,ss=math.cos(source.yaw),math.sin(source.yaw)
    ct,st=math.cos(target.yaw),math.sin(target.yaw)
    def tr(p):
        rx=p[0]-source.x; ry=p[1]-source.y
        bx=cs*rx+ss*ry; by=-ss*rx+cs*ry
        return target.x+ct*bx-st*by,target.y+st*bx+ct*by
    return Segment(tr(seg.a),tr(seg.b),seg.inliers,seg.rms,
                   tuple(tr(p) for p in seg.support))


def split_by_grid_cells(seg, *, min_points=6, min_span=.08):
    """Split at 0.4m cell boundaries; refit each piece from its own hits."""
    tx,ty=seg.tangent
    angle=math.atan2(ty,tx)%math.pi
    vertical=abs(angle-math.pi/2)<=math.radians(15)
    horizontal=min(angle,math.pi-angle)<=math.radians(15)
    if not (vertical or horizontal): return [seg]
    coord=(lambda p:p[1]) if vertical else (lambda p:p[0])
    lo,hi=sorted((coord(seg.a),coord(seg.b)))
    cuts=[lo]+[k*CELL for k in range(math.floor(lo/CELL)+1,math.ceil(hi/CELL))
               if lo+1e-6<k*CELL<hi-1e-6]+[hi]
    if len(cuts)<=2: return [seg]
    pieces=[]
    for a,b in zip(cuts,cuts[1:]):
        support=[p for p in seg.support if a-1e-6<=coord(p)<=b+1e-6]
        if len(support)<min_points: continue
        pa,pb,rms=_fit_tls(support)
        piece=Segment(pa,pb,len(support),rms,tuple(support))
        if piece.length>=min_span and rms<=.018: pieces.append(piece)
    return pieces


def segment_error(observed, reference):
    """Angle and normal-coordinate error between two fitted infinite lines."""
    ot=observed.tangent; rt=reference.tangent
    da=abs(math.atan2(ot[0]*rt[1]-ot[1]*rt[0],ot[0]*rt[0]+ot[1]*rt[1]))
    da=min(da,math.pi-da)  # wall tangent is an undirected axis
    nx,ny=-rt[1],rt[0]
    om=((observed.a[0]+observed.b[0])/2,(observed.a[1]+observed.b[1])/2)
    rm=((reference.a[0]+reference.b[0])/2,(reference.a[1]+reference.b[1])/2)
    normal=abs((om[0]-rm[0])*nx+(om[1]-rm[1])*ny)
    # Overlap on the reference tangent protects against matching a remote
    # collinear segment with the same discrete edge label.
    rt0=(reference.a[0]*rt[0]+reference.a[1]*rt[1]); rt1=(reference.b[0]*rt[0]+reference.b[1]*rt[1])
    ot0=min(observed.a[0]*rt[0]+observed.a[1]*rt[1],observed.b[0]*rt[0]+observed.b[1]*rt[1])
    ot1=max(observed.a[0]*rt[0]+observed.a[1]*rt[1],observed.b[0]*rt[0]+observed.b[1]*rt[1])
    overlap=max(0.,min(rt1,ot1)-max(rt0,ot0))
    return normal,da,overlap


def wall_residuals(points, walls, *, ambiguity=.01):
    """Point-to-finite-segment normal residual; ambiguous/endpoint hits abstain."""
    values=[]
    for p in points:
        candidates=[]
        for w in walls:
            dx,dy=w.end[0]-w.start[0],w.end[1]-w.start[1]
            length=math.hypot(dx,dy); tx,ty=dx/length,dy/length; nx,ny=-ty,tx
            along=(p.x-w.start[0])*tx+(p.y-w.start[1])*ty
            if not .03 <= along <= length-.03: continue
            residual=(p.x-w.start[0])*nx+(p.y-w.start[1])*ny
            candidates.append((abs(residual),residual))
        candidates.sort(key=lambda x:x[0])
        if not candidates or (len(candidates)>1 and candidates[1][0]-candidates[0][0]<ambiguity):
            continue
        values.append(abs(candidates[0][1]))
    return values


def associate_segment(seg, *, angle_gate=math.radians(15), normal_gate=.10,
                      ambiguity_gap=.025, field_n=7):
    """Associate a fitted finite segment to one nearby grid edge or abstain."""
    tx, ty = seg.tangent
    angle = math.atan2(ty, tx) % math.pi
    candidates = []; direction_compatible=False; in_field=False; distance_compatible=False
    mx, my = (seg.a[0]+seg.b[0])/2, (seg.a[1]+seg.b[1])/2
    # Grid-aligned walls only for identity; measured segment orientation/span
    # remain continuous and are retained in history.
    for orient, target, coord in (('V', math.pi/2, mx), ('H', 0., my)):
        da = abs((angle-target+math.pi/2) % math.pi-math.pi/2)
        if da > angle_gate: continue
        direction_compatible=True
        k0 = round(coord/CELL)
        for k in range(max(0,k0-1), min(field_n,k0+1)+1):
            residual = abs(coord-k*CELL)
            along = my if orient == 'V' else mx
            j = math.floor(along/CELL)
            if j < 0 or j >= field_n: continue
            in_field=True
            if residual > normal_gate: continue
            distance_compatible=True
            from m3pro_nav.grid_association import edge_id_for_line
            eid = edge_id_for_line(orient, k, j)
            score = residual + .08*da
            candidates.append((score, eid, orient, k*CELL, da))
    candidates.sort(key=lambda x:x[0])
    if not candidates:
        return None, ('NO_IN_FIELD_GRID_LINE' if direction_compatible and not in_field
                      else 'GRID_LINE_DISTANCE' if direction_compatible and in_field and not distance_compatible
                      else 'LINE_DIRECTION'), None
    if len(candidates)>1 and candidates[1][0]-candidates[0][0] < ambiguity_gap:
        return None, 'AMBIGUOUS_GRID_MATCH', candidates[:2]
    return candidates[0], 'UNIQUE', candidates


@dataclass
class History:
    edge_id: tuple
    views: list = field(default_factory=list)
    stable: bool = False
    conflicts: int = 0
    segment: Segment | None = None

    def add(self, seg, pose, stamp):
        # Store the scan's measured infinite-line normal and segment bounds,
        # along with odom viewpoint. Cluster repeated near-identical views.
        tx, ty = seg.tangent
        angle = math.atan2(ty,tx) % math.pi
        mx,my=(seg.a[0]+seg.b[0])/2,(seg.a[1]+seg.b[1])/2
        nx,ny=-math.sin(angle),math.cos(angle)
        normal=mx*nx+my*ny
        view=[pose.x,pose.y,pose.yaw,stamp,mx,my,angle,seg.a,seg.b,normal]
        if any(math.hypot(v[0]-pose.x,v[1]-pose.y)<.15 and
               abs(math.atan2(math.sin(v[2]-pose.yaw),math.cos(v[2]-pose.yaw)))<math.radians(15)
               for v in self.views): return
        self.views.append(view)
        if self.stable:
            old=self.segment
            if old:
                ot=old.tangent; nt=seg.tangent
                da=abs(math.atan2(ot[0]*nt[1]-ot[1]*nt[0],ot[0]*nt[0]+ot[1]*nt[1]))
                on=(-ot[1],ot[0]); ox=(old.a[0]+old.b[0])/2; oy=(old.a[1]+old.b[1])/2
                sx=(seg.a[0]+seg.b[0])/2; sy=(seg.a[1]+seg.b[1])/2
                normal=abs((sx-ox)*on[0]+(sy-oy)*on[1])
                # Segment endpoints vary with visibility; compare line support,
                # not endpoint overlap, so partial views do not create conflict.
                if da>math.radians(4) or normal>.035: self.conflicts+=1
            return
        separated=[v for v in self.views if all(
            (math.hypot(v[0]-other[0],v[1]-other[1])>=.15 or
             abs(math.atan2(math.sin(v[2]-other[2]),math.cos(v[2]-other[2])))>=math.radians(15))
            for other in self.views if other is not v)]
        if len(separated)<4: return
        # robust line representative from view medians, veto inconsistent
        # orientation/normal before promoting.
        angles=[v[6] for v in separated]
        a0=angles[0]
        aligned=[a0+math.atan2(math.sin(a-a0),math.cos(a-a0)) for a in angles]
        if max(aligned)-min(aligned)>math.radians(4): return
        normals=[v[9] for v in separated]
        med=sorted(normals)[len(normals)//2]
        if max(abs(x-med) for x in normals)>.035: return
        # Preserve observed orientation and span union; pose corrections use
        # only historical segments and the fitted segment itself.
        pts=[p for v in separated for p in (v[7],v[8])]
        # Align line direction with the representative orientation.
        tx,ty=math.cos(sum(aligned)/len(aligned)),math.sin(sum(aligned)/len(aligned))
        nx,ny=-ty,tx
        cx=sum(p[0] for p in pts)/len(pts); cy=sum(p[1] for p in pts)/len(pts)
        offsets=[(p[0]-cx)*nx+(p[1]-cy)*ny for p in pts]
        off=sorted(offsets)[len(offsets)//2]
        cx-=off*nx; cy-=off*ny
        along=[(p[0]-cx)*tx+(p[1]-cy)*ty for p in pts]
        self.segment=Segment((cx+min(along)*tx,cy+min(along)*ty),
                             (cx+max(along)*tx,cy+max(along)*ty),len(pts),0.)
        self.stable=True


def multiview_bootstrap_probe(rows, expected, *, window_s=30.):
    """Post-run first-window audit; never feeds line IDs or truth into replay."""
    if not rows: return {'window_s':window_s,'candidate_edges':0,'seed_edges':[]}
    t0=rows[0]['stamp']; per_edge={}
    for row in rows:
        if row['stamp']-t0>window_s: break
        # At most one segment vote per edge per scan, even when a scan has
        # duplicate line fits or multiple supported pieces.
        best={}
        for seg in row['fitted_segments']:
            edge=seg['edge_id']
            if seg['status']=='UNIQUE' and edge and seg['inliers']>=6 and seg['span_m']>=.08:
                if edge not in best or seg['inliers']>best[edge]['inliers']: best[edge]=seg
        for edge,seg in best.items(): per_edge.setdefault(edge,[]).append((row,seg))
    diagnostics=[]; seeds=[]
    for edge,obs in per_edge.items():
        clusters=[]
        for row,seg in obs:
            pose=row['pose']
            idx=next((i for i,c in enumerate(clusters)
              if math.hypot(pose[0]-c['pose'][0],pose[1]-c['pose'][1])<.15 and
                 abs(math.atan2(math.sin(pose[2]-c['pose'][2]),math.cos(pose[2]-c['pose'][2])))<math.radians(15)),None)
            if idx is None: clusters.append({'pose':pose,'row':row,'seg':seg})
            elif seg['inliers']>clusters[idx]['seg']['inliers']: clusters[idx]={'pose':pose,'row':row,'seg':seg}
        angles=[c['seg']['line_angle_rad'] for c in clusters]
        a0=angles[0]; aligned=[a0+math.atan2(math.sin(2*(a-a0)),math.cos(2*(a-a0)))/2 for a in angles]
        angle_span=max(aligned)-min(aligned); angle=sum(aligned)/len(aligned)
        tx,ty=math.cos(angle),math.sin(angle); nx,ny=-ty,tx
        offsets=[]; intervals=[]
        for c in clusters:
            seg=c['seg']; mx,my=seg['midpoint']; half=seg['span_m']/2
            offsets.append(mx*nx+my*ny)
            along=mx*tx+my*ty
            intervals.append((along-half,along+half))
        normal_span=max(offsets)-min(offsets)
        common_span=min(x[1] for x in intervals)-max(x[0] for x in intervals)
        seed=(len(clusters)>=4 and normal_span<=.035 and angle_span<=math.radians(4)
              and common_span>=.08)
        try: edge_id=ast.literal_eval(edge)
        except (ValueError,SyntaxError): edge_id=None
        if seed and edge_id is not None: seeds.append(edge_id)
        diagnostics.append({'edge_id':edge,'scan_frames':len(obs),'independent_pose_clusters':len(clusters),
          'normal_span_m':normal_span,'angle_span_deg':math.degrees(angle_span),
          'common_supported_span_m':max(0.,common_span),'seed':seed})
    tp=sum(expected.get(e) is True for e in seeds); fp=sum(expected.get(e) is False for e in seeds)
    return {'window_s':window_s,'sampling':'anchor plus short-term odometry; pose clusters use >=0.15m displacement OR >=15deg heading; no time dwell gate',
      'per_frame_edge_deduplication':'one highest-inlier segment per edge per scan',
      'thresholds':'>=4 pose clusters; normal span <=35mm; angle span <=4deg; common supported segment >=80mm',
      'candidate_edge_count':len(per_edge),'seed_count':len(seeds),'seed_true_edges_posthoc':tp,
      'seed_false_edges_posthoc':fp,'seed_precision_posthoc':tp/(tp+fp) if tp+fp else None,
      'orientation_coverage':{'vertical_EW':sum(e[-1] in ('E','W') for e in seeds),
                              'horizontal_NS':sum(e[-1] in ('N','S') for e in seeds)},
      'candidate_diagnostics':sorted(diagnostics,key=lambda x:-x['independent_pose_clusters'])}


def replay(session, log_path, summary_path, truth_path=None):
    if log_path.parent != Path('/tmp') or summary_path.parent != Path('/tmp'):
        raise ValueError('outputs must be directly under /tmp')
    db,topics,values,odoms,extrinsic,corrected=_read_bag(session)
    anchor_pose=corrected; first_odom=None; prev_odom=prev_corrected=None
    hist={}; counts=Counter(); last_stamp=None; candidate_lines=Counter()
    holdout={'frames':0,'corrected_normal':[],'baseline_normal':[],
             'corrected_angle':[],'baseline_angle':[],'matched_segments':0}
    try:
        with log_path.open('w') as out:
            for _,blob in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(topics[values['scan_topic']][0],)):
                scan=audit.decode_laser_scan(blob)
                try: odom=interpolate_bounded_pose(odoms,scan['stamp'])
                except ValueError: counts['no_odom_bracket']+=1; continue
                if prev_odom is None: predicted=corrected; first_odom=odom
                else:
                    delta=audit.compose(audit.inverse(_tf(prev_odom)),_tf(odom))
                    predicted=_pose(audit.compose(_tf(prev_corrected),delta))
                # /scan_multi frame_id=base_link: each finite range is one
                # fused endpoint in vehicle coordinates. _project applies
                # only the recorded static base frame transform.
                # predicted is already in maze coordinates (manual anchor
                # composed with odom delta). Pass it exactly once as the
                # maze_from_odom transform; points returned are maze-frame.
                _,_,points=audit.scan_frame(scan,Pose2D(0.,0.,0.),_tf(predicted),extrinsic)
                projected=[(p.x,p.y) for p in points]
                fitted_segments=extract_segments(projected)
                counts['raw_segments_over_cell']+=sum(s.length>CELL for s in fitted_segments)
                counts['raw_segments_total']+=len(fitted_segments)
                predicted_segments=[piece for s in fitted_segments for piece in split_by_grid_cells(s)]
                counts['grid_supported_pieces']+=len(predicted_segments)
                prior=[h.segment for h in hist.values() if h.stable and h.conflicts==0]
                walls=[KnownWallSegment(s.a,s.b,True,repr(e)) for e,h in hist.items()
                       if h.stable and h.conflicts==0 and (s:=h.segment)]
                baseline_delta=audit.compose(audit.inverse(_tf(first_odom)),_tf(odom))
                baseline_pose=_pose(audit.compose(_tf(anchor_pose),baseline_delta))
                proposal=propose_pose_correction(points,walls,predicted,PoseCorrectionConfig())
                current=proposal.corrected_pose if proposal.accepted else predicted
                if proposal.accepted: counts['accepted_corrections']+=1
                # Reproject and fit/associate after correction, then update history.
                _,_,corrected_points=audit.scan_frame(scan,Pose2D(0.,0.,0.),_tf(current),extrinsic)
                segs=[_repose_segment(s,predicted,current) for s in predicted_segments]
                if walls:
                    prior_by_id={e:h for e,h in hist.items() if h.stable and h.conflicts==0}
                    cpair={}; bpair={}
                    for raw_seg,cseg in zip(predicted_segments,segs):
                        bseg=_repose_segment(raw_seg,predicted,baseline_pose)
                        cm,_,_=associate_segment(cseg); bm,_,_=associate_segment(bseg)
                        if not cm or not bm or cm[1]!=bm[1] or cm[1] not in prior_by_id: continue
                        ref=prior_by_id[cm[1]].segment
                        ce,ca,co=segment_error(cseg,ref); be,ba,bo=segment_error(bseg,ref)
                        if min(co,bo)>=.06:
                            cpair[cm[1]]=(ce,ca); bpair[bm[1]]=(be,ba)
                    if cpair and bpair:
                        holdout['frames']+=1
                        holdout['corrected_normal'].extend(v[0] for v in cpair.values())
                        holdout['baseline_normal'].extend(v[0] for v in bpair.values())
                        holdout['corrected_angle'].extend(v[1] for v in cpair.values())
                        holdout['baseline_angle'].extend(v[1] for v in bpair.values())
                        holdout['matched_segments']+=len(cpair)
                associations=[]
                for seg in segs:
                    match,status,_=associate_segment(seg)
                    mx=(seg.a[0]+seg.b[0])/2; my=(seg.a[1]+seg.b[1])/2
                    associations.append({'status':status,'edge_id':repr(match[1]) if match else None,
                        'line_angle_rad':math.atan2(seg.tangent[1],seg.tangent[0]),
                        'normal_coordinate_m':(mx if match and match[2]=='V' else my) if match else None,
                        'midpoint':[mx,my],'span_m':seg.length,'rms_m':seg.rms,'inliers':seg.inliers})
                    if match:
                        item=hist.setdefault(match[1],History(match[1]))
                        was=item.stable; item.add(seg,odom,scan['stamp'])
                        if not was and item.stable: counts['stable_promotions']+=1
                counts['scans']+=1; counts['fitted_segments']+=len(segs)
                counts['ambiguous_segments']+=sum(a['status']=='AMBIGUOUS_GRID_MATCH' for a in associations)
                counts['unmatched_segments']+=sum(a['status'] in ('LINE_DIRECTION','GRID_LINE_DISTANCE','NO_IN_FIELD_GRID_LINE') for a in associations)
                counts['direction_rejections']+=sum(a['status']=='LINE_DIRECTION' for a in associations)
                counts['grid_distance_rejections']+=sum(a['status']=='GRID_LINE_DISTANCE' for a in associations)
                counts['out_of_field_rejections']+=sum(a['status']=='NO_IN_FIELD_GRID_LINE' for a in associations)
                out.write(json.dumps({'stamp':scan['stamp'],'endpoint_count':len(points),
                    'fitted_segments':associations,'prior_stable_edges':len(walls),
                    'correction':{'accepted':proposal.accepted,'reason':proposal.reason,
                      'dx_m':proposal.dx,'dy_m':proposal.dy,'dyaw_rad':proposal.dyaw,
                      'associated_hits':proposal.n_associated,'inliers':proposal.n_inliers},
                    'pose':[current.x,current.y,current.yaw],
                    'raw_fitted_segments':len(fitted_segments)},separators=(',',':'))+'\n')
                prev_odom,prev_corrected=odom,current; corrected=current; last_stamp=scan['stamp']
    finally: db.close()
    stable=[h for h in hist.values() if h.stable]
    spread=[]
    for h in hist.values():
        if h.views:
            vals=[v[9] for v in h.views]
            spread.append({'edge_id':repr(h.edge_id),'observations':len(vals),
              'normal_span_m':max(vals)-min(vals),'normal_median_m':sorted(vals)[len(vals)//2],
              'stable':h.stable})
    summary={'result':'OFFLINE_EXPERIMENT_ONLY','session':session.name,
      'input_semantics':'/scan_multi finite endpoints in base_link; interpreted as point cloud; no OPEN/free-space claims',
      'algorithm':'pair-RANSAC line extraction, TLS refit, gap segmentation, local discrete edge association; history-only correction before current-frame promotion',
      'counts':dict(counts),'accepted_pose_corrections':counts['accepted_corrections'],
      'long_segment_handling':{'raw_segments_over_0_4m':counts['raw_segments_over_cell'],
        'raw_segment_total':counts['raw_segments_total'],
        'fraction_over_0_4m':counts['raw_segments_over_cell']/counts['raw_segments_total'] if counts['raw_segments_total'] else None,
        'grid_supported_pieces_after_split':counts['grid_supported_pieces'],
        'policy':'split at 0.4m tangent coordinates; refit each piece from hits in its interval; discard pieces with <6 hits, <0.08m span, or >18mm RMS'},
      'stable_edges':len(stable),
      'candidate_edge_geometry_spread':sorted(spread,key=lambda x:-x['observations']),
      'historical_wall_holdout':{'frames_evaluated':holdout['frames'],
        'matched_segments_lower_bound':holdout['matched_segments'],
        'corrected_abs_normal_residual_p50_p90_m':[audit.quantile(holdout['corrected_normal'],q) for q in (.5,.9)] if holdout['corrected_normal'] else None,
        'anchor_plus_odom_abs_normal_residual_p50_p90_m':[audit.quantile(holdout['baseline_normal'],q) for q in (.5,.9)] if holdout['baseline_normal'] else None,
        'corrected_angle_error_p50_p90_rad':[audit.quantile(holdout['corrected_angle'],q) for q in (.5,.9)] if holdout['corrected_angle'] else None,
        'anchor_plus_odom_angle_error_p50_p90_rad':[audit.quantile(holdout['baseline_angle'],q) for q in (.5,.9)] if holdout['baseline_angle'] else None,
        'comparison':'same-frame fitted segment with matching historical edge ID vs frozen historical continuous segment; truth independent',
        'evidence':'not evaluable: no stable prior wall set' if not holdout['frames'] else 'observational holdout; frames after wall promotion only'},
      'stable_edge_ids':[repr(h.edge_id) for h in stable],
      'stable_edge_conflicts':sum(h.conflicts>0 for h in stable),
      'limitations':['Manhattan grid used only to assign discrete edge IDs; fitted angle/span retained continuously',
        'view diversity uses wheel odometry and may be corrupted by drift',
        'no independently surveyed physical pose or walls; topology scores are conditional on session anchor',
        'prototype is offline and never controls the robot']}
    if truth_path:
        truth=json.loads(truth_path.read_text()); expected=audit.edge_truth(truth)
        found={h.edge_id for h in stable}; tp=sum(expected.get(e) is True for e in found)
        fp=sum(expected.get(e) is False for e in found); fn=sum(v is True and e not in found for e,v in expected.items())
        summary['post_replay_topology_score']={'true_positive':tp,'false_positive':fp,'missed_true_wall_edges':fn,
          'precision':tp/(tp+fp) if tp+fp else None,'recall':tp/(tp+fn) if tp+fn else None}
        by_repr={repr(k):v for k,v in expected.items()}
        rows=[json.loads(line) for line in log_path.read_text().splitlines()]
        t0=rows[0]['stamp'] if rows else 0.
        bins={}
        for lo in range(0,181,30):
            samples=[]
            for row in rows:
                elapsed=row['stamp']-t0
                if lo<=elapsed<lo+30:
                    # One repeated-sample vote per discrete edge per frame.
                    best={}
                    for seg in row['fitted_segments']:
                        edge=seg['edge_id']
                        if seg['status']!='UNIQUE' or not edge: continue
                        if edge not in best or seg['inliers']>best[edge]['inliers']:
                            best[edge]=seg
                    samples.extend((row,e,s) for e,s in best.items())
            tp_s=sum(by_repr.get(e) is True for _,e,_ in samples)
            fp_s=sum(by_repr.get(e) is False for _,e,_ in samples)
            observations={}
            for row,e,seg in samples: observations.setdefault(e,[]).append((row,seg))
            cluster_counts=[]; normal_spans=[]; angle_spans=[]
            for obs in observations.values():
                clusters=[]
                for row,seg in obs:
                    p=row['pose']; stamp=row['stamp']
                    is_new=all(
                        math.hypot(p[0]-v[0],p[1]-v[1])>=.15 or
                        abs(math.atan2(math.sin(p[2]-v[2]),math.cos(p[2]-v[2])))>=math.radians(15)
                        for v in clusters)
                    if not clusters or is_new: clusters.append((p[0],p[1],p[2],stamp))
                cluster_counts.append(len(clusters))
                coords=[s['normal_coordinate_m'] for _,s in obs if s['normal_coordinate_m'] is not None]
                if coords: normal_spans.append(max(coords)-min(coords))
                angles=[s['line_angle_rad'] for _,s in obs]
                if angles:
                    a0=angles[0]; aligned=[a0+math.atan2(math.sin(2*(a-a0)),math.cos(2*(a-a0)))/2 for a in angles]
                    angle_spans.append(max(aligned)-min(aligned))
            bins[f'{lo:03d}-{lo+30:03d}s']={'unique_segment_samples':len(samples),
                'conditional_true_positive_samples':tp_s,'conditional_false_positive_samples':fp_s,
                'sample_precision':tp_s/(tp_s+fp_s) if tp_s+fp_s else None,
                'distinct_edge_ids':len(observations),
                'repeated_frames_per_edge_p50_p90':[audit.quantile([len(v) for v in observations.values()],q) for q in (.5,.9)] if observations else None,
                'independent_pose_clusters_per_edge_p50_p90':[audit.quantile(cluster_counts,q) for q in (.5,.9)] if cluster_counts else None,
                'within_edge_normal_span_m_p50_p90':[audit.quantile(normal_spans,q) for q in (.5,.9)] if normal_spans else None,
                'within_edge_angle_span_deg_p50_p90':[math.degrees(audit.quantile(angle_spans,q)) for q in (.5,.9)] if angle_spans else None}
        summary['post_replay_segment_bootstrap']=bins
        summary['multiview_bootstrap_probe']=multiview_bootstrap_probe(rows,expected)
    summary_path.write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,default=ROOT/'field_data/20261001_174719_joystick_full_maze')
    p.add_argument('--log',type=Path,default=Path('/tmp/honor-cup-pointcloud-wall-match.jsonl'))
    p.add_argument('--summary',type=Path,default=Path('/tmp/honor-cup-pointcloud-wall-match-summary.json'))
    p.add_argument('--truth-score',type=Path,default=ROOT/'field/maze_truth_7x7.json')
    a=p.parse_args(); print(json.dumps(replay(a.session,a.log,a.summary,a.truth_score),indent=2,ensure_ascii=False))

if __name__=='__main__': main()
