#!/usr/bin/env python3
"""Offline causal per-frame Manhattan grid-snap replay for /scan_multi.

Only finite hit endpoints are used. Each frame fits local wall segments, then
estimates one shared pose correction against nearby ideal grid lines. No map
tracks or future observations participate in the estimate; truth is loaded
only after the replay has completed for optional scoring.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))

import audit_lidar_odom_calibration as audit
from analyze_scan_edge import CELL_SIZE_M, Pose2D
from replay_continuous_wall_geometry import (_read_bag, _pose, _tf,
    interpolate_bounded_pose)
from replay_pointcloud_wall_match import split_by_grid_cells, _repose_segment
from m3pro_nav.grid_association import edge_id_for_line
from m3pro_nav.frame_grid_snap import (
    Segment, extract_segments, GridSnapConfig, FrameCorrection,
    _wrap_pi, _axis_angle, _transform_segment, _median, _mad,
    solve_frame_correction)

DEFAULT_SESSION = ROOT / 'field_data' / '20261001_174719_joystick_full_maze'
DEFAULT_LOG = Path('/tmp/honor-cup-frame-grid-snap.jsonl')
DEFAULT_SUMMARY = Path('/tmp/honor-cup-frame-grid-snap-summary.json')
EDGE_ROI_HALF_WIDTH_M = .12
EDGE_ROI_HALF_WIDTHS_M = {'2': .02, '3': .03, '4': .04, '5': .05}
EDGE_ROI_CONTIGUITY_GAP_M = .06


def _edge_id_text(edge_id):
    """JSON-safe, deterministic rendering of the canonical tuple edge key."""
    return repr(edge_id)


def _fp_edge_diagnostics(per_frame):
    """Posthoc attribution of segment and supported-piece FP by map edge."""
    output={}
    for label_name in ('corrected','anchor_plus_raw_odom'):
        by_segment=Counter(); segment_samples={}
        by_piece=Counter(); piece_samples={}
        for frame in per_frame:
            for detail in frame[label_name]['details']:
                if detail['label']=='FP':
                    for edge in detail['fp_edge_ids']:
                        edge_pieces=[p for p in detail['pieces'] if p['edge_id']==edge]
                        by_segment[edge]+=1
                        segment_samples.setdefault(edge,[]).append({
                            'frame_index':frame['frame_index'],'stamp':frame['stamp'],
                            'elapsed_s':frame['elapsed_s'],
                            'pose':(frame['corrected_pose'] if label_name=='corrected'
                                    else frame['anchor_plus_raw_odom_pose']),
                            'segment_index':detail['segment_index'],
                            'edge_id':edge,'support_points':detail['support_points'],
                            'span_m':detail['span_m'],
                            'truth':'open' if edge_pieces and edge_pieces[0]['truth_wall'] is False else None,
                            'line_residuals_m':[p['line_residual_m'] for p in edge_pieces]})
                for piece in detail['pieces']:
                    if piece['label']=='FP' and piece['edge_id'] is not None:
                        edge=piece['edge_id']; by_piece[edge]+=1
                        piece_samples.setdefault(edge,[]).append({
                            'frame_index':frame['frame_index'],'stamp':frame['stamp'],
                            'elapsed_s':frame['elapsed_s'],
                            'segment_index':detail['segment_index'],
                            **piece})
        def runs(samples):
            indexes=sorted({s['frame_index'] for s in samples})
            groups=[]
            for idx in indexes:
                if not groups or idx>groups[-1][-1]+1: groups.append([idx])
                else: groups[-1].append(idx)
            return [{'start_frame_index':g[0],'end_frame_index':g[-1],
                     'frame_count':len(g)} for g in groups]
        output[label_name]={
            'segment_fp':{edge:{'count':count,'consecutive_frame_runs':runs(segment_samples[edge]),
                'samples':segment_samples[edge]} for edge,count in sorted(by_segment.items())},
            'piece_fp':{edge:{'count':count,'consecutive_frame_runs':runs(piece_samples[edge]),
                'samples':piece_samples[edge]} for edge,count in sorted(by_piece.items())}}
    return output


def _segment_grid_residual(seg, pose, cell=CELL_SIZE_M):
    """Distance of a Manhattan segment normal coordinate to nearest grid line."""
    a=_axis_angle(seg); vertical=abs(a)>math.pi/4
    if (math.pi/2-abs(a) if vertical else abs(a))>math.radians(15): return None
    coord=(seg.a[0]+seg.b[0])/2 if vertical else (seg.a[1]+seg.b[1])/2
    return abs(coord-round(coord/cell)*cell)


def _heldout_residual(segments, pose, dx=0., dy=0., dyaw=0.):
    vals=[]
    for seg in segments:
        moved=_transform_segment(seg,pose,dx,dy,dyaw)
        r=_segment_grid_residual(moved,pose)
        if r is not None: vals.append(r)
    return _median(vals)


def _pose_local_point(point, corrected_pose):
    """Project one base-frame endpoint under this frame's corrected pose."""
    ct, st = math.cos(corrected_pose.yaw), math.sin(corrected_pose.yaw)
    return (corrected_pose.x + ct*point[0] - st*point[1],
            corrected_pose.y + st*point[0] + ct*point[1])


def _edge_local_features(orientation, k, j, corrected_pose, raw_local_points,
                         *, roi_halfwidth_m=EDGE_ROI_HALF_WIDTH_M,
                         odom_yaw_rate_rad_s=None):
    """Truth-free endpoint features inside one canonical edge rectangle.

    These are geometric endpoint proxies. The heading angle is not true
    per-beam lidar incidence because the merged scan has no source-ray IDs.
    """
    lo, hi = j * CELL_SIZE_M, (j + 1) * CELL_SIZE_M
    grid_normal = k * CELL_SIZE_M
    vertical = orientation == 'V'
    endpoints = []
    for point in raw_local_points:
        x, y = _pose_local_point(point, corrected_pose)
        tangent = y if vertical else x
        normal = x if vertical else y
        if lo <= tangent < hi and abs(normal - grid_normal) <= roi_halfwidth_m:
            endpoints.append((tangent, normal - grid_normal))
    roi = {}
    for cm, halfwidth in EDGE_ROI_HALF_WIDTHS_M.items():
        selected = sorted(t for t, offset in endpoints if abs(offset) <= halfwidth)
        max_gap = max((b - a for a, b in zip(selected, selected[1:])), default=0.)
        clusters = []
        for tangent in selected:
            if not clusters or tangent - clusters[-1][-1] > EDGE_ROI_CONTIGUITY_GAP_M:
                clusters.append([tangent])
            else:
                clusters[-1].append(tangent)
        longest = max(clusters,
                      key=lambda run: (run[-1] - run[0], len(run)), default=[])
        roi[cm] = {'support_points': len(selected),
                   'span_m': max(selected) - min(selected) if len(selected) >= 2 else 0.,
                   'max_gap_m': max_gap,
                   'longest_contiguous_span_m': (longest[-1] - longest[0]
                                                   if len(longest) >= 2 else 0.),
                   'longest_contiguous_support_points': len(longest)}
    angle_to_normal = corrected_pose.yaw - (0. if vertical else math.pi/2)
    angle_to_normal = abs(_wrap_pi(angle_to_normal))
    angle_to_normal = min(angle_to_normal, abs(math.pi - angle_to_normal))
    edge_start = (grid_normal, lo) if vertical else (lo, grid_normal)
    edge_end = (grid_normal, hi) if vertical else (hi, grid_normal)
    return {
        'roi_tangential_interval_m': [lo, hi],
        'roi_halfwidth_m': roi_halfwidth_m,
        'roi': roi,
        'roi_tangent_m': [t for t, _ in endpoints],
        'roi_signed_normal_m': [offset for _, offset in endpoints],
        'normal_mad_m': _mad([offset for _, offset in endpoints]),
        'robot_to_edge_endpoint_m': [
            math.hypot(corrected_pose.x - p[0], corrected_pose.y - p[1])
            for p in (edge_start, edge_end)],
        'heading_to_wall_normal_rad': angle_to_normal,
        'odom_yaw_rate_rad_s': odom_yaw_rate_rad_s,
        'heading_angle_is_incidence': False,
        'heading_to_wall_normal_semantics': (
            'vehicle heading proxy; merged /scan_multi has no source-ray IDs, '
            'so this is not per-beam incidence'),
    }


def _wall_edge_hits(segments, source_pose, corrected_pose, raw_local_points=(),
                    *, roi_halfwidth_m=EDGE_ROI_HALF_WIDTH_M,
                    odom_yaw_rate_rad_s=None):
    """Per-frame, per-cell WALL hit evidence after correction; truth-free."""
    by_edge = {}
    for segment_index, segment in enumerate(segments):
        corrected = _repose_segment(segment, source_pose, corrected_pose)
        for piece in split_by_grid_cells(corrected):
            if piece.inliers < 6 or piece.length < .08:
                continue
            angle = _axis_angle(piece)
            vertical = abs(angle) > math.pi / 4
            off = math.pi / 2 - abs(angle) if vertical else abs(angle)
            if off > math.radians(15):
                continue
            mx = (piece.a[0] + piece.b[0]) / 2
            my = (piece.a[1] + piece.b[1]) / 2
            normal = mx if vertical else my
            # Assign only observed support points that actually fall inside a
            # tangential cell interval. A fitted line crossing an empty cell
            # is not WALL evidence for that cell.
            k = round(normal / CELL_SIZE_M)
            if not (0 <= k <= 7):
                continue
            residual = normal - k * CELL_SIZE_M
            if abs(residual) > .12:
                continue
            support_by_cell = {}
            for point in piece.support:
                tangent = point[1] if vertical else point[0]
                j_point = math.floor(tangent / CELL_SIZE_M)
                if tangent == 7 * CELL_SIZE_M:
                    j_point = 6
                if 0 <= j_point < 7:
                    support_by_cell.setdefault(j_point, []).append(point)
            for j, points in support_by_cell.items():
                tangent_values = [p[1] if vertical else p[0] for p in points]
                span = max(tangent_values) - min(tangent_values)
                if len(points) < 6 or span < .08:
                    continue
                edge = edge_id_for_line('V' if vertical else 'H', k, j)
                # Match ObservationAdapter's edge-normal distance semantics;
                # EdgeMap uses this value for its near-hit vote weight.
                distance = abs(k * CELL_SIZE_M -
                               (corrected_pose.x if vertical else corrected_pose.y))
                if vertical:
                    cell = [0, j] if k == 0 else ([6, j] if k == 7 else [k - 1, j])
                    direction = 'W' if k == 0 else 'E'
                else:
                    cell = [j, 0] if k == 0 else ([j, 6] if k == 7 else [j, k - 1])
                    direction = 'S' if k == 0 else 'N'
                hit = {
                    'edge_id': _edge_id_text(edge),
                    'cell': cell, 'direction': direction,
                    'orientation': 'V' if vertical else 'H', 'line_k': k,
                    'cell_j': j, 'segment_index': segment_index,
                    'support_points': len(points), 'span_m': span,
                    'normal_residual_m': residual, 'distance_m': distance,
                    'same_frame_piece_count': 1,
                    **_edge_local_features(
                        'V' if vertical else 'H', k, j,
                        corrected_pose, raw_local_points,
                        roi_halfwidth_m=roi_halfwidth_m,
                        odom_yaw_rate_rad_s=odom_yaw_rate_rad_s),
                }
                previous = by_edge.get(edge)
                if previous is None:
                    by_edge[edge] = hit
                else:
                    piece_count = previous['same_frame_piece_count'] + 1
                    if hit['support_points'] > previous['support_points']:
                        hit['same_frame_piece_count'] = piece_count
                        by_edge[edge] = hit
                    else:
                        previous['same_frame_piece_count'] = piece_count
    return list(by_edge.values())


def replay(session, log_path=DEFAULT_LOG, summary_path=DEFAULT_SUMMARY,
           truth_path=None, edge_roi_halfwidth_m=EDGE_ROI_HALF_WIDTH_M):
    if log_path.parent != Path('/tmp') or summary_path.parent != Path('/tmp'):
        raise ValueError('outputs must be directly under /tmp')
    db,topics,values,odoms,extrinsic,anchor=_read_bag(session)
    counts=Counter(); rows=[]; truth_rows=[]
    prev_odom=prev_corrected=None; first_odom=None; stamp0=prev_scan_stamp=None
    try:
        query='SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp'
        for _,blob in db.execute(query,(topics[values['scan_topic']][0],)):
            scan=audit.decode_laser_scan(blob)
            try: odom=interpolate_bounded_pose(odoms,scan['stamp'])
            except ValueError: counts['no_odom_bracket']+=1; continue
            if prev_odom is None:
                pred=anchor; first_odom=odom; stamp0=scan['stamp']
            else:
                delta=audit.compose(audit.inverse(_tf(prev_odom)),_tf(odom))
                pred=_pose(audit.compose(_tf(prev_corrected),delta))
            raw_delta=audit.compose(audit.inverse(_tf(first_odom)),_tf(odom))
            raw_pose=_pose(audit.compose(_tf(anchor),raw_delta))
            odom_yaw_rate = None
            if prev_odom is not None and prev_scan_stamp is not None:
                dt = scan['stamp'] - prev_scan_stamp
                if dt > 0:
                    odom_yaw_rate = _wrap_pi(odom.yaw - prev_odom.yaw) / dt
            _,_,local_pts=audit.scan_frame(scan,Pose2D(0.,0.,0.),_tf(Pose2D(0.,0.,0.)),extrinsic)
            c,s=math.cos(pred.yaw),math.sin(pred.yaw)
            segs=extract_segments([(p.x,p.y) for p in local_pts])
            # Fit in base frame, then project under prediction for solver.
            projected=[]
            for seg in segs:
                def tr(p): return (pred.x+c*p[0]-s*p[1],pred.y+s*p[0]+c*p[1])
                projected.append(Segment(tr(seg.a),tr(seg.b),seg.inliers,seg.rms,
                    tuple(tr(p) for p in seg.support)))
            # Reserve a stable deterministic subset before fitting when enough
            # lines exist. This gives a same-frame held-out check; small frames
            # use all lines and are explicitly marked in the log.
            if len(projected)>=7:
                hold_idx={i for i in range(len(projected)) if i%5==0}
                fit=[seg for i,seg in enumerate(projected) if i not in hold_idx]
                held=[seg for i,seg in enumerate(projected) if i in hold_idx]
            else:
                fit=projected; held=[]
            if prev_odom is None:
                result=FrameCorrection(False,'REJECTED','FIRST_FRAME_ANCHOR',0,0,0,pred,
                    len(fit),0,0,None,None,None,tuple(range(len(fit))))
            else:
                result=solve_frame_correction(fit,pred)
            current=result.corrected_pose if result.accepted else pred
            held_pre=_heldout_residual(held,pred)
            held_post=_heldout_residual(held,pred,result.dx,result.dy,result.dyaw)
            held_raw_segments=[_repose_segment(seg,pred,raw_pose) for seg in held]
            held_raw=_heldout_residual(held_raw_segments,raw_pose)
            row={'stamp':scan['stamp'],'elapsed_s':scan['stamp']-stamp0,
              'predicted_pose':[pred.x,pred.y,pred.yaw],
              'raw_odom_pose':[raw_pose.x,raw_pose.y,raw_pose.yaw],
              'corrected_pose':[current.x,current.y,current.yaw],
              'correction':{'accepted':result.accepted,'mode':result.mode,'reason':result.reason,
                'dx':result.dx,'dy':result.dy,'dyaw':result.dyaw,
                'fitted_wall_count':result.fitted_wall_count,
                'associated_wall_count':result.associated_wall_count,
                'inlier_wall_count':result.inlier_wall_count,
                'pre_residual_m':result.pre_residual_m,'post_residual_m':result.post_residual_m,
                'joint_score_margin':result.score_margin,
                'association_line_margin_m':None,'outlier_indices':result.outlier_indices,
                'x_residual_median':result.x_residual_median,'x_residual_mad':result.x_residual_mad,
                'y_residual_median':result.y_residual_median,'y_residual_mad':result.y_residual_mad},
              'heldout_wall_count':len(held),
              'odom_yaw_rate_rad_s':odom_yaw_rate,
              'wall_edge_hits':_wall_edge_hits(
                  projected,pred,current,[(p.x,p.y) for p in local_pts],
                  roi_halfwidth_m=edge_roi_halfwidth_m,
                  odom_yaw_rate_rad_s=odom_yaw_rate),
              'heldout_pre_residual_m':held_pre,
              'heldout_post_residual_m':held_post,
              'heldout_raw_odom_residual_m':held_raw,
              'residual_evaluation':'held-out same-frame lines' if held else 'in-sample; too few fitted lines to hold out',
              'segments':[{'a':list(seg.a),'b':list(seg.b),'inliers':seg.inliers,'rms':seg.rms}
                          for seg in projected]}
            rows.append(row); counts['frames']+=1; counts['segments']+=len(segs)
            truth_rows.append((pred,current,raw_pose,projected))
            counts['accepted']+=int(result.accepted); counts[result.mode]+=1
            prev_odom,prev_corrected=odom,current
            prev_scan_stamp=scan['stamp']
    finally: db.close()
    # Summarize causal motion and grid jumps before any truth access.
    jumps=[]; smooth=[]
    for a,b in zip(rows,rows[1:]):
        pa=a['corrected_pose']; pb=b['corrected_pose']
        dist=math.hypot(pb[0]-pa[0],pb[1]-pa[1])
        if dist>0.30: jumps.append({'from_stamp':a['stamp'],'to_stamp':b['stamp'],'distance_m':dist})
        ca=a['correction']; cb=b['correction']
        if ca['accepted'] and cb['accepted']:
            smooth.append(math.hypot(cb['dx']-ca['dx'],cb['dy']-ca['dy']))
    held_rows=[r for r in rows if r['heldout_wall_count']]
    divergence=[]
    for row in rows:
        c=row['corrected_pose']; b=row['raw_odom_pose']
        divergence.append((math.hypot(c[0]-b[0],c[1]-b[1]),
                           abs(_wrap_pi(c[2]-b[2]))))
    scatter={}
    for pred,corr,raw,segs in truth_rows:
        for seg in segs:
            angle=_axis_angle(seg); vertical=abs(angle)>math.pi/4
            off=math.pi/2-abs(angle) if vertical else abs(angle)
            if off>math.radians(15): continue
            mx=(seg.a[0]+seg.b[0])/2; my=(seg.a[1]+seg.b[1])/2
            normal=mx if vertical else my
            tangent=my if vertical else mx
            k=round(normal/CELL_SIZE_M); j=math.floor(tangent/CELL_SIZE_M)
            if not (0<=k<=7 and 0<=j<7) or abs(normal-k*CELL_SIZE_M)>.12: continue
            corrected_seg=_repose_segment(seg,pred,corr)
            post_normal=((corrected_seg.a[0]+corrected_seg.b[0])/2 if vertical
                         else (corrected_seg.a[1]+corrected_seg.b[1])/2)
            key=('V' if vertical else 'H',k,j)
            values=scatter.setdefault(key,{'pre':[],'post':[]})
            values['pre'].append(normal-k*CELL_SIZE_M)
            values['post'].append(post_normal-k*CELL_SIZE_M)
    summary={'status':'OFFLINE_EXPERIMENT_ONLY','session':str(session),
      'model':'causal previous-corrected pose + odom delta; per-frame shared Manhattan grid snap',
      'counts':dict(counts),'correction_smoothness_median_delta':_median(smooth),
      'heldout_residual_median_pre_m':_median([r['heldout_pre_residual_m'] for r in held_rows if r['heldout_pre_residual_m'] is not None]),
      'heldout_residual_median_post_m':_median([r['heldout_post_residual_m'] for r in held_rows if r['heldout_post_residual_m'] is not None]),
      'heldout_residual_median_anchor_raw_odom_m':_median([r['heldout_raw_odom_residual_m'] for r in held_rows if r['heldout_raw_odom_residual_m'] is not None]),
      'heldout_frames':len(held_rows),
      'corrected_vs_anchor_raw_odom_divergence':{
        'position_m':{'p50':audit.quantile([d[0] for d in divergence],.5),
                      'p90':audit.quantile([d[0] for d in divergence],.9),
                      'max':max((d[0] for d in divergence),default=None),
                      'final':divergence[-1][0] if divergence else None},
        'yaw_rad':{'p50':audit.quantile([d[1] for d in divergence],.5),
                   'p90':audit.quantile([d[1] for d in divergence],.9),
                   'max':max((d[1] for d in divergence),default=None),
                   'final':divergence[-1][1] if divergence else None}},
      'same_local_line_scatter_grid_prior_diagnostic':[
        {'orientation':key[0],'grid_line_k':key[1],'tangential_cell_j':key[2],
         'support_frames':len(vals['pre']),
         'pre_m':{'signed_mad':_mad(vals['pre']),
                  'abs_p50':audit.quantile([abs(v) for v in vals['pre']],.5),
                  'abs_p90':audit.quantile([abs(v) for v in vals['pre']],.9)},
         'post_m':{'signed_mad':_mad(vals['post']),
                   'abs_p50':audit.quantile([abs(v) for v in vals['post']],.5),
                   'abs_p90':audit.quantile([abs(v) for v in vals['post']],.9)}}
        for key,vals in sorted(scatter.items())],
      'grid_jump_candidates_gt_0.30m':jumps,'truth_use':'not loaded during replay'}
    # Optional truth is strictly post-replay.
    if truth_path:
        truth=json.loads(Path(truth_path).read_text())
        expected=audit.edge_truth(truth)
        def labels(pose, source_pose, segs):
            segment_tp=segment_fp=segment_abstain=0
            piece_tp=piece_fp=piece_abstain=0; details=[]
            for segment_index,seg in enumerate(segs):
                world=_repose_segment(seg,source_pose,pose)
                segment_piece_labels=[]; segment_piece_details=[]
                # Retain only actual-hit-supported pieces within each cell.
                for piece in split_by_grid_cells(world):
                    value_label='ABSTAIN'; edge=None; edge_text=None
                    truth_value=None; residual=None; orientation=None
                    line_k=None; cell_j=None
                    a=_axis_angle(piece); vertical=abs(a)>math.pi/4
                    orientation='V' if vertical else 'H'
                    if (math.pi/2-abs(a) if vertical else abs(a))>math.radians(15):
                        piece_abstain+=1
                    else:
                        mx=(piece.a[0]+piece.b[0])/2; my=(piece.a[1]+piece.b[1])/2
                        k=round((mx if vertical else my)/CELL_SIZE_M)
                        j=math.floor((my if vertical else mx)/CELL_SIZE_M)
                        line_k=k; cell_j=j
                        normal=mx if vertical else my
                        if 0<=k<=7 and 0<=j<7 and abs(normal-k*CELL_SIZE_M)<=.12:
                            edge=edge_id_for_line(orientation,k,j)
                            edge_text=_edge_id_text(edge)
                            residual=normal-k*CELL_SIZE_M
                            value=expected.get(edge)
                            truth_value=value
                            if value is True: piece_tp+=1; value_label='TP'
                            elif value is False: piece_fp+=1; value_label='FP'
                            else: piece_abstain+=1
                        else: piece_abstain+=1
                    segment_piece_labels.append(value_label)
                    segment_piece_details.append({
                        'edge_id':edge_text,
                        'orientation':orientation,'line_k':line_k,'cell_j':cell_j,
                        'label':value_label,
                        'truth':('wall' if truth_value is True else
                                 'open' if truth_value is False else None),
                        'truth_wall':truth_value,
                        'support_points':piece.inliers,
                        'span_m':math.hypot(piece.b[0]-piece.a[0],piece.b[1]-piece.a[1]),
                        'line_residual_m':residual})
                # Pair the same original fit segment across poses. Cell splits
                # can differ after correction, so piece-wise zip is invalid.
                if 'FP' in segment_piece_labels:
                    segment_label='FP'
                elif 'TP' in segment_piece_labels:
                    segment_label='TP'
                else:
                    segment_label='ABSTAIN'
                if segment_label=='TP': segment_tp+=1
                elif segment_label=='FP': segment_fp+=1
                else: segment_abstain+=1
                edge_ids=sorted({p['edge_id'] for p in segment_piece_details
                                 if p['edge_id'] is not None})
                fp_edge_ids=sorted({p['edge_id'] for p in segment_piece_details
                                    if p['label']=='FP' and p['edge_id'] is not None})
                details.append({'segment_index':segment_index,
                    'label':segment_label,'edge_ids':edge_ids,
                    'fp_edge_ids':fp_edge_ids,
                    'support_points':seg.inliers,
                    'span_m':math.hypot(seg.b[0]-seg.a[0],seg.b[1]-seg.a[1]),
                    'pieces':segment_piece_details})
            return {'tp':segment_tp,'fp':segment_fp,'abstain':segment_abstain,
                    'scored':segment_tp+segment_fp,
                    'total':segment_tp+segment_fp+segment_abstain,
                    'conditional_precision':segment_tp/(segment_tp+segment_fp) if segment_tp+segment_fp else None,
                    'scored_fraction':(segment_tp+segment_fp)/(segment_tp+segment_fp+segment_abstain) if segment_tp+segment_fp+segment_abstain else None,
                    'piece_score':{'tp':piece_tp,'fp':piece_fp,'abstain':piece_abstain,
                      'scored':piece_tp+piece_fp,'total':piece_tp+piece_fp+piece_abstain,
                      'conditional_precision':piece_tp/(piece_tp+piece_fp) if piece_tp+piece_fp else None,
                      'scored_fraction':(piece_tp+piece_fp)/(piece_tp+piece_fp+piece_abstain) if piece_tp+piece_fp+piece_abstain else None},
                    'details':details}
        per_frame=[]
        for frame_index,(row,(pred,corr,raw,segs)) in enumerate(zip(rows,truth_rows)):
            per_frame.append({'frame_index':frame_index,'stamp':row['stamp'],
                'elapsed_s':row['elapsed_s'],
                'corrected_pose':row['corrected_pose'],
                'anchor_plus_raw_odom_pose':row['raw_odom_pose'],
                'corrected':labels(corr,pred,segs),
                'anchor_plus_raw_odom':labels(raw,pred,segs)})
        def aggregate(items,name,metric=None):
            def get(p): return p[name]['piece_score'] if metric=='pieces' else p[name]
            tp=sum(get(p)['tp'] for p in items); fp=sum(get(p)['fp'] for p in items)
            abstain=sum(get(p)['abstain'] for p in items); total=tp+fp+abstain
            return {'tp':tp,'fp':fp,'abstain':abstain,'scored':tp+fp,'total':total,
              'conditional_precision':tp/(tp+fp) if tp+fp else None,
              'scored_fraction':(tp+fp)/total if total else None}
        paired=Counter()
        for frame in per_frame:
            for c,r in zip(frame['corrected']['details'],frame['anchor_plus_raw_odom']['details']):
                if c['label']!='ABSTAIN' and r['label']!='ABSTAIN':
                    paired[f"corrected_{c['label']}_raw_{r['label']}"]+=1
        bounds=[(0,30),(30,60),(60,120),(120,float('inf'))]
        windows=[]
        for lo,hi in bounds:
            selected=[p for p in per_frame if lo<=p['elapsed_s']<hi]
            item={'elapsed_s':[lo,None if math.isinf(hi) else hi]}
            for name in ('corrected','anchor_plus_raw_odom'):
                item[name]=aggregate(selected,name,'pieces')
            windows.append(item)
        summary['posthoc_truth']={'geometry_sample_score':{
            name:aggregate(per_frame,name,'pieces')
            for name in ('corrected','anchor_plus_raw_odom')},
            'same_fitted_segment_score':{
              name:aggregate(per_frame,name)
              for name in ('corrected','anchor_plus_raw_odom')},
            'paired_same_fitted_segment_outcomes':dict(paired),
            'fp_by_edge_id':_fp_edge_diagnostics(per_frame),
            'time_windows':windows,
            'physical_image_transform_resolved':truth.get('grid',{}).get('coordinate_views',{}).get('physical_image_transform_resolved'),
            'axis_assumption':'scored against the user-provided anchor, grid axes, and wall table',
            'truth_use':'post-run only; excluded from association and correction'}
    log_path.write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in rows))
    summary_path.write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,default=DEFAULT_SESSION)
    p.add_argument('--log',type=Path,default=DEFAULT_LOG)
    p.add_argument('--summary',type=Path,default=DEFAULT_SUMMARY)
    p.add_argument('--truth-score',type=Path,default=None)
    p.add_argument('--edge-roi-halfwidth-m',type=float,
                   default=EDGE_ROI_HALF_WIDTH_M,
                   help='maximum normal half-width retained for per-edge endpoint features')
    a=p.parse_args()
    print(json.dumps(replay(a.session,a.log,a.summary,a.truth_score,
                            a.edge_roi_halfwidth_m),indent=2,ensure_ascii=False))

if __name__=='__main__': main()
