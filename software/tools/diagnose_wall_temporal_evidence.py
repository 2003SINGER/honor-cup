#!/usr/bin/env python3
"""Posthoc temporal and per-edge diagnostics for the dynamic wall replay.

This is an evaluation tool only. It does not change pose correction or EdgeMap.
Input may be plain JSONL or zstd-compressed JSONL (the zstd CLI is used).
"""
from __future__ import annotations

import argparse
import ast
import csv
import json
import math
import statistics
import subprocess
from collections import defaultdict
from pathlib import Path

CELL = .4
VIEW_POSITION = .20
VIEW_YAW = math.radians(20)
CONTIG_GAP = .06


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def read_lines(path):
    p = Path(path)
    if p.suffix == '.zst':
        proc = subprocess.Popen(['zstd', '-q', '-d', '-c', str(p)], stdout=subprocess.PIPE, text=True)
        try:
            for line in proc.stdout:
                yield line
        finally:
            proc.stdout.close()
            if proc.wait() != 0:
                raise RuntimeError(f'zstd failed reading {p}')
    else:
        yield from p.open()


def edge_axis(edge):
    if edge.startswith('E('):
        a, b = map(int, edge[2:-1].split(',')); return 'E', a, b
    if edge.startswith('N('):
        a, b = map(int, edge[2:-1].split(',')); return 'N', a, b
    return None


def canonical_edge_id(value):
    edge=ast.literal_eval(value) if isinstance(value,str) else value
    if edge[0]=='B': return f'B({edge[1][0]},{edge[1][1]},{edge[2]})'
    cell,direction=edge
    return f'{direction}({cell[0]},{cell[1]})'


def roi_stats(hit):
    roi = hit.get('roi', {}).get('5', {})
    points = roi.get('support_points')
    span = roi.get('span_m')
    cspan = roi.get('longest_contiguous_span_m')
    cpoints = roi.get('longest_contiguous_support_points')
    if points is None:
        vals = sorted(abs(float(n)) for n in hit.get('roi_signed_normal_m', []))
        tang = hit.get('roi_tangent_m', [])
        selected = sorted(float(t) for t, n in zip(tang, hit.get('roi_signed_normal_m', [])) if abs(float(n)) <= .05)
        points = len(selected); span = selected[-1]-selected[0] if len(selected)>1 else 0.
        runs=[]
        for t in selected:
            if not runs or t-runs[-1][-1] > CONTIG_GAP: runs.append([t])
            else: runs[-1].append(t)
        best=max(runs,key=lambda r:(r[-1]-r[0],len(r)),default=[])
        cspan=best[-1]-best[0] if len(best)>1 else 0.; cpoints=len(best)
    return points, span, cpoints, cspan


def view_count(samples):
    reps=[]
    for s in samples:
        pose=s['pose']
        if not any(math.dist(pose[:2],r[:2]) <= VIEW_POSITION and abs(wrap(pose[2]-r[2])) <= VIEW_YAW for r in reps):
            reps.append(pose)
    return len(reps)


def analyze(log_path, sweep_csv, edge_csv, output_csv, output_json):
    frames=[]
    for raw in read_lines(log_path):
        if raw.strip(): frames.append(json.loads(raw))
    # Select the report's 5 cm / 6 point / 18 cm continuous-cluster candidate.
    with Path(sweep_csv).open(newline='') as f:
        candidates=[]
        for row in csv.DictReader(f):
            if (row.get('sweep_kind') == 'contiguous_roi' and row.get('contiguous_roi_half_width_cm') == '5'
                and row.get('min_contiguous_span_m') == '0.18'
                and row.get('min_contiguous_support_points') == '6'
                and row.get('independent_view_confirmations') == '2'):
                candidates.append(row)
    if not candidates:
        raise ValueError('could not find the 5 cm / 6 point / 18 cm candidate in sweep CSV')
    candidate=max(candidates,key=lambda r:int(r['true_wall_confirmed'])-int(r['open_final_effective_wall']))
    candidate_false_edges=set(json.loads(candidate['false_final_edges']))
    baseline_false=set()
    with Path(edge_csv).open(newline='') as f:
        for row in csv.DictReader(f):
            if row.get('truth') == 'OPEN' and row.get('effective_false_wall', '').lower() == 'true':
                baseline_false.add(row['edge_id'].strip('"'))
    diagnostic_edges=baseline_false or candidate_false_edges
    samples=defaultdict(list)
    temporal={'eligible_adjacent_pairs':0,'same_edge_continuations':0,'same_edge_gaps':0,
              'adjacent_id_switches':0,'adjacent_switches_same_physical_normal':0,
              'adjacent_switch_examples':[]}
    previous={}
    for fi,frame in enumerate(frames):
        curr={}
        for hit in frame.get('wall_edge_hits',[]):
            edge=canonical_edge_id(hit['edge_id']); pose=frame.get('corrected_pose',frame.get('pose'))
            if isinstance(pose,dict): pose=[pose['x'],pose['y'],pose['theta']]
            p,sp,cp,csp=roi_stats(hit)
            item={'frame_index':fi,'elapsed_s':frame.get('elapsed_s'),'pose':pose,
                  'range_m':hit.get('distance_m'),'points_5cm':p,'span_5cm_m':sp,
                  'cluster_points_5cm':cp,'cluster_span_5cm_m':csp,
                  'normal_residual_m':hit.get('normal_residual_m'),
                  'normal_mad_m':hit.get('normal_mad_m'),'yaw_rate_rad_s':hit.get('odom_yaw_rate_rad_s'),
                  'orientation':hit.get('orientation'),'line_k':hit.get('line_k'),'cell_j':hit.get('cell_j'),
                  'tangent_interval_m':hit.get('roi_tangential_interval_m'),
                  'tangents':hit.get('roi_tangent_m',[]),'normals':hit.get('roi_signed_normal_m',[])}
            curr[edge]=item; samples[edge].append(item)
        # A temporal neighbor is the immediately following usable scan record.
        for edge, old in previous.items():
            if edge in curr:
                temporal['eligible_adjacent_pairs']+=1
                temporal['same_edge_continuations']+=1
        for edge,item in curr.items():
            ax=edge_axis(edge)
            if ax is None: continue
            ori,k,j=ax
            for old_edge,old in previous.items():
                oa=edge_axis(old_edge)
                # Count only an actual one-frame ID handoff: old ID disappears,
                # neighboring tangent cell appears, while both share a normal
                # grid line. Co-visible neighbors are not an ID jump.
                if (old_edge in curr or edge in previous or not oa or oa[0]!=ori
                    or oa[1]!=k or abs(oa[2]-j)!=1): continue
                temporal['adjacent_id_switches']+=1
                n0=k*CELL + (old.get('normal_residual_m') or 0.)
                n1=k*CELL + (item.get('normal_residual_m') or 0.)
                same=abs(n0-n1)<=.06
                if same: temporal['adjacent_switches_same_physical_normal']+=1
                if len(temporal['adjacent_switch_examples'])<30:
                    temporal['adjacent_switch_examples'].append({'from_edge':old_edge,'to_edge':edge,
                        'frame_from':old['frame_index'],'frame_to':item['frame_index'],
                        'same_physical_normal_within_6cm':same,
                        'normal_residual_from_m':old.get('normal_residual_m'),
                        'normal_residual_to_m':item.get('normal_residual_m')})
        previous=curr
    rows=[]
    for edge in sorted(diagnostic_edges):
        ss=samples.get(edge,[])
        runs=[]; run=[]
        for s in ss:
            if run and s['frame_index'] != run[-1]['frame_index']+1:
                runs.append(run); run=[]
            run.append(s)
        if run:runs.append(run)
        spans=[s['span_5cm_m'] for s in ss if s['span_5cm_m'] is not None]
        counts=[s['points_5cm'] for s in ss if s['points_5cm'] is not None]
        mads=[s['normal_mad_m'] for s in ss if s['normal_mad_m'] is not None]
        res=[s['normal_residual_m'] for s in ss if s['normal_residual_m'] is not None]
        views=view_count(ss)
        for s in ss:
            vertical=s.get('orientation')=='V'
            px,py,theta=s['pose']
            if vertical:
                x=(s.get('line_k') or 0)*CELL + (s.get('normal_residual_m') or 0.)-px
                y=((s.get('cell_j') or 0)+.5)*CELL-py
            else:
                x=((s.get('cell_j') or 0)+.5)*CELL-px
                y=(s.get('line_k') or 0)*CELL + (s.get('normal_residual_m') or 0.)-py
            s['robot_local_relative_xy_m']=[math.cos(theta)*x+math.sin(theta)*y,-math.sin(theta)*x+math.cos(theta)*y]
            s['side_mask_class']=side_mask(s['robot_local_relative_xy_m'])
        side=sum(s['side_mask_class']=='verified_side' for s in ss)
        relxs=[s['robot_local_relative_xy_m'][0] for s in ss]
        rely=[s['robot_local_relative_xy_m'][1] for s in ss]
        maxrun=max((len(r) for r in runs),default=0)
        rows.append({'edge_id':edge,'truth':'OPEN','frames':len(ss),'elapsed_start_s':ss[0]['elapsed_s'] if ss else None,
          'elapsed_end_s':ss[-1]['elapsed_s'] if ss else None,'max_consecutive_frames':maxrun,
          'independent_viewpoints':views,'range_median':statistics.median([s['range_m'] for s in ss if s['range_m'] is not None]) if ss else None,
          'roi_points_5cm_median':statistics.median(counts) if counts else None,'roi_span_5cm_median_m':statistics.median(spans) if spans else None,
          'cluster_span_5cm_median_m':statistics.median([s['cluster_span_5cm_m'] for s in ss]) if ss else None,
          'normal_mad_median_m':statistics.median(mads) if mads else None,'normal_residual_median_m':statistics.median(res) if res else None,
          'robot_local_forward_median_m':statistics.median(relxs) if relxs else None,
          'robot_local_lateral_median_m':statistics.median(rely) if rely else None,
          'max_abs_yaw_rate_rad_s':max((abs(s['yaw_rate_rad_s']) for s in ss if s['yaw_rate_rad_s'] is not None),default=None),
          'verified_side_mask_frames':side,'verified_side_mask_any':bool(side),'frame_details':ss})
    with Path(output_csv).open('w',newline='') as f:
        fields=[k for k in rows[0] if k!='frame_details'] if rows else []
        writer = csv.DictWriter(f, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        writer.writerows(
            [{key: value for key, value in row.items()
              if key != 'frame_details'} for row in rows]
        )
    report={'source_log':str(log_path),'baseline_edge_csv':str(edge_csv),'frame_count':len(frames),'candidate_config':candidate,
      'candidate_false_edges':sorted(candidate_false_edges),
      'baseline_false_edges_used_for_attribution':sorted(diagnostic_edges),
      'baseline_false_edge_count':len(diagnostic_edges),
      'temporal_diagnostics':temporal,'false_edge_details':rows,
      'side_mask_definition':'verified side band: robot-local |lateral|<=1.0 m and forward coordinate from -0.5 to 1.0 m; diagnostic approximation only; does not encode the requested per-cell lateral-wall mask.',
      'interpretation_limits':['No external pose truth: robot-local positions inherit corrected-pose error.',
       'Consecutive replay records measure persistence, not proven physical identity.',
       'Independent viewpoints are re-counted using the existing 0.20 m OR 20 degree representative rule, consistent with sweep behavior only approximately.',
       'Adjacent canonical-edge switches are a diagnostic candidate for ID jumps; this tool does not alter or suppress WALL votes.']}
    Path(output_json).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'frames':len(frames),'candidate_false_edges':len(candidate_false_edges),
      'baseline_false_edges_attributed':len(diagnostic_edges),'temporal':{k:v for k,v in temporal.items() if not k.endswith('examples')},
      'edges':[ {k:v for k,v in r.items() if k!='frame_details'} for r in rows]},ensure_ascii=False,indent=2))


def side_mask(xy):
    forward,lateral=xy
    if abs(lateral)<.12:return 'forward'
    if 0 <= forward <= 1.0 and abs(lateral)<=1.0:return 'verified_side'
    if 1.0 < forward <= 1.2 and abs(lateral)<=1.0:return 'side_transition'
    return 'outside'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log',type=Path,required=True)
    p.add_argument('--sweep-csv',type=Path,required=True)
    p.add_argument('--edge-level-csv',type=Path,required=True)
    p.add_argument('--csv',type=Path,default=Path('/tmp/wall-temporal-diagnostics.csv'))
    p.add_argument('--json',type=Path,default=Path('/tmp/wall-temporal-diagnostics.json'))
    a=p.parse_args(); analyze(a.log,a.sweep_csv,a.edge_level_csv,a.csv,a.json)

if __name__=='__main__': main()
