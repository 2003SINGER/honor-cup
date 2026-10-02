#!/usr/bin/env python3
"""Offline sequential replay of joystick bag with full-maze truth correction.

Only even-indexed stationary scans may propose a bounded update. The following
odd scan is the temporal holdout. Nothing is sent to ROS or written to the bag.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sqlite3
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2' / 'm3pro_nav'))
from analyze_scan_edge import (CELL_SIZE_M, HEADING_RAD, Pose2D, Transform2D,
    compose, decode_laser_scan, decode_odometry, decode_tf_message,
    interpolate_pose, inverse, static_transform)
from audit_lidar_odom_calibration import edge_truth, scan_frame
from m3pro_nav.frame_projector import WorldRay
from m3pro_nav.grid_association import GridAssociation, UNIQUE
from m3pro_nav.pose_correction import (KnownWallSegment, PoseCorrectionConfig,
    ProjectedEndpoint, propose_pose_correction)
from m3pro_nav.pose_correction import (_signed_distance_and_tangent,
    _within_segment_interior)


def truth_walls(truth):
    opened = truth['open_directions_by_cell']; n = truth['grid']['width']; c = truth['grid']['cell_size_m']
    walls = {}
    for y, row in enumerate(opened):
        for x, dirs in enumerate(row):
            for d, key, a, b in (
                ('N', ('H', y+1, x), (x*c, (y+1)*c), ((x+1)*c, (y+1)*c)),
                ('S', ('H', y, x), (x*c, y*c), ((x+1)*c, y*c)),
                ('E', ('V', x+1, y), ((x+1)*c, y*c), ((x+1)*c, (y+1)*c)),
                ('W', ('V', x, y), (x*c, y*c), (x*c, (y+1)*c))):
                wall = d not in dirs
                if key in walls and walls[key][0] != wall: raise ValueError(f'inconsistent truth edge {key}')
                walls[key] = (wall, a, b)
    return tuple(KnownWallSegment(a, b, True, str(k)) for k,(w,a,b) in walls.items() if w), edge_truth(truth)


def load_bag(session, truth):
    meta = {}
    for line in (session/'session.yaml').read_text().splitlines():
        if ':' in line:
            k,v=line.split(':',1); meta[k.strip()]=v.strip().strip('"\'')
    dbfile=next((session/'bag').rglob('*.db3')); db=sqlite3.connect(f'file:{dbfile}?mode=ro',uri=True)
    topics={name:tid for tid,name,_ in db.execute('select id,name,type from topics')}
    odoms=[]
    for _,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topics[meta['odom_topic']],)):
        stamp,frame,child,p=decode_odometry(blob); odoms.append((stamp,p))
    odoms.sort(key=lambda x:x[0]); tf=[]
    for _,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topics['/tf_static'],)): tf.extend(decode_tf_message(blob))
    base_laser=static_transform(tf,meta['base_frame'],meta['laser_frame'])
    cell=tuple(map(int,meta['cell'].strip('[]').split(','))); heading=meta['heading']
    first=odoms[0][1]; anchor=compose(Transform2D((cell[0]+.5)*.4,(cell[1]+.5)*.4,HEADING_RAD[heading]),inverse(Transform2D(first.x,first.y,first.yaw)))
    scans=[]
    for bag_ns,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topics[meta['scan_topic']],)):
        scan=decode_laser_scan(blob)
        try: pose=interpolate_pose(odoms,scan['stamp'])
        except ValueError: continue
        scans.append((bag_ns*1e-9,scan,pose))
    db.close(); return scans,anchor,base_laser


def score(rays, truth_edges):
    assoc=GridAssociation(); wc=wf=oc=of=0
    for o in assoc.process(rays):
        if o.outcome==UNIQUE and o.candidate:
            wall=truth_edges.get(o.candidate.edge_id)
            if wall is True: wc+=1
            elif wall is False: wf+=1
        for e in o.open_edges:
            wall=truth_edges.get(e)
            if wall is False: oc+=1
            elif wall is True: of+=1
    return [wc,wf,oc,of]


def local_walls(walls, pose, radius):
    """Deterministic predicted-neighborhood wall set, without scan selection."""
    selected = []
    for wall in walls:
        ax, ay = wall.start
        vx = wall.end[0] - ax
        vy = wall.end[1] - ay
        length_sq = vx * vx + vy * vy
        t = max(0.0, min(1.0,
            ((pose.x - ax) * vx + (pose.y - ay) * vy) / length_sq))
        if math.hypot(pose.x - (ax + t * vx),
                      pose.y - (ay + t * vy)) <= radius:
            selected.append(wall)
    return tuple(selected)


def pruned_proposal(endpoints, walls, pose, config, max_walls=8):
    """Experimental training-only wall-subset search; production code untouched.

    Candidate walls are chosen from unambiguous gated endpoint associations.
    Every retained subset still passes the unchanged production estimator and
    its gates. Select the smallest accepted correction among well-supported
    candidates to avoid choosing a large fit merely because it wins in-sample.
    """
    counts=Counter()
    for p in endpoints:
        choices=[]
        for wall in walls:
            residual,along,length,_,_=_signed_distance_and_tangent((p.x,p.y),wall)
            if (_within_segment_interior(along,length,config) and
                    abs(residual)<=config.association_gate_m):
                choices.append((abs(residual),wall))
        choices.sort(key=lambda item:item[0])
        if choices and (len(choices)==1 or choices[1][0]-choices[0][0]>=.005):
            counts[choices[0][1]]+=1
    active=[w for w,n in counts.most_common() if n>=config.min_hits_per_wall][:max_walls]
    candidates=[]
    for size in range(2,min(5,len(active))+1):
        for subset in itertools.combinations(active,size):
            result=propose_pose_correction(endpoints,subset,pose,config)
            if result.accepted and result.n_walls>=2 and result.n_inliers>=30:
                magnitude=math.hypot(result.dx,result.dy)+.4*abs(result.dyaw)
                candidates.append((magnitude,-result.n_walls,-result.n_inliers,
                                   result.p90_residual_m,result,subset))
    if not candidates:
        return None, len(active), 0
    chosen=min(candidates,key=lambda item:item[:4])
    return (chosen[4],chosen[5]),len(active),len(candidates)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session',type=Path,nargs='?',default=Path('field_data/20261001_174719_joystick_full_maze'))
    ap.add_argument('--truth',type=Path,default=Path('field/maze_truth_7x7.json'))
    ap.add_argument('--window',type=int,default=100,help='frames per report segment')
    ap.add_argument('--pruned-ab',action='store_true',help='run experimental bounded wall-subset A/B replay')
    ap.add_argument('--local-radius', type=float,
                    help='only fit truth walls this far from the predicted pose')
    ap.add_argument('--output',type=Path)
    args=ap.parse_args(); truth=json.loads(args.truth.read_text()); walls,edges=truth_walls(truth)
    scans,anchor,base_laser=load_bag(args.session,truth)
    cfg=PoseCorrectionConfig(); correction=Transform2D(0,0,0); association=GridAssociation()
    reasons=Counter(); accepted=0; stationary_frames=0; baseline=defaultdict(lambda:[0,0,0,0]); replay=defaultdict(lambda:[0,0,0,0]); hold=defaultdict(lambda:[0,0,0,0]); nframes=0
    prev=None; updates=[]; pruned_updates=[]; pruned_reasons=Counter(); pruned_active=Counter(); pruned_candidates=0
    pruned_correction=Transform2D(0,0,0); ab_baseline=defaultdict(lambda:[0,0,0,0]); ab_corrected=defaultdict(lambda:[0,0,0,0]); holdout_change=Counter(); holdout_change_by_segment=defaultdict(Counter)
    for i,(arrival,scan,odom) in enumerate(scans):
        # stationarity from adjacent scan-pose velocity; require this and the prior scan
        dt=(scan['stamp']-prev[0]) if prev else 99
        speed=(math.hypot(odom.x-prev[1].x,odom.y-prev[1].y)/dt if prev and dt>0 else math.inf)
        yawrate=(abs(math.atan2(math.sin(odom.yaw-prev[1].yaw),math.cos(odom.yaw-prev[1].yaw)))/dt if prev and dt>0 else math.inf)
        stationary=(speed<.025 and yawrate<math.radians(5))
        nominal,_,_=scan_frame(scan,odom,anchor,base_laser)
        maze_from_odom=compose(correction,anchor)
        corrected, rays, endpoints=scan_frame(scan,odom,maze_from_odom,base_laser)
        seg=i//args.window
        b=score(scan_frame(scan,odom,anchor,base_laser)[1],edges)
        baseline[seg]=[x+y for x,y in zip(baseline[seg],b)]
        s=score(rays,edges); replay[seg]=[x+y for x,y in zip(replay[seg],s)]
        if i%2==1:
            hold[seg]=[x+y for x,y in zip(hold[seg],s)]
            if args.pruned_ab:
                ab_baseline[seg]=[x+y for x,y in zip(ab_baseline[seg],b)]
                # correction enters on previous even frame; replay this odd holdout
                ab_pose,ab_rays,_=scan_frame(scan,odom,compose(pruned_correction,anchor),base_laser)
                ab_s=score(ab_rays,edges); ab_corrected[seg]=[x+y for x,y in zip(ab_corrected[seg],ab_s)]
                for label,lo,hi in (('unique',0,2),('open',2,4)):
                    old=b[lo]/sum(b[lo:hi]) if sum(b[lo:hi]) else None
                    new=ab_s[lo]/sum(ab_s[lo:hi]) if sum(ab_s[lo:hi]) else None
                    if old is not None and new is not None:
                        key=f'{label}_improved' if new>old+1e-12 else f'{label}_worsened' if new<old-1e-12 else f'{label}_unchanged'
                        holdout_change[key]+=1; holdout_change_by_segment[seg][key]+=1
        # Fit only an even, stationary scan; apply to future frames only.
        if stationary:
            stationary_frames+=1
            if i%2==0:
                fit_walls = (local_walls(walls, corrected, args.local_radius)
                             if args.local_radius is not None else walls)
                result=propose_pose_correction(endpoints,fit_walls,corrected,cfg); reasons[result.reason]+=1
                if result.accepted:
                    accepted+=1
                    delta=Transform2D(result.dx,result.dy,result.dyaw)
                    correction=compose(delta,correction)
                    updates.append({'frame':i,'dx':result.dx,'dy':result.dy,'dyaw':result.dyaw,'walls':result.n_walls,'inliers':result.n_inliers,'fit_p90_m':result.p90_residual_m})
                if args.pruned_ab:
                    candidate,nactive,ncandidates=pruned_proposal(endpoints,walls,corrected,cfg)
                    pruned_active[str(nactive)]+=1; pruned_candidates+=ncandidates
                    if candidate is None:
                        pruned_reasons[result.reason]+=1
                    else:
                        zr,subset=candidate
                        # Candidate choice uses only the fit frame; subsequent odd frame
                        # provides temporal holdout scoring.
                        pruned_correction=compose(Transform2D(zr.dx,zr.dy,zr.dyaw),pruned_correction)
                        pruned_updates.append({'frame':i,'dx':zr.dx,'dy':zr.dy,'dyaw':zr.dyaw,'walls':zr.n_walls,'inliers':zr.n_inliers,'fit_p90_m':zr.p90_residual_m,'subset':[w.wall_id for w in subset]})
        prev=(scan['stamp'],odom)
        nframes+=1
    def summary(v):
        w,fw,o,fo=v; return {'unique_wall_precision':w/(w+fw) if w+fw else None,'open_precision':o/(o+fo) if o+fo else None,'wall_votes':w+fw,'open_votes':o+fo,'counts':[w,fw,o,fo]}
    proposal_count=sum(reasons.values())
    segments=[]
    for k in sorted(baseline):
        row={'frame_range':[k*args.window,min((k+1)*args.window,nframes)-1],'baseline_all':summary(baseline[k]),'corrected_all':summary(replay[k]),'corrected_odd_holdout':summary(hold[k])}
        if args.pruned_ab: row.update({'pruned_ab_baseline_odd':summary(ab_baseline[k]),'pruned_ab_corrected_odd':summary(ab_corrected[k]),'pruned_ab_holdout_change_counts':dict(holdout_change_by_segment[k])})
        segments.append(row)
    report={'session':args.session.name,'scans':nframes,'stationary_scans':stationary_frames,'fit_policy':'even-index stationary frames fit and update map<-odom; odd frames are temporal holdout, no same-frame update used for scored output','motion_thresholds':{'linear_m_s':.025,'yaw_deg_s':5},'truth_walls':len(walls),'local_radius_m':args.local_radius,'proposal_count':proposal_count,'updates':len(updates),'acceptance_rate':len(updates)/proposal_count if proposal_count else None,'rejections':dict(reasons),'update_samples':updates,'segments':segments,'cumulative_map_from_odom_correction':[correction.x,correction.y,correction.yaw],'pruned_ab':({'candidate_updates':len(pruned_updates),'candidate_acceptance_rate':len(pruned_updates)/proposal_count if proposal_count else None,'subset_candidates_evaluated':pruned_candidates,'active_wall_count_histogram':dict(pruned_active),'no_candidate_rejections':dict(pruned_reasons),'holdout_frame_precision_change_counts':dict(holdout_change),'update_samples':pruned_updates,'cumulative_map_from_odom_correction':[pruned_correction.x,pruned_correction.y,pruned_correction.yaw]} if args.pruned_ab else None),'caveats':['session anchor alignment is assumed','stationary windows are detected from odometry, which may itself drift','adjacent odd-frame holdout is temporally correlated; this tests immediate prediction, not independent validation','maze_truth_7x7.json leaves physical image transform unresolved; this replay assumes the session anchor maps directly into its ideal 0.4 m axes','all truth walls include perimeter, assuming ideal axis-aligned geometry','experimental subset selection uses only the fit frame but remains susceptible to in-sample model selection overfit']}
    out=json.dumps(report,indent=2,ensure_ascii=False)+'\n'
    if args.output: args.output.write_text(out)
    print(out,end='')
if __name__=='__main__': main()
