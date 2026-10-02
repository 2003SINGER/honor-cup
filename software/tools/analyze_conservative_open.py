#!/usr/bin/env python3
"""Conditional, conservative OPEN evidence probe for a fused /scan_multi bag.

This is an offline diagnostic only. It treats each decoded finite return as an
endpoint hypothesis, never treats base_link->hit as a real beam, and requires
every plausible lidar-origin hypothesis to cross an interior grid edge before
the hit. Timing is only bounded by an explicit 150 ms motion envelope; this is
an empirical assumption, not a guarantee provided by the fused message.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))

import audit_lidar_odom_calibration as audit
from analyze_scan_edge import (CELL_SIZE_M, HEADING_RAD, Transform2D,
    compose, decode_laser_scan, decode_odometry, decode_tf_message,
    inverse, static_transform)
from m3pro_nav.edge_map import EdgeMap
from m3pro_nav.grid_association import edge_id_for_line
from m3pro_nav.pose import Pose2D

N = 7
DT_BOUND = .15
MAX_SPEED = .03
MAX_YAW_RATE = .05
HIT_MARGIN = .08
CORNER_MARGIN = .06

ORIGINS = ()  # Filled from this bag's /tf_static at runtime.


def crossing_edge(origin, hit, uncertainty):
    """Return every edge robustly crossed before the hit."""
    ox, oy = origin
    hx, hy = hit
    dx, dy = hx - ox, hy - oy
    length = math.hypot(dx, dy)
    if length <= HIT_MARGIN + uncertainty:
        return frozenset()
    found = []
    for k in range(N + 1):
        x = k * CELL_SIZE_M
        if abs(dx) < 1e-12:
            continue
        t = (x - ox) / dx
        if not (0 < t < 1):
            continue
        along = oy + t * dy
        margin = CORNER_MARGIN + uncertainty
        if margin < along < N * CELL_SIZE_M - margin:
            j = math.floor(along / CELL_SIZE_M)
            local = along - j * CELL_SIZE_M
            if CORNER_MARGIN + uncertainty < local < CELL_SIZE_M - CORNER_MARGIN - uncertainty:
                before_hit = (1 - t) * length
                if before_hit > HIT_MARGIN + uncertainty:
                    found.append(('V', k, j))
    for k in range(N + 1):
        y = k * CELL_SIZE_M
        if abs(dy) < 1e-12:
            continue
        t = (y - oy) / dy
        if not (0 < t < 1):
            continue
        along = ox + t * dx
        margin = CORNER_MARGIN + uncertainty
        if margin < along < N * CELL_SIZE_M - margin:
            j = math.floor(along / CELL_SIZE_M)
            local = along - j * CELL_SIZE_M
            if CORNER_MARGIN + uncertainty < local < CELL_SIZE_M - CORNER_MARGIN - uncertainty:
                before_hit = (1 - t) * length
                if before_hit > HIT_MARGIN + uncertainty:
                    found.append(('H', k, j))
    # A near-corner crossing is already removed by the along-edge guards.
    return frozenset(found)


def edge_id(key):
    orientation, line, seg = key
    return EdgeMap(7)._fk(edge_id_for_line(orientation, line, seg))


def hit_edge_candidates(hit, residual=.05):
    """Possible WALL edges from same-frame endpoints close to grid lines."""
    x, y = hit
    found = set()
    kx = round(x / CELL_SIZE_M)
    if 0 <= kx <= N and abs(x-kx*CELL_SIZE_M) <= residual:
        j = math.floor(y / CELL_SIZE_M)
        local = y-j*CELL_SIZE_M
        if 0 <= j < N and CORNER_MARGIN < local < CELL_SIZE_M-CORNER_MARGIN:
            found.add(('V',kx,j))
    ky = round(y / CELL_SIZE_M)
    if 0 <= ky <= N and abs(y-ky*CELL_SIZE_M) <= residual:
        j = math.floor(x / CELL_SIZE_M)
        local = x-j*CELL_SIZE_M
        if 0 <= j < N and CORNER_MARGIN < local < CELL_SIZE_M-CORNER_MARGIN:
            found.add(('H',ky,j))
    return found


def laser_origins_from_tf(transforms, base_frame='base_link'):
    found={}
    for parent, child, tf in transforms:
        if parent == base_frame and child in ('laser0_frame','laser1_frame'):
            found[child]=(tf.x,tf.y)
    if set(found) != {'laser0_frame','laser1_frame'}:
        raise ValueError(f'/tf_static lacks both laser origins: {found}')
    return tuple(found[k] for k in ('laser0_frame','laser1_frame'))


def _pose_at(samples, stamp):
    times = [row[0] for row in samples]
    i = bisect.bisect_left(times, stamp)
    if i == 0 or i == len(samples):
        raise ValueError('no odometry bracket')
    ta, a = samples[i - 1]
    tb, b = samples[i]
    if tb - ta > .15:
        raise ValueError('odometry gap exceeds 150 ms')
    f = (stamp - ta) / (tb - ta)
    dy = math.atan2(math.sin(b.yaw-a.yaw), math.cos(b.yaw-a.yaw))
    return Pose2D(a.x + f*(b.x-a.x), a.y + f*(b.y-a.y), a.yaw + f*dy)


def _inputs(session):
    values = {}
    for line in (session / 'session.yaml').read_text().splitlines():
        if ':' in line:
            k, v = line.split(':', 1)
            values[k.strip()] = v.strip().strip('"\'')
    cell = tuple(int(x) for x in values['cell'].strip('[]').split(','))
    dbpath, = sorted((session / 'bag').rglob('*.db3'))
    db = sqlite3.connect(f'file:{dbpath}?mode=ro', uri=True)
    topics = {name:(tid,typ) for tid,name,typ in db.execute('select id,name,type from topics')}
    required = (values['scan_topic'], values['odom_topic'], '/tf_static')
    missing = set(required) - topics.keys()
    if missing:
        raise ValueError(f'missing bag topics: {sorted(missing)}')
    odoms=[]
    for (blob,) in db.execute('select data from messages where topic_id=? order by timestamp', (topics[values['odom_topic']][0],)):
        stamp, frame, child, pose = decode_odometry(blob)
        if frame != values['odom_frame'] or child != values['base_frame']:
            raise ValueError(f'odom frames {frame}/{child} unexpected')
        odoms.append((stamp,pose))
    transforms=[]
    for (blob,) in db.execute('select data from messages where topic_id=? order by timestamp', (topics['/tf_static'][0],)):
        transforms.extend(decode_tf_message(blob))
    basefoot_from_baselink = static_transform(transforms, values['base_frame'], values['laser_frame'])
    first = odoms[0][1]
    anchor = Transform2D((cell[0]+.5)*CELL_SIZE_M,(cell[1]+.5)*CELL_SIZE_M,HEADING_RAD[values['heading']])
    maze_from_odom = compose(anchor, inverse(Transform2D(first.x,first.y,first.yaw)))
    laser_origins = laser_origins_from_tf(transforms, values['laser_frame'])
    return db,topics,values,odoms,basefoot_from_baselink,maze_from_odom,laser_origins


def run(session=ROOT/'field_data/20261001_174719_joystick_full_maze', truth_path=ROOT/'field/maze_truth_7x7.json'):
    db, topics, values, odoms, basefoot_from_baselink, maze_from_odom, laser_origins = _inputs(Path(session))
    candidates=Counter(); eligible=Counter(); reasons=Counter(); per_edge=Counter()
    previous=None
    scan_id=topics[values['scan_topic']][0]
    try:
        for (blob,) in db.execute('select data from messages where topic_id=? order by timestamp',(scan_id,)):
            scan=decode_laser_scan(blob); stamp=scan['stamp']
            if scan['frame_id'] != values['laser_frame']:
                raise ValueError(f"scan frame {scan['frame_id']} is not {values['laser_frame']}")
            try: odom=_pose_at(odoms,stamp)
            except ValueError: reasons['NO_ODOM_BRACKET']+=1; previous=(stamp,None); continue
            if previous is None or previous[1] is None:
                reasons['NO_MOTION_BRACKET']+=1; previous=(stamp,odom); continue
            dt=stamp-previous[0]
            if dt<=0: reasons['NONPOSITIVE_DT']+=1; previous=(stamp,odom); continue
            speed=math.hypot(odom.x-previous[1].x,odom.y-previous[1].y)/dt
            dyaw=math.atan2(math.sin(odom.yaw-previous[1].yaw),math.cos(odom.yaw-previous[1].yaw))
            yawrate=abs(dyaw)/dt
            if speed>MAX_SPEED or yawrate>MAX_YAW_RATE:
                reasons['MOTION_GATE']+=1; previous=(stamp,odom); continue
            # scan coordinates are reconstructed base_link endpoints; TF locates
            # base_link relative to odom's base_footprint.
            maze_from_base=compose(maze_from_odom,compose(Transform2D(odom.x,odom.y,odom.yaw),basefoot_from_baselink))
            c,s=math.cos(maze_from_base.yaw),math.sin(maze_from_base.yaw)
            frame_votes=set(); frame_hit_edges=set()
            for i,r in enumerate(scan['ranges']):
                if not math.isfinite(r) or not scan['range_min']<=r<=scan['range_max']: continue
                a=scan['angle_min']+i*scan['angle_increment']
                # Fused scan is 1-degree binned. Half-bin endpoint uncertainty.
                bx,by=r*math.cos(a),r*math.sin(a)
                hx=maze_from_base.x+c*bx-s*by; hy=maze_from_base.y+s*bx+c*by
                hit=(hx,hy); candidate_sets=[]; max_u=.02 + r*math.sin(math.radians(.5)) + (speed+yawrate*r)*DT_BOUND
                frame_hit_edges.update(hit_edge_candidates(hit))
                for ox,oy in laser_origins:
                    mx=maze_from_base.x+c*ox-s*oy; my=maze_from_base.y+s*ox+c*oy
                    candidate_sets.append(crossing_edge((mx,my),hit,max_u))
                candidates['finite_returns']+=1
                common=set(candidate_sets[0])
                for edges in candidate_sets[1:]: common &= set(edges)
                if common:
                    frame_votes.update(common)
                else: reasons['SOURCE_OR_GEOMETRY_ABSTAIN']+=1
            vetoed=frame_votes & frame_hit_edges
            reasons['SAME_FRAME_WALL_CONFLICT']+=len(vetoed)
            frame_votes.difference_update(frame_hit_edges)
            for key in frame_votes:
                ident=edge_id(key); per_edge[ident]+=1; eligible['edge_votes']+=1
            eligible['frames']+=1
            previous=(stamp,odom)
    finally: db.close()
    # Load truth only after replay has emitted all source-only evidence.
    truth={EdgeMap(7)._fk(k):v for k,v in audit.edge_truth(json.loads(Path(truth_path).read_text())).items()}
    scored={k: v for k,v in truth.items() if per_edge[k]}
    edge_tp=sum(not truth[k] for k in scored); edge_fp=sum(truth[k] for k in scored)
    vote_tp=sum(n for k,n in per_edge.items() if not truth[k])
    vote_fp=sum(n for k,n in per_edge.items() if truth[k])
    return {
      'classification':'CONDITIONAL_DIAGNOSTIC_ONLY',
      'assumptions':{'source_age_bound_s':DT_BOUND,'speed_gate_mps':MAX_SPEED,'yaw_rate_gate_radps':MAX_YAW_RATE,'quantization_half_bin_deg':.5,'candidate_origins_base_link_m':laser_origins,'origins_source':'this bag /tf_static (laser0_frame then laser1_frame)','margin_hit_m':HIT_MARGIN,'margin_corner_m':CORNER_MARGIN},
      'frames_eligible':eligible['frames'],'finite_returns_examined':candidates['finite_returns'],'open_edge_frame_votes':eligible['edge_votes'],'unique_edges_voted':len(per_edge),'vote_edges_per_frame_at_most_one':True,
      'frame_count_by_edge':dict(sorted(per_edge.items())),'frame_edge_confusion':{'TP_open_votes':vote_tp,'FP_wall_misread_open_votes':vote_fp,'precision':vote_tp/(vote_tp+vote_fp) if vote_tp+vote_fp else None},
      'unique_edge_coverage':{'true_open_edges':edge_tp,'true_wall_edges':edge_fp,'precision':edge_tp/(edge_tp+edge_fp) if edge_tp+edge_fp else None},
      'reasons':dict(reasons),'truth_note':'Truth is used only after votes are complete; physical maze axes remain unresolved, so scores are conditional on the session anchor and ideal abstract grid.','time_note':'150 ms bounds assumed from observed scan/odom cadence; fused message does not bound per-source scan age.','bag_unchanged':True}


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--session',type=Path,default=ROOT/'field_data/20261001_174719_joystick_full_maze'); p.add_argument('--truth',type=Path,default=ROOT/'field/maze_truth_7x7.json'); p.add_argument('--output',type=Path,default=Path('/tmp/honor-cup-conservative-open.json')); a=p.parse_args()
    if a.output.parent != Path('/tmp'): raise SystemExit('output must be directly under /tmp')
    result=run(a.session,a.truth); a.output.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))

if __name__=='__main__': main()
