#!/usr/bin/env python3
"""Score real joystick rosbag scans against field edge truth, offline only.

The truth map is used exclusively after GridAssociation/RealObservationAdapter
produce edge evidence. Every physical edge receives at most one vote per
distinct source scan stamp through EdgeMap's own vote de-duplication.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / 'software' / 'ros2' / 'm3pro_nav'))
import audit_lidar_odom_calibration as audit
from m3pro_nav.edge_map import EdgeMap, WALL, OPEN, UNKNOWN
from m3pro_nav.observation_adapter import RealObservationAdapter
from m3pro_nav.grid_association import GridAssociation
from m3pro_nav.trust_policy import TrustPolicy


def edge_class(edge_map, key):
    if key[0] == 'B':
        _, cell, direction = key
    else:
        cell, direction = key
    return edge_map.state(cell, direction)


def score(states, truth):
    c = Counter()
    for edge, expected_wall in truth.items():
        state = states.get(edge, UNKNOWN)
        if state == UNKNOWN:
            c['unknown'] += 1
        elif expected_wall and state == WALL:
            c['wall_correct'] += 1
        elif expected_wall:
            c['truth_wall_written_open'] += 1
        elif state == OPEN:
            c['open_correct'] += 1
        else:
            c['truth_open_written_wall'] += 1
    c['edges'] = len(truth)
    return dict(c)


def process(session: Path, truth: dict, *, limit_s=None):
    meta = session / 'session.yaml'
    cell = tuple(int(v.strip()) for v in audit.load_session_value(meta, 'cell').strip('[]').split(','))
    heading = audit.load_session_value(meta, 'heading')
    scan_topic = audit.load_session_value(meta, 'scan_topic')
    odom_topic = audit.load_session_value(meta, 'odom_topic')
    laser_frame = audit.load_session_value(meta, 'laser_frame')
    odom_frame = audit.load_session_value(meta, 'odom_frame')
    base_frame = audit.load_session_value(meta, 'base_frame')
    bag = next((session / 'bag').glob('**/*.db3'))
    db = sqlite3.connect(f'file:{bag}?mode=ro', uri=True)
    topics = {name: tid for tid, name, _ in db.execute('SELECT id,name,type FROM topics')}
    odoms = []
    for _, blob in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (topics[odom_topic],)):
        stamp, frame, child, pose = audit.decode_odometry(blob)
        if (frame, child) != (odom_frame, base_frame):
            raise ValueError(f'odometry frames mismatch {frame}/{child}')
        odoms.append((stamp, pose))
    odoms.sort(key=lambda row: row[0])
    if not odoms:
        raise ValueError('no odometry')
    tf = []
    for _, blob in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (topics['/tf_static'],)):
        tf.extend(audit.decode_tf_message(blob))
    base_laser = audit.static_transform(tf, base_frame, laser_frame)
    first = odoms[0][1]
    anchor = audit.Transform2D((cell[0]+.5)*.4, (cell[1]+.5)*.4, audit.HEADING_RAD[heading])
    maze_odom = audit.compose(anchor, audit.inverse(audit.Transform2D(first.x, first.y, first.yaw)))
    association = GridAssociation()
    adapter = RealObservationAdapter(association)
    policy = TrustPolicy(diagnostic_only=True)
    policy_rejections = Counter()
    edge_evidence = defaultdict(Counter)
    first_evidence = {}
    first_wrong_state = {}
    maps = {name: EdgeMap(7) for name in ('all', 'static', 'moving', 'early', 'late')}
    scans = []
    start_stamp = None
    last_distinct = None
    duplicate_stamps = 0
    for bag_ns, blob in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (topics[scan_topic],)):
        scan = audit.decode_laser_scan(blob)
        if scan['frame_id'] != laser_frame:
            raise ValueError(f'scan frame mismatch {scan["frame_id"]}')
        if start_stamp is None:
            start_stamp = scan['stamp']
        elapsed = scan['stamp'] - start_stamp
        if limit_s is not None and elapsed > limit_s:
            continue
        try:
            pose = audit.interpolate_pose(odoms, scan['stamp'])
        except ValueError:
            continue
        maze_pose, rays, _ = audit.scan_frame(scan, pose, maze_odom, base_laser)
        hits, opens, _stats = adapter.to_nav_observation(rays, maze_pose, stamp=scan['stamp'])
        # Mirror the currently configured production TrustPolicy gate for reporting.
        for obs in association.process([ray for ray in rays if ray.valid]):
            policy_rejections[policy.evaluate(obs, stamp=scan['stamp']).reason] += 1
        # Motion label from nearest bracketing odometry, retaining all moving scans.
        idx = bisect.bisect_left([x[0] for x in odoms], scan['stamp'])
        lo, hi = max(0, idx-1), min(len(odoms)-1, idx)
        dt = odoms[hi][0]-odoms[lo][0]
        speed = math.hypot(odoms[hi][1].x-odoms[lo][1].x, odoms[hi][1].y-odoms[lo][1].y)/dt if dt > 0 else 0.0
        dyaw = math.atan2(math.sin(odoms[hi][1].yaw-odoms[lo][1].yaw), math.cos(odoms[hi][1].yaw-odoms[lo][1].yaw))
        yaw_rate = abs(dyaw)/dt if dt > 0 else 0.0
        moving = speed > .03 or yaw_rate > .05
        records = []
        for (_cell, _d), (dist, _alpha) in hits.items():
            key = (_cell, _d)
            records.append((key, WALL, dist))
        for _cell, _d, dist, _alpha in opens:
            records.append(((_cell, _d), OPEN, dist))
        # A hit wins within a single scan; adapter already suppresses plausible-hit conflicts,
        # but this also makes stamp-level dedup explicit for repeated source stamps.
        per_scan = {}
        for key, state, dist in records:
            if state == WALL or key not in per_scan:
                per_scan[key] = (state, dist)
        if last_distinct == scan['stamp']:
            duplicate_stamps += 1
        last_distinct = scan['stamp']
        for key, (state, dist) in per_scan.items():
            key = maps['all'].edge_key(*_edge_parts(key))
            edge_evidence[key][state] += 1
            first_evidence.setdefault((key, state), scan['stamp'])
            for name in ('all', 'moving' if moving else 'static', 'early' if elapsed <= 20 else 'late'):
                if state == WALL:
                    maps[name].observe_wall(*_edge_parts(key), dist=dist, stamp=scan['stamp'])
                else:
                    maps[name].observe_open(*_edge_parts(key), dist=dist, stamp=scan['stamp'])
            observed_state = edge_class(maps['all'], key)
            expected_state = WALL if truth[key] else OPEN
            if observed_state == (OPEN if truth[key] else WALL):
                first_wrong_state.setdefault(key, scan['stamp'])
        scans.append({'stamp': scan['stamp'], 'elapsed_s': elapsed, 'moving': moving, 'speed_mps': speed, 'yaw_rate_radps': yaw_rate})
    db.close()
    wrong = []
    all_states = {edge: edge_class(maps['all'], edge) for edge in truth}
    for edge, state in all_states.items():
        expected = WALL if truth[edge] else OPEN
        if state in (WALL, OPEN) and state != expected:
            bad_state = state
            wrong.append({'edge': repr(edge), 'truth': expected, 'written': state,
                          'first_wrong_stamp': first_wrong_state.get(edge),
                          'votes': dict(edge_evidence[edge])})
    wrong_wall_to_open = [row for row in wrong if row['truth'] == WALL]
    result = {'session': session.name, 'scans': len(scans), 'duplicate_source_stamps': duplicate_stamps,
              'duration_s': scans[-1]['elapsed_s'] if scans else 0,
              'motion_scan_counts': dict(Counter('moving' if x['moving'] else 'static' for x in scans)),
              'formal_trust_policy_diagnostic_only_rejections': dict(policy_rejections),
              'wrong_edge_examples_wall_written_open': wrong_wall_to_open[:5],
              'groups': {name: score({edge: edge_class(m, edge) for edge in truth}, truth) for name, m in maps.items()},
              'method': 'diagnostic GridAssociation/RealObservationAdapter outputs replayed into soft EdgeMap (2 distinct-stamp evidence threshold); parallel formal TrustPolicy(diagnostic_only=True) rejection count; no hard boundary evidence; truth used only after predictions for scoring; static/moving threshold speed>0.03m/s or yaw-rate>0.05rad/s'}
    return result


def _edge_parts(key):
    if key[0] == 'B':
        return key[1], key[2]
    return key[0], key[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sessions', nargs='*', type=Path)
    p.add_argument('--limit-s', type=float)
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    sessions = a.sessions or sorted((ROOT/'field_data').glob('20261001_17*_joystick_full_maze'))
    truth = audit.edge_truth(json.loads((ROOT/'field/maze_truth_7x7.json').read_text()))
    report = [process(s, truth, limit_s=a.limit_s) for s in sessions]
    data = json.dumps(report, indent=2)
    if a.output:
        a.output.write_text(data+'\n')
    else:
        print(data)

if __name__ == '__main__':
    main()
