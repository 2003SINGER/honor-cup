#!/usr/bin/env python3
"""scan session summary —— frames.jsonl → summary.json + summary.csv.

在线 session 与离线重放共用本工具 (同一聚合链, 无第二套代码)。
绝不自动宣布可信距离 —— 只产出统计, 结论由人看完数据决定。"""

import argparse
import pathlib
import csv
import json
import os
import sqlite3
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.scan_diagnostics import summarize_frames  # noqa: E402


def audit_capture_topics(session_dir):
    """Count required scan/odom/TF messages across every finalized SQLite shard."""
    session = pathlib.Path(session_dir)
    metadata = {}
    for line in (session / 'session.yaml').read_text(encoding='utf-8').splitlines():
        if ':' in line:
            key, value = line.split(':', 1)
            metadata[key.strip()] = value.strip().strip('"\'')

    required = {
        metadata.get('scan_topic', '/scan_multi'): 'sensor_msgs/msg/LaserScan',
        '/scan0': 'sensor_msgs/msg/LaserScan',
        '/scan1': 'sensor_msgs/msg/LaserScan',
        metadata.get('odom_topic', '/odom_raw'): 'nav_msgs/msg/Odometry',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
    }
    totals = {name: {'expected_type': expected, 'type': None,
                     'message_count': 0}
              for name, expected in required.items()}
    db_files = sorted((session / 'bag' / 'record').rglob('*.db3'))
    errors = []
    for db_path in db_files:
        try:
            db = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
            rows = db.execute(
                'SELECT t.name,t.type,COUNT(m.id) FROM topics t '
                'LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id'
            ).fetchall()
            db.close()
        except Exception as exc:
            errors.append(f'{db_path.name}: {exc}')
            continue
        for name, msg_type, count in rows:
            if name not in totals:
                continue
            entry = totals[name]
            if entry['type'] not in (None, msg_type):
                errors.append(
                    f'{name}: inconsistent message types '
                    f'{entry["type"]} and {msg_type}')
            entry['type'] = msg_type
            entry['message_count'] += int(count)

    missing = [name for name, entry in totals.items()
               if entry['type'] != entry['expected_type'] or
               entry['message_count'] == 0]
    return {
        'complete': bool(db_files) and not missing and not errors,
        'bag_files': [str(path.relative_to(session)) for path in db_files],
        'required_topics': totals,
        'missing_or_empty_topics': missing,
        'errors': errors,
        'raw_bag_preserved': True,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('session_dir', help='包含 frames.jsonl 的 session 目录')
    args = ap.parse_args()

    frames_path = os.path.join(args.session_dir, 'frames.jsonl')
    frames = []
    with open(frames_path) as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    summary = summarize_frames(frames)

    with open(os.path.join(args.session_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    with open(os.path.join(args.session_dir, 'summary.csv'), 'w',
              newline='') as f:
        w = csv.writer(f)
        w.writerow(['range_bin', 'n', 'unique', 'ambiguous',
                    'unique_rate', 'ambiguous_rate',
                    'residual_p50_mm', 'residual_p90_mm', 'residual_p95_mm',
                    'incidence_p50_deg', 'incidence_p90_deg',
                    'corner_p10_mm'])
        for b in summary['by_range_bin']:
            w.writerow([b['range_m'], b['n'], b['unique'], b['ambiguous'],
                        b['unique_rate'], b['ambiguous_rate'],
                        b['residual_p50_mm'], b['residual_p90_mm'],
                        b['residual_p95_mm'], b['incidence_p50_deg'],
                        b['incidence_p90_deg'], b['corner_p10_mm']])
    print(f"frames={summary['n_frames']} valid_beams={summary['n_valid']} "
          f"unique_rate={summary['unique_rate']} "
          f"ambiguous_rate={summary['ambiguous_rate']}")
    print(f"summary: {os.path.join(args.session_dir, 'summary.json')}")


if __name__ == '__main__':
    main()
