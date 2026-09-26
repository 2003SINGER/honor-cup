#!/usr/bin/env python3
"""scan session summary —— frames.jsonl → summary.json + summary.csv.

在线 session 与离线重放共用本工具 (同一聚合链, 无第二套代码)。
绝不自动宣布可信距离 —— 只产出统计, 结论由人看完数据决定。"""

import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.scan_diagnostics import summarize_frames  # noqa: E402


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
