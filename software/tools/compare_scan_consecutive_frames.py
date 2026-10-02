#!/usr/bin/env python3
"""Read-only comparison of consecutive LaserScan frames in a ROS 2 SQLite bag.

Reports exact range-array repeats, valid-beam coverage, per-beam changes, and
header/arrival timing separately for /scan0 and /scan1. It reports LaserScan's
declared scan_time as metadata only; it does not treat that field as the actual
physical revolution period. It does not modify the bag or require ROS packages.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software" / "tools"))

from analyze_scan_edge import CDRReader  # noqa: E402


def decode_scan(data: bytes) -> dict:
    reader = CDRReader(data)
    stamp, frame_id = reader.read_header()
    angle_min, angle_max, angle_increment = reader.read("fff")
    time_increment, scan_time, range_min, range_max = reader.read("ffff")
    count = reader.read("I")
    if count > 1_000_000:
        raise ValueError(f"implausible LaserScan range count: {count}")
    start = reader.pos
    ranges = tuple(reader.read("f") for _ in range(count))
    range_bytes = data[start : start + count * 4]
    intensity_count = reader.read("I")
    for _ in range(intensity_count):
        reader.read("f")
    return {
        "stamp": stamp,
        "frame_id": frame_id,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": angle_increment,
        "scan_time": scan_time,
        "range_min": range_min,
        "range_max": range_max,
        "ranges": ranges,
        "range_bytes": range_bytes,
    }


def quantile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * q
    low = math.floor(index)
    high = math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def numeric_summary(values: list[float]) -> dict:
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return {"n": 0}
    return {
        "n": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": quantile(values, 0.95),
        "max": max(values),
    }


def summarize_topic(db: sqlite3.Connection, topic_id: int, topic_name: str) -> dict:
    frames = []
    query = "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp"
    for arrival_ns, blob in db.execute(query, (topic_id,)):
        scan = decode_scan(blob)
        scan["arrival"] = arrival_ns * 1e-9
        frames.append(scan)

    if not frames:
        return {"topic": topic_name, "frames": 0, "error": "no messages"}

    valid_counts = [
        sum(math.isfinite(d) and scan["range_min"] <= d <= scan["range_max"] for d in scan["ranges"])
        for scan in frames
    ]
    pair_abs_diffs = []
    pair_common_valid = []
    pair_changed_bins = []
    exact_pairs = 0
    comparable_pairs = 0
    header_dt = []
    arrival_dt = []
    for previous, current in zip(frames, frames[1:]):
        comparable_pairs += 1
        exact_pairs += previous["range_bytes"] == current["range_bytes"]
        header_dt.append(current["stamp"] - previous["stamp"])
        arrival_dt.append(current["arrival"] - previous["arrival"])
        if len(previous["ranges"]) != len(current["ranges"]):
            pair_common_valid.append(0.0)
            continue
        overlap = []
        changed = 0
        for a, b in zip(previous["ranges"], current["ranges"]):
            a_valid = math.isfinite(a) and previous["range_min"] <= a <= previous["range_max"]
            b_valid = math.isfinite(b) and current["range_min"] <= b <= current["range_max"]
            if a_valid and b_valid:
                delta = abs(a - b)
                overlap.append(delta)
                changed += delta > 0
        pair_common_valid.append(len(overlap) / max(1, len(current["ranges"])))
        if overlap:
            pair_abs_diffs.extend(overlap)
            pair_changed_bins.append(changed / len(overlap))

    return {
        "topic": topic_name,
        "frames": len(frames),
        "beam_count_min_max": [min(map(len, (f["ranges"] for f in frames))), max(map(len, (f["ranges"] for f in frames)))],
        "valid_beams_per_frame": numeric_summary([float(n) for n in valid_counts]),
        "valid_coverage_fraction_per_frame": numeric_summary(
            [n / max(1, len(scan["ranges"])) for n, scan in zip(valid_counts, frames)]
        ),
        "declared_scan_time_s_metadata_only": numeric_summary([f["scan_time"] for f in frames]),
        "consecutive_pairs": comparable_pairs,
        "exact_equal_full_ranges_pairs": exact_pairs,
        "exact_equal_full_ranges_fraction": exact_pairs / comparable_pairs if comparable_pairs else None,
        "header_stamp_dt_s": numeric_summary(header_dt),
        "bag_arrival_dt_s": numeric_summary(arrival_dt),
        "common_valid_coverage_fraction_per_pair": numeric_summary(pair_common_valid),
        "pointwise_abs_range_delta_m_on_common_valid_beams": numeric_summary(pair_abs_diffs),
        "changed_beam_fraction_per_pair": numeric_summary(pair_changed_bins),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 rosbag2 SQLite .db3 file")
    parser.add_argument("--topics", nargs=2, default=("/scan0", "/scan1"), metavar=("TOPIC0", "TOPIC1"))
    args = parser.parse_args()
    db = sqlite3.connect(f"file:{args.bag.resolve()}?mode=ro", uri=True)
    try:
        topics = {name: (topic_id, msg_type) for topic_id, name, msg_type in db.execute("SELECT id,name,type FROM topics")}
        reports = []
        for name in args.topics:
            if name not in topics:
                reports.append({"topic": name, "frames": 0, "error": "topic missing from bag"})
                continue
            topic_id, msg_type = topics[name]
            if msg_type != "sensor_msgs/msg/LaserScan":
                reports.append({"topic": name, "frames": 0, "error": f"unexpected type: {msg_type}"})
                continue
            reports.append(summarize_topic(db, topic_id, name))
    finally:
        db.close()
    print(json.dumps({"bag": str(args.bag), "reports": reports}, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
