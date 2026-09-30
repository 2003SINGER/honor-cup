#!/usr/bin/env python3
"""Read-only raw LaserScan geometry audit for one manually anchored grid edge.

Uses only Python's standard library. It reads rosbag2 SQLite/CDR data plus
session.yaml, resolves the static base<-laser transform from /tf_static, and
projects valid scan hits into the manually anchored maze frame. It never edits
the bag or session files.

Example:
  python3 software/tools/analyze_scan_edge.py field_data/<session> \
      --axis y --coord 2.0 --along-min 0 --along-max 0.4
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


CELL_SIZE_M = 0.4
HEADING_RAD = {'N': math.pi / 2, 'E': 0.0,
               'S': -math.pi / 2, 'W': math.pi}


class CDRReader:
    """Small little-endian CDR reader for the ROS messages used here."""

    def __init__(self, data: bytes):
        if len(data) < 4 or data[0] != 0 or data[1] != 1:
            raise ValueError('expected a little-endian CDR encapsulation header')
        self.data = data
        self.pos = 4

    def _align(self, size: int) -> None:
        relative = self.pos - 4
        self.pos = 4 + (relative + size - 1) // size * size

    def read(self, fmt: str):
        size = struct.calcsize('<' + fmt)
        alignments = {'b': 1, 'B': 1, 'h': 2, 'H': 2,
                      'i': 4, 'I': 4, 'f': 4, 'q': 8,
                      'Q': 8, 'd': 8}
        self._align(max(alignments[char] for char in fmt))
        end = self.pos + size
        if end > len(self.data):
            raise ValueError('truncated CDR message')
        values = struct.unpack_from('<' + fmt, self.data, self.pos)
        self.pos = end
        return values[0] if len(values) == 1 else values

    def read_string(self) -> str:
        size = self.read('I')
        if size == 0 or self.pos + size > len(self.data):
            raise ValueError('invalid CDR string length')
        raw = self.data[self.pos:self.pos + size]
        self.pos += size
        if raw[-1] != 0:
            raise ValueError('CDR string is not null terminated')
        return raw[:-1].decode('utf-8')

    def read_header(self) -> tuple[float, str]:
        sec = self.read('i')
        nsec = self.read('I')
        if sec < 0 or nsec >= 1_000_000_000:
            raise ValueError('invalid ROS header timestamp')
        return sec + nsec * 1e-9, self.read_string()


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Transform2D:
    """Rigid transform p_target = T * p_source."""

    x: float
    y: float
    yaw: float


def compose(a: Transform2D, b: Transform2D) -> Transform2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Transform2D(a.x + c * b.x - s * b.y,
                       a.y + s * b.x + c * b.y,
                       math.atan2(math.sin(a.yaw + b.yaw),
                                  math.cos(a.yaw + b.yaw)))


def inverse(t: Transform2D) -> Transform2D:
    c, s = math.cos(t.yaw), math.sin(t.yaw)
    return Transform2D(-c * t.x - s * t.y,
                       s * t.x - c * t.y,
                       -t.yaw)


def quaternion_yaw(q: tuple[float, float, float, float]) -> float:
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9 or abs(norm - 1.0) > 1e-3:
        raise ValueError(f'invalid quaternion norm {norm}')
    x, y, z, w = (v / norm for v in q)
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def decode_laser_scan(data: bytes) -> dict:
    r = CDRReader(data)
    stamp, frame_id = r.read_header()
    angle_min, angle_max, angle_increment = r.read('fff')
    time_increment, scan_time, range_min, range_max = r.read('ffff')
    n_ranges = r.read('I')
    if n_ranges > 1_000_000:
        raise ValueError(f'implausible LaserScan length: {n_ranges}')
    ranges = tuple(r.read('f') for _ in range(n_ranges))
    n_intensities = r.read('I')
    if n_intensities > 1_000_000:
        raise ValueError(f'implausible intensity length: {n_intensities}')
    return {
        'stamp': stamp, 'frame_id': frame_id,
        'angle_min': angle_min, 'angle_max': angle_max,
        'angle_increment': angle_increment,
        'range_min': range_min, 'range_max': range_max,
        'ranges': ranges,
    }


def decode_odometry(data: bytes) -> tuple[float, str, str, Pose2D]:
    r = CDRReader(data)
    stamp, frame_id = r.read_header()
    child_frame = r.read_string()
    x, y, _z = r.read('ddd')
    q = r.read('dddd')
    pose = Pose2D(x, y, quaternion_yaw(q))
    return stamp, frame_id, child_frame, pose


def decode_tf_message(data: bytes) -> list[tuple[str, str, Transform2D]]:
    r = CDRReader(data)
    count = r.read('I')
    if count > 100_000:
        raise ValueError(f'implausible TFMessage length: {count}')
    transforms = []
    for _ in range(count):
        _stamp, parent = r.read_header()
        child = r.read_string()
        x, y, _z = r.read('ddd')
        q = r.read('dddd')
        transforms.append((parent, child,
                           Transform2D(x, y, quaternion_yaw(q))))
    return transforms


def static_transform(edges: Iterable[tuple[str, str, Transform2D]],
                     target: str, source: str) -> Transform2D:
    """Resolve target<-source through a static TF tree."""
    graph: dict[str, list[tuple[str, Transform2D]]] = {}
    for parent, child, parent_from_child in edges:
        graph.setdefault(child, []).append((parent, parent_from_child))
        graph.setdefault(parent, []).append((child, inverse(parent_from_child)))
    queue = [(source, Transform2D(0.0, 0.0, 0.0))]
    seen = {source}
    for frame, frame_from_source in queue:
        if frame == target:
            return frame_from_source
        for neighbor, neighbor_from_frame in graph.get(frame, []):
            if neighbor in seen:
                continue
            seen.add(neighbor)
            queue.append((neighbor, compose(neighbor_from_frame,
                                            frame_from_source)))
    raise ValueError(f'no /tf_static path from {source!r} to {target!r}')


def interpolate_pose(samples: list[tuple[float, Pose2D]], stamp: float) -> Pose2D:
    times = [sample[0] for sample in samples]
    i = bisect.bisect_left(times, stamp)
    if i == 0:
        if times[0] - stamp > 0.5:
            raise ValueError('no nearby odometry before scan')
        return samples[0][1]
    if i == len(samples):
        if stamp - times[-1] > 0.5:
            raise ValueError('no nearby odometry after scan')
        return samples[-1][1]
    ta, a = samples[i - 1]
    tb, b = samples[i]
    if tb - ta > 0.5:
        raise ValueError('odom gap too large to interpolate scan pose')
    f = (stamp - ta) / (tb - ta) if tb != ta else 0.0
    dyaw = math.atan2(math.sin(b.yaw - a.yaw), math.cos(b.yaw - a.yaw))
    return Pose2D(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
                  a.yaw + f * dyaw)


def load_session_value(path: Path, key: str) -> str:
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith(key + ':'):
            return line.split(':', 1)[1].strip().strip('"\'')
    raise ValueError(f'{path} is missing {key}')


def quantile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * q)]


def line_fit(points: list[tuple[float, float]]) -> tuple[float, float,
                                                          list[float]] | None:
    """Fit normal=m*along+b; return slope, intercept and signed residuals."""
    if len(points) < 2:
        return None
    along = [p[0] for p in points]
    normal = [p[1] for p in points]
    ma, mn = sum(along) / len(along), sum(normal) / len(normal)
    denom = sum((v - ma) ** 2 for v in along)
    if denom <= 1e-12:
        return None
    slope = sum((x - ma) * (y - mn)
                for x, y in zip(along, normal)) / denom
    intercept = mn - slope * ma
    return slope, intercept, [y - (slope*x + intercept)
                              for x, y in points]


def segment_overshoot(along: float, start: float, end: float) -> float:
    """Distance along the fitted edge beyond its requested segment."""
    return max(start - along, along - end, 0.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir', type=Path)
    parser.add_argument('--axis', choices=('x', 'y'), required=True,
                        help='normal axis: x for vertical walls, y for horizontal')
    parser.add_argument('--coord', type=float, required=True,
                        help='expected wall coordinate on the normal axis (m)')
    parser.add_argument('--along-min', type=float, required=True)
    parser.add_argument('--along-max', type=float, required=True)
    parser.add_argument('--tolerance', type=float, default=0.04,
                        help='hit tolerance from expected line (m; default 0.04)')
    parser.add_argument('--endpoint-guard', type=float, default=0.0,
                        help='also count returns up to this distance past each '
                             'segment endpoint (m; default off)')
    args = parser.parse_args()
    session = args.session_dir
    if (args.along_min >= args.along_max or args.tolerance <= 0
            or args.endpoint_guard < 0):
        parser.error('along-min must be below along-max, tolerance positive, '
                     'and endpoint-guard nonnegative')

    meta = session / 'session.yaml'
    cell = load_session_value(meta, 'cell').strip('[]').split(',')
    cx, cy = (int(v.strip()) for v in cell)
    heading = load_session_value(meta, 'heading')
    if heading not in HEADING_RAD:
        raise ValueError(f'unsupported heading: {heading}')
    scan_topic = load_session_value(meta, 'scan_topic')
    odom_topic = load_session_value(meta, 'odom_topic')
    laser_frame = load_session_value(meta, 'laser_frame')
    odom_frame = load_session_value(meta, 'odom_frame')
    base_frame = load_session_value(meta, 'base_frame')

    bag_files = sorted((session / 'bag').glob('**/*.db3'))
    if len(bag_files) != 1:
        raise ValueError(f'expected exactly one SQLite bag, found {len(bag_files)}')
    db = sqlite3.connect(f'file:{bag_files[0]}?mode=ro', uri=True)
    topics = {name: (topic_id, msg_type)
              for topic_id, name, msg_type
              in db.execute('SELECT id, name, type FROM topics')}
    for name in (scan_topic, odom_topic, '/tf_static'):
        if name not in topics:
            raise ValueError(f'bag has no required topic {name}')
    if topics[scan_topic][1] != 'sensor_msgs/msg/LaserScan':
        raise ValueError(f'{scan_topic} is not a LaserScan topic')
    if topics[odom_topic][1] != 'nav_msgs/msg/Odometry':
        raise ValueError(f'{odom_topic} is not an Odometry topic')

    odom_samples = []
    for (_bag_stamp, data) in db.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[odom_topic][0],)):
        stamp, frame, child, pose = decode_odometry(data)
        if frame != odom_frame or child != base_frame:
            raise ValueError(f'odometry frame mismatch: {frame!r}/{child!r}')
        odom_samples.append((stamp, pose))
    if not odom_samples:
        raise ValueError('bag contains no odometry messages')
    odom_samples.sort(key=lambda x: x[0])

    static_edges = []
    for (_bag_stamp, data) in db.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics['/tf_static'][0],)):
        static_edges.extend(decode_tf_message(data))
    base_from_laser = static_transform(static_edges, base_frame, laser_frame)

    maze_anchor = Pose2D((cx + 0.5) * CELL_SIZE_M,
                         (cy + 0.5) * CELL_SIZE_M,
                         HEADING_RAD[heading])
    map_from_odom = compose(Transform2D(maze_anchor.x, maze_anchor.y,
                                       maze_anchor.yaw),
                            inverse(Transform2D(odom_samples[0][1].x,
                                                odom_samples[0][1].y,
                                                odom_samples[0][1].yaw)))

    along_axis = 'x' if args.axis == 'y' else 'y'
    normal_axis = args.axis
    frame_counts: list[int] = []
    frame_fits = []
    selected_points: list[tuple[float, float]] = []
    guarded_points: list[tuple[float, float]] = []
    guarded_frame_counts: list[int] = []
    overshoots: list[tuple[float, int]] = []
    selected_all = 0
    scan_count = 0
    valid_returns = 0
    skipped_pose = 0

    for (_bag_stamp, data) in db.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (topics[scan_topic][0],)):
        scan = decode_laser_scan(data)
        if scan['frame_id'] != laser_frame:
            raise ValueError(f'scan frame mismatch: {scan["frame_id"]!r} '
                             f'!= session laser_frame {laser_frame!r}')
        try:
            odom_pose = interpolate_pose(odom_samples, scan['stamp'])
        except ValueError:
            skipped_pose += 1
            continue
        scan_count += 1
        odom_from_base = Transform2D(odom_pose.x, odom_pose.y,
                                     odom_pose.yaw)
        map_from_laser = compose(map_from_odom,
                                 compose(odom_from_base, base_from_laser))
        c, s = math.cos(map_from_laser.yaw), math.sin(map_from_laser.yaw)
        frame_points = []
        frame_guard_points = []
        for i, distance in enumerate(scan['ranges']):
            if (not math.isfinite(distance)
                    or distance < scan['range_min']
                    or distance > scan['range_max']):
                continue
            valid_returns += 1
            angle = scan['angle_min'] + i * scan['angle_increment']
            lx, ly = distance * math.cos(angle), distance * math.sin(angle)
            mx = map_from_laser.x + c * lx - s * ly
            my = map_from_laser.y + s * lx + c * ly
            along = mx if along_axis == 'x' else my
            normal = mx if normal_axis == 'x' else my
            if abs(normal - args.coord) <= args.tolerance:
                point = (along, normal)
                overshoot = segment_overshoot(
                    along, args.along_min, args.along_max)
                if overshoot == 0.0:
                    frame_points.append(point)
                if overshoot <= args.endpoint_guard:
                    frame_guard_points.append(point)
                    overshoots.append((overshoot, scan_count - 1))
        frame_counts.append(len(frame_points))
        guarded_frame_counts.append(len(frame_guard_points))
        selected_all += len(frame_points)
        selected_points.extend(frame_points)
        guarded_points.extend(frame_guard_points)
        fit = line_fit(frame_points)
        if fit:
            slope, intercept, residuals = fit
            frame_fits.append((slope, intercept,
                               [abs(x) for x in residuals],
                               min(p[0] for p in frame_points),
                               max(p[0] for p in frame_points)))

    db.close()
    if not frame_counts:
        raise ValueError('no LaserScan frames had a usable odometry pose')

    fit_p95 = [quantile(residuals, 0.95) for _, _, residuals, _, _ in frame_fits]
    fit_p95 = [x for x in fit_p95 if x is not None]
    result = {
        'session': str(session),
        'anchor': {'cell': [cx, cy], 'heading': heading,
                   'cell_size_m': CELL_SIZE_M},
        'edge': {'axis': args.axis, 'coord_m': args.coord,
                 'along_min_m': args.along_min,
                 'along_max_m': args.along_max,
                 'tolerance_m': args.tolerance},
        'bag': {'scan_frames_with_pose': scan_count,
                'scans_skipped_no_pose': skipped_pose,
                'odom_samples': len(odom_samples),
                'valid_raw_returns': valid_returns},
        'raw_edge_support': {
            'returns': selected_all,
            'supported_frames': sum(n > 0 for n in frame_counts),
            'support_rate': round(sum(n > 0 for n in frame_counts)
                                  / len(frame_counts), 6),
            'hits_per_frame_min_median_p95_max': [
                min(frame_counts), quantile(frame_counts, 0.5),
                quantile(frame_counts, 0.95), max(frame_counts)],
            'observed_along_extent_m': [
                min((p[0] for p in selected_points), default=None),
                max((p[0] for p in selected_points), default=None)],
            'abs_expected_line_residual_m_p50_p90_p95': [
                quantile([abs(p[1] - args.coord) for p in selected_points], q)
                for q in (0.5, 0.9, 0.95)],
        },
        'fitted_line': {
            'frames_fit': len(frame_fits),
            'slope_normal_vs_along_p05_median_p95': [
                quantile([x[0] for x in frame_fits], q)
                for q in (0.05, 0.5, 0.95)],
            'intercept_m_p05_median_p95': [
                quantile([x[1] for x in frame_fits], q)
                for q in (0.05, 0.5, 0.95)],
            'span_m_p05_median_p95': [
                quantile([x[4] - x[3] for x in frame_fits], q)
                for q in (0.05, 0.5, 0.95)],
            'per_frame_abs_residual_p95_m_p50_p95': [
                quantile(fit_p95, 0.5), quantile(fit_p95, 0.95)],
        },
        'endpoint_guard': None,
        'interpretation_limit': (
            'Repeated scans from a stationary pose are correlated observations; '
            'support extent describes visible returns, not wall continuity beyond '
            'the requested edge segment.'),
    }
    if args.endpoint_guard > 0:
        bin_width = 0.05
        bins = []
        bin_count = math.ceil(args.endpoint_guard / bin_width)
        for i in range(bin_count):
            lo = i * bin_width
            hi = min((i + 1) * bin_width, args.endpoint_guard)
            selected = [(d, frame) for d, frame in overshoots
                        if lo <= d < hi
                        or (i == bin_count - 1 and lo <= d <= hi)]
            bins.append({
                'overshoot_m': [lo, hi],
                'returns': len(selected),
                'frames': len({frame for _, frame in selected}),
            })
        guard_only = [d for d, _ in overshoots if d > 0]
        result['endpoint_guard'] = {
            'guard_m': args.endpoint_guard,
            'returns_within_segment_or_guard': len(guarded_points),
            'returns_past_segment_endpoint': len(guard_only),
            'frames_with_any_guarded_return': sum(
                n > 0 for n in guarded_frame_counts),
            'overshoot_m_p50_p90_p95_max': [
                quantile(guard_only, q) for q in (0.5, 0.9, 0.95, 1.0)],
            'bins': bins,
        }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
