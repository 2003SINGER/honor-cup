"""Tests for the ROS-free rosbag CDR edge analysis helpers."""

import math
import struct
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import analyze_scan_edge as analysis


class CDRWriter:
    def __init__(self):
        self.data = bytearray(b'\x00\x01\x00\x00')

    def write(self, fmt, *values):
        alignment = max({'b': 1, 'B': 1, 'h': 2, 'H': 2,
                         'i': 4, 'I': 4, 'f': 4, 'q': 8,
                         'Q': 8, 'd': 8}[c] for c in fmt)
        relative = len(self.data) - 4
        self.data.extend(b'\x00' * ((-relative) % alignment))
        self.data.extend(struct.pack('<' + fmt, *values))

    def string(self, value):
        raw = value.encode() + b'\x00'
        self.write('I', len(raw))
        self.data.extend(raw)


class AnalyzeScanEdgeTest(unittest.TestCase):
    def test_decode_laserscan_header_float_fields_and_ranges(self):
        w = CDRWriter()
        w.write('iI', 12, 345)
        w.string('base_link')
        w.write('fff', -1.0, 1.0, 0.5)
        w.write('ffff', 0.01, 0.1, 0.05, 4.0)
        w.write('I', 2)
        w.write('f', 0.75)
        w.write('f', float('inf'))
        w.write('I', 2)
        w.write('ff', 2.0, 3.0)

        scan = analysis.decode_laser_scan(bytes(w.data))
        self.assertAlmostEqual(scan['stamp'], 12.000000345)
        self.assertEqual(scan['frame_id'], 'base_link')
        self.assertEqual(scan['angle_min'], -1.0)
        self.assertEqual(scan['angle_increment'], 0.5)
        self.assertAlmostEqual(scan['range_min'], 0.05)
        self.assertEqual(scan['ranges'][0], 0.75)
        self.assertTrue(math.isinf(scan['ranges'][1]))

    def test_static_transform_composes_parent_child_chain_and_inverse(self):
        parent_from_mid = analysis.Transform2D(1.0, 0.0, math.pi / 2)
        mid_from_sensor = analysis.Transform2D(1.0, 0.0, 0.0)
        edges = [('parent', 'mid', parent_from_mid),
                 ('mid', 'sensor', mid_from_sensor)]

        parent_from_sensor = analysis.static_transform(edges, 'parent', 'sensor')
        self.assertAlmostEqual(parent_from_sensor.x, 1.0)
        self.assertAlmostEqual(parent_from_sensor.y, 1.0)
        self.assertAlmostEqual(parent_from_sensor.yaw, math.pi / 2)

        sensor_from_parent = analysis.static_transform(edges, 'sensor', 'parent')
        self.assertAlmostEqual(sensor_from_parent.x, -1.0)
        self.assertAlmostEqual(sensor_from_parent.y, 1.0)
        self.assertAlmostEqual(sensor_from_parent.yaw, -math.pi / 2)

    def test_static_transform_fails_when_frames_are_disconnected(self):
        with self.assertRaisesRegex(ValueError, 'no /tf_static path'):
            analysis.static_transform([], 'base_footprint', 'laser')

    def test_segment_overshoot_is_measured_along_the_edge(self):
        self.assertEqual(analysis.segment_overshoot(0.2, 0.0, 0.4), 0.0)
        self.assertAlmostEqual(analysis.segment_overshoot(-0.15, 0.0, 0.4),
                               0.15)
        self.assertAlmostEqual(analysis.segment_overshoot(0.55, 0.0, 0.4),
                               0.15)


if __name__ == '__main__':
    unittest.main()
