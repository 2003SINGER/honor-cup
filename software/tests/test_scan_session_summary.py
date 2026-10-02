"""Raw rosbag topic completeness tests for the joystick capture finalizer."""
import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
from scan_session_summary import audit_capture_topics


EXPECTED = {
    '/scan_multi': 'sensor_msgs/msg/LaserScan',
    '/scan0': 'sensor_msgs/msg/LaserScan',
    '/scan1': 'sensor_msgs/msg/LaserScan',
    '/odom_raw': 'nav_msgs/msg/Odometry',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}


def _write_shard(path, topics):
    db = sqlite3.connect(path)
    db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT)')
    db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)')
    for topic_id, (name, msg_type, count) in enumerate(topics, 1):
        db.execute('INSERT INTO topics VALUES(?,?,?)',
                   (topic_id, name, msg_type))
        for index in range(count):
            db.execute('INSERT INTO messages VALUES(?,?,?,?)',
                       (None, topic_id, index, sqlite3.Binary(b'x')))
    db.commit()
    db.close()


class AuditCaptureTopicsTest(unittest.TestCase):
    def _session(self, root):
        session = Path(root)
        (session / 'bag' / 'record').mkdir(parents=True)
        (session / 'session.yaml').write_text(
            'scan_topic: /scan_multi\nodom_topic: /odom_raw\n',
            encoding='utf-8')
        return session

    def test_counts_topics_across_multiple_db_shards(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = self._session(tmp)
            first = list(EXPECTED.items())[:2]
            second = list(EXPECTED.items())[2:]
            _write_shard(session / 'bag/record/record_0.db3',
                         [(n, t, 2) for n, t in first])
            _write_shard(session / 'bag/record/record_1.db3',
                         [(n, t, 3) for n, t in second])

            report = audit_capture_topics(session)

            self.assertTrue(report['complete'])
            self.assertEqual(len(report['bag_files']), 2)
            for name in EXPECTED:
                self.assertEqual(report['required_topics'][name]['message_count'],
                                 2 if name in dict(first) else 3)

    def test_zero_message_raw_scan_is_incomplete_and_bag_is_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = self._session(tmp)
            _write_shard(
                session / 'bag/record/record_0.db3',
                [(name, msg_type, 1 if name != '/scan1' else 0)
                 for name, msg_type in EXPECTED.items()])

            report = audit_capture_topics(session)

            self.assertFalse(report['complete'])
            self.assertEqual(report['missing_or_empty_topics'], ['/scan1'])
            self.assertTrue(report['raw_bag_preserved'])
            self.assertTrue((session / 'bag/record/record_0.db3').exists())


if __name__ == '__main__':
    unittest.main()
