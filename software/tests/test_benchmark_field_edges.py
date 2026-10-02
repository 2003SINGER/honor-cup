import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / 'software' / 'tools'))
import benchmark_field_edges as benchmark
from m3pro_nav.edge_map import UNKNOWN


class BenchmarkFieldEdgesTest(unittest.TestCase):
    def test_edge_level_confusion_counts_unknown_separately(self):
        truth = {'wall': True, 'open': False, 'unseen': True}
        states = {'wall': 'OPEN', 'open': 'WALL'}
        self.assertEqual(benchmark.score(states, truth), {
            'truth_wall_written_open': 1,
            'truth_open_written_wall': 1,
            'unknown': 1,
            'edges': 3,
        })

    def test_unknown_state_constant_matches_score_default(self):
        self.assertEqual(UNKNOWN, 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
