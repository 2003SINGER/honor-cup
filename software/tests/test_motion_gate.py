"""Small, deterministic motion cases; no random maze or task simulation."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

from motion_gate import cases, run_case
from m3pro_nav.plant import MAX_WHEEL_MM_S, WHEEL_CIRCUMFERENCE_M


def test_five_motion_cases_with_source_based_lower_loop():
    expected = {'straight', 'lateral', 'straight-left-straight',
                'straight-right-straight',
                'straight-left-straight-right-reverse'}
    selected = cases()
    assert {case.name for case in selected} == expected
    rpm_limit = MAX_WHEEL_MM_S / 1000.0 / WHEEL_CIRCUMFERENCE_M * 60.0
    for case in selected:
        metrics = run_case(case, backend='lower')
        assert metrics.collisions == 0, metrics
        assert metrics.min_wall_clearance_m > 0.005, metrics
        assert metrics.max_wheel_rpm <= rpm_limit + 1e-9, metrics
        assert metrics.max_yaw_error_rad < 0.05, metrics
        assert metrics.max_cross_track_error_m < 0.06, metrics
        assert metrics.final_position_error_m < 0.04, metrics
        assert all(math.isfinite(v) for v in vars(metrics).values()
                   if isinstance(v, (int, float))), metrics


def test_ideal_backend_preserves_motion_geometry():
    for case in cases():
        metrics = run_case(case, backend='ideal')
        assert metrics.collisions == 0, metrics
        assert metrics.final_position_error_m < 0.005, metrics
