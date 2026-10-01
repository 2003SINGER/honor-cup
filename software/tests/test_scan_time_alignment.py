import math
from types import SimpleNamespace as NS

import pytest

from m3pro_nav.pose import Pose2D
from m3pro_nav.scan_adapter import parse_scan
from m3pro_nav.scan_time_alignment import OdomPoseHistory, ScanTimeAligner


def scan(*, stamp=1.0, increment=0.0, count=1, ranges=None):
    msg = NS(
        header=NS(frame_id='laser', stamp=NS(sec=int(stamp),
                                              nanosec=int((stamp % 1) * 1e9))),
        angle_min=0.0, angle_increment=0.1, range_min=0.05, range_max=10.0,
        time_increment=increment, scan_time=0.1,
        ranges=[1.0] * count if ranges is None else ranges)
    return parse_scan(msg)


def test_history_interpolates_translation_and_shortest_yaw_across_wrap():
    history = OdomPoseHistory()
    history.add(1.0, Pose2D(0.0, 2.0, math.radians(179)))
    history.add(1.1, Pose2D(1.0, 4.0, math.radians(-179)))

    pose, skew = history.interpolate(1.05)

    assert pose.x == pytest.approx(0.5)
    assert pose.y == pytest.approx(3.0)
    assert abs(abs(pose.yaw) - math.pi) < 1e-8
    assert skew == pytest.approx(0.05)


def test_history_requires_finite_strictly_increasing_source_stamps():
    history = OdomPoseHistory()
    with pytest.raises(ValueError, match='finite'):
        history.add(math.nan, Pose2D(0, 0, 0))
    history.add(1.0, Pose2D(0, 0, 0))
    with pytest.raises(ValueError, match='strictly advance'):
        history.add(1.0, Pose2D(0, 0, 0))


def test_history_is_bounded_by_age_and_sample_count():
    history = OdomPoseHistory(history_s=1.0, max_samples=3)
    for i in range(5):
        history.add(float(i), Pose2D(i, 0, 0))
    assert len(history) == 3
    assert [s.stamp for s in history.samples] == [2.0, 3.0, 4.0]
    assert history.interpolate(1.5) is None


def test_history_rejects_large_bracket_gap_and_bounds_extrapolation():
    history = OdomPoseHistory()
    history.add(1.0, Pose2D(0, 0, 0))
    history.add(1.2, Pose2D(0.2, 0, 0))
    assert history.interpolate(1.1, max_bracket_gap_s=0.15) is None
    assert history.interpolate(1.22, max_bracket_gap_s=0.25,
                               max_extrapolation_s=0.03)[0].x == pytest.approx(0.22)
    assert history.interpolate(1.24, max_bracket_gap_s=0.25,
                               max_extrapolation_s=0.03) is None


def test_scan_alignment_uses_midpoint_pose_and_source_time():
    history = OdomPoseHistory()
    for t, x in ((1.0, 0.0), (1.05, 0.001), (1.1, 0.002)):
        history.add(t, Pose2D(x, 0, 0), received_stamp=t + 0.01)
    result = ScanTimeAligner(history).align(scan(stamp=1.0, increment=0.01,
                                                count=11))
    assert result.accepted
    assert result.reason is None
    assert result.pose.x == pytest.approx(0.001)
    assert result.scan_end == pytest.approx(1.1)


def test_scan_alignment_abstains_when_scan_not_bracketed_or_odom_skewed():
    history = OdomPoseHistory()
    history.add(1.0, Pose2D(0, 0, 0))
    history.add(1.2, Pose2D(0, 0, 0))
    result = ScanTimeAligner(history, max_bracket_gap_s=0.15,
                             max_extrapolation_s=0.0).align(
        scan(stamp=1.0, increment=0.01, count=11))
    assert not result.accepted
    assert result.reason == 'ODOM_TIME_UNBRACKETED_OR_SKEWED'


def test_scan_alignment_abstains_for_motion_distortion_without_deskew():
    history = OdomPoseHistory()
    history.add(1.0, Pose2D(0, 0, math.radians(179)))
    history.add(1.05, Pose2D(0.01, 0, math.radians(-179)))
    result = ScanTimeAligner(history).align(
        scan(stamp=1.0, increment=0.005, count=11))
    assert not result.accepted
    assert result.reason == 'MOTION_DISTORTION_REQUIRES_DESKEW'
    assert result.pose is None


def test_scan_alignment_abstains_when_multibeam_duration_is_unknown():
    history = OdomPoseHistory()
    history.add(1.0, Pose2D(0, 0, 0))
    history.add(1.1, Pose2D(0, 0, 0))
    result = ScanTimeAligner(history).align(scan(stamp=1.0, count=11))
    assert not result.accepted
    assert result.reason == 'STATIC_WINDOW_NOT_COVERED'
    assert result.mode == 'STATIC_WINDOW_FALLBACK'


def test_unknown_duration_scan_uses_pose_at_stamp_only_with_full_static_window():
    history = OdomPoseHistory()
    for i in range(11):
        t = 0.75 + i * 0.05
        history.add(t, Pose2D(0.1, -0.2, 0.3))

    result = ScanTimeAligner(history).align(scan(stamp=1.0, count=11))

    assert result.accepted
    assert result.reason is None
    assert result.mode == 'STATIC_WINDOW_FALLBACK'
    assert result.pose.x == pytest.approx(0.1)
    assert result.pose.y == pytest.approx(-0.2)
    assert result.pose.yaw == pytest.approx(0.3)
    assert result.scan_start == result.scan_end == pytest.approx(1.0)
    assert result.motion_window_start == pytest.approx(0.75)
    assert result.motion_window_end == pytest.approx(1.25)


def test_unknown_duration_scan_rejects_motion_inside_stationary_window():
    history = OdomPoseHistory()
    for i in range(11):
        t = 0.75 + i * 0.05
        x = (t - 0.75) * 0.1 if t <= 1.0 else (1.25 - t) * 0.1
        history.add(t, Pose2D(x, 0, 0))

    result = ScanTimeAligner(history).align(scan(stamp=1.0, count=11))

    assert not result.accepted
    assert result.pose is None
    assert result.reason == 'STATIC_WINDOW_MOTION'
    assert result.mode == 'STATIC_WINDOW_FALLBACK'


def test_unknown_duration_scan_rejects_yaw_motion_inside_stationary_window():
    history = OdomPoseHistory()
    for i in range(11):
        t = 0.75 + i * 0.05
        yaw = (t - 0.75) * 0.1 if t <= 1.0 else (1.25 - t) * 0.1
        history.add(t, Pose2D(0, 0, yaw))

    result = ScanTimeAligner(history).align(scan(stamp=1.0, count=11))

    assert not result.accepted
    assert result.reason == 'STATIC_WINDOW_MOTION'
    assert result.mode == 'STATIC_WINDOW_FALLBACK'


def test_parse_scan_keeps_positive_infinity_as_invalid_without_verified_sensor_semantics():
    frame = scan(ranges=[math.inf])
    # The standalone parser's established contract remains conservative:
    # nonfinite values do not become hit or free-space evidence.
    assert not frame.rays[0].valid
    assert frame.rays[0].invalid_reason == 'INVALID_RANGE'
