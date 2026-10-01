"""ROS-free tests for gyro-z relative yaw using measured M3 Pro bag values."""

import math

import pytest

from m3pro_nav.relative_yaw import ImuTimingError, RelativeYawEstimator


# First 16 raw gyro-z samples in the 2026-10-01 measured_stop_loop bag.
# Source intervals are 24–66 ms, consistent with the recorded 25 Hz stream
# and observed sub-112 ms maximum gap. These are stationary startup samples.
RECORDED_STATIONARY = (
    (1790847024.458, 0.004227768164128065),
    (1790847024.498, -0.0032290827948600054),
    (1790847024.564, -0.0010985539993271232),
    (1790847024.594, -0.0010985539993271232),
    (1790847024.618, 0.0031625039409846067),
    (1790847024.658, -0.00003328951424919069),
    (1790847024.720, -0.0010985539993271232),
    (1790847024.746, -0.01281646266579628),
    (1790847024.778, 0.0031625039409846067),
    (1790847024.844, -0.00003328951424919069),
    (1790847024.874, 0.004227768164128065),
    (1790847024.898, -0.007490140851587057),
    (1790847024.938, 0.0031625039409846067),
    (1790847025.002, 0.0020972394850105047),
    (1790847025.022, 0.011684619821608067),
    (1790847025.058, -0.0032290827948600054),
)


def calibrated(*, max_gap=0.16):
    estimator = RelativeYawEstimator(stationary_sample_count=16,
                                     minimum_bias_duration_s=0.5,
                                     maximum_sample_gap_s=max_gap)
    for stamp, gyro_z in RECORDED_STATIONARY:
        estimator.add_stationary_sample(stamp, gyro_z)
    return estimator


def test_calibrates_bias_from_recorded_stationary_bag_samples():
    estimator = calibrated()

    expected_bias = sum(rate for _, rate in RECORDED_STATIONARY) / 16
    assert estimator.bias_ready
    assert estimator.start(0.0) == 0.0
    assert estimator.bias_radps == pytest.approx(expected_bias)
    assert estimator.bias_std_radps == pytest.approx(0.0054, abs=0.001)


def test_default_calibration_window_matches_five_seconds_at_25_hz():
    estimator = RelativeYawEstimator()
    assert estimator.stationary_sample_count == 125
    assert estimator.minimum_bias_duration_s == 4.5


def test_dense_samples_do_not_replace_minimum_stationary_duration():
    estimator = RelativeYawEstimator()
    start = 100.0
    for i in range(125):
        estimator.add_stationary_sample(start + i * 0.001, 0.001)

    assert estimator.calibration_duration_s == pytest.approx(0.124)
    assert not estimator.bias_ready
    with pytest.raises(RuntimeError, match='need at least 4.500 s'):
        estimator.start(0.0)


def test_relative_yaw_integrates_bias_corrected_rate_and_wraps_at_pi():
    estimator = calibrated()
    anchor = math.pi - 0.02
    estimator.start(anchor)
    bias = estimator.bias_radps
    t = RECORDED_STATIONARY[-1][0]

    first = estimator.update(t + 0.04, bias + 0.4)
    second = estimator.update(t + 0.08, bias + 0.4)

    assert first == pytest.approx(math.pi - 0.012)
    assert second == pytest.approx(-math.pi + 0.004)
    assert -math.pi <= second <= math.pi


def test_observed_112ms_source_interval_is_accepted():
    estimator = calibrated()
    estimator.start(0.3)
    stamp = RECORDED_STATIONARY[-1][0]
    estimator.update(stamp + 0.112, estimator.bias_radps + 0.2)
    assert estimator.valid
    assert estimator.yaw_rad == pytest.approx(0.3112)


def test_start_rejects_anchor_after_unobserved_motion_gap():
    estimator = calibrated()
    last_calibration_stamp = RECORDED_STATIONARY[-1][0]

    with pytest.raises(ImuTimingError, match='excessive IMU source gap'):
        estimator.start(0.0, last_calibration_stamp + 0.17)

    assert estimator.bias_radps is None
    assert not estimator.valid


def test_recorded_dynamic_gyro_peak_is_integrated_as_rate_not_orientation():
    estimator = calibrated()
    estimator.start(0.0)
    stamp = RECORDED_STATIONARY[-1][0]

    # 1.760982 rad/s was present in the measured-stop-loop bag. It is a gyro
    # sample, not an orientation quaternion, and remains below the physical
    # input ceiling. One 40 ms interval contributes about 2 degrees.
    yaw = estimator.update(stamp + 0.04, 1.760982)

    assert estimator.valid
    assert yaw == pytest.approx(0.0352, abs=0.0001)


def test_motion_during_bias_calibration_clears_the_window():
    estimator = calibrated()
    with pytest.raises(ValueError, match='stationary calibration threshold'):
        estimator.add_stationary_sample(1790847025.10, 1.760982)
    assert not estimator.bias_ready
    with pytest.raises(RuntimeError, match='need 16 stationary samples'):
        estimator.start(0.0)


def test_nonmonotonic_sample_is_rejected_without_advancing_estimate():
    estimator = calibrated()
    estimator.start(0.0)
    stamp = RECORDED_STATIONARY[-1][0]
    estimator.update(stamp + 0.04, estimator.bias_radps + 0.2)
    yaw_before = estimator.yaw_rad

    with pytest.raises(ImuTimingError, match='did not advance'):
        estimator.update(stamp + 0.03, estimator.bias_radps + 0.2)

    assert estimator.valid
    assert estimator.yaw_rad == yaw_before
    assert estimator.update(stamp + 0.08,
                            estimator.bias_radps + 0.2) > yaw_before


def test_large_source_gap_invalidates_until_explicit_odom_reanchor():
    estimator = calibrated()
    estimator.start(0.4)
    stamp = RECORDED_STATIONARY[-1][0]

    with pytest.raises(ImuTimingError, match='invalidated relative yaw'):
        estimator.update(stamp + 0.20, estimator.bias_radps)

    assert not estimator.valid
    assert estimator.yaw_rad is None
    with pytest.raises(RuntimeError, match='not valid'):
        estimator.update(stamp + 0.24, estimator.bias_radps)

    anchored = estimator.reanchor(-math.pi - 0.1, stamp + 0.24)
    assert anchored == pytest.approx(math.pi - 0.1)
    assert estimator.valid
    assert estimator.update(stamp + 0.28,
                            estimator.bias_radps + 0.2) > anchored


def test_rejects_nonfinite_input_and_implausible_rate():
    estimator = calibrated()
    estimator.start(0.0)
    stamp = RECORDED_STATIONARY[-1][0]
    with pytest.raises(ValueError, match='must be finite'):
        estimator.update(stamp + 0.04, math.nan)
    with pytest.raises(ValueError, match='physical rate limit'):
        estimator.update(stamp + 0.04, 3.01)
    assert estimator.valid
