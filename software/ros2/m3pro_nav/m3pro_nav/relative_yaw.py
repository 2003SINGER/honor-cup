"""Short-horizon yaw estimate from a calibrated IMU z-axis gyro.

This module deliberately does not consume ``sensor_msgs/Imu.orientation``:
the vehicle's raw IMU orientation field has not been validated.  The caller
must collect stationary gyro samples, anchor the estimate to a trusted yaw
(currently odometry), then feed samples in source-time order.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import statistics

from .pose import norm_angle


class ImuTimingError(ValueError):
    """The IMU source timestamp is out of order or has an excessive gap."""


@dataclass
class RelativeYawEstimator:
    """Integrate gyro-z into a short-term yaw estimate.

    Call :meth:`add_stationary_sample` for at least 4.5 seconds of stationary
    samples, then :meth:`start` with the initial odometry yaw.  ``update``
    integrates trapezoidally using IMU *source* timestamps. A source gap marks
    the estimate invalid; the caller must explicitly re-anchor it to a trusted
    yaw with :meth:`reanchor` before continuing.

    Defaults reflect the M3 Pro recordings inspected on 2026-10-01: 25 Hz IMU,
    40 ms median source period, <=112 ms observed period, stationary gyro-z
    standard deviation around 0.006 rad/s, and occasional dynamic samples up
    to about 1.9 rad/s.
    """

    stationary_sample_count: int = 125
    minimum_bias_duration_s: float = 4.5
    stationary_rate_limit_radps: float = 0.05
    maximum_bias_std_radps: float = 0.02
    maximum_sample_gap_s: float = 0.16
    maximum_abs_rate_radps: float = 3.0
    _bias_samples: list[float] = field(default_factory=list, init=False,
                                       repr=False)
    _calibration_start_stamp: float | None = field(default=None, init=False,
                                                   repr=False)
    _calibration_stamp: float | None = field(default=None, init=False,
                                              repr=False)
    _bias_radps: float | None = field(default=None, init=False, repr=False)
    _bias_std_radps: float | None = field(default=None, init=False, repr=False)
    _yaw_rad: float | None = field(default=None, init=False, repr=False)
    _last_stamp: float | None = field(default=None, init=False, repr=False)
    _last_rate_radps: float = field(default=0.0, init=False, repr=False)
    _valid: bool = field(default=False, init=False, repr=False)

    def __post_init__(self) -> None:
        params = (self.minimum_bias_duration_s,
                  self.stationary_rate_limit_radps,
                  self.maximum_bias_std_radps,
                  self.maximum_sample_gap_s,
                  self.maximum_abs_rate_radps)
        if self.stationary_sample_count < 2:
            raise ValueError('stationary_sample_count must be at least 2')
        if not all(math.isfinite(v) and v > 0.0 for v in params):
            raise ValueError('estimator limits must be finite and positive')

    @property
    def bias_radps(self) -> float | None:
        return self._bias_radps

    @property
    def bias_std_radps(self) -> float | None:
        return self._bias_std_radps

    @property
    def yaw_rad(self) -> float | None:
        return self._yaw_rad if self._valid else None

    @property
    def yaw_rate_radps(self) -> float | None:
        return self._last_rate_radps if self._valid else None

    @property
    def valid(self) -> bool:
        return self._valid

    @property
    def bias_ready(self) -> bool:
        return (len(self._bias_samples) >= self.stationary_sample_count
                and self.calibration_duration_s >= self.minimum_bias_duration_s)

    @property
    def calibration_duration_s(self) -> float:
        if self._calibration_start_stamp is None or self._calibration_stamp is None:
            return 0.0
        return self._calibration_stamp - self._calibration_start_stamp

    def add_stationary_sample(self, source_stamp_s: float,
                              gyro_z_radps: float) -> None:
        """Add one stationary calibration sample, rejecting motion or gaps."""
        if self._yaw_rad is not None:
            raise RuntimeError('bias calibration is closed after start')
        stamp, rate = self._validate_sample(source_stamp_s, gyro_z_radps)
        if abs(rate) > self.stationary_rate_limit_radps:
            self._bias_samples.clear()
            self._calibration_start_stamp = None
            self._calibration_stamp = stamp
            raise ValueError('gyro-z exceeds stationary calibration threshold; '
                             'calibration window was cleared')
        if self._calibration_stamp is not None:
            dt = stamp - self._calibration_stamp
            if dt <= 0.0:
                raise ImuTimingError('IMU source timestamp did not advance')
            if dt > self.maximum_sample_gap_s:
                self._bias_samples.clear()
                self._calibration_start_stamp = None
                self._calibration_stamp = stamp
                raise ImuTimingError('IMU source gap cleared the bias '
                                     'calibration window')
        self._bias_samples.append(rate)
        if self._calibration_start_stamp is None:
            self._calibration_start_stamp = stamp
        self._calibration_stamp = stamp

    def reset_bias_calibration(self) -> None:
        """Discard a failed stationary window before a fresh retry."""
        if self._yaw_rad is not None:
            raise RuntimeError('cannot reset bias calibration after start')
        self._bias_samples.clear()
        self._calibration_start_stamp = None
        self._calibration_stamp = None

    def start(self, initial_odom_yaw_rad: float,
              source_stamp_s: float | None = None) -> float:
        """Finish bias calibration and anchor relative yaw to odometry."""
        if self._yaw_rad is not None:
            raise RuntimeError('estimator has already started')
        if len(self._bias_samples) < self.stationary_sample_count:
            raise RuntimeError(
                f'need {self.stationary_sample_count} stationary samples; '
                f'have {len(self._bias_samples)}')
        if self.calibration_duration_s < self.minimum_bias_duration_s:
            raise RuntimeError(
                f'stationary calibration spans {self.calibration_duration_s:.3f} '
                f's; need at least {self.minimum_bias_duration_s:.3f} s')
        if not math.isfinite(initial_odom_yaw_rad):
            raise ValueError('initial odometry yaw must be finite')
        bias = statistics.fmean(self._bias_samples)
        bias_std = statistics.pstdev(self._bias_samples)
        if bias_std > self.maximum_bias_std_radps:
            raise ValueError('stationary gyro-z variation is too high for '
                             'bias calibration')
        stamp = (self._calibration_stamp if source_stamp_s is None
                 else self._finite(source_stamp_s, 'source timestamp'))
        if stamp is None or stamp < self._calibration_stamp:
            raise ImuTimingError('yaw anchor timestamp precedes calibration')
        if stamp - self._calibration_stamp > self.maximum_sample_gap_s:
            raise ImuTimingError('yaw anchor is separated from calibration by '
                                 'an excessive IMU source gap')
        self._bias_radps = bias
        self._bias_std_radps = bias_std
        self._yaw_rad = norm_angle(initial_odom_yaw_rad)
        self._last_stamp = stamp
        # Calibration ended while stationary; start integration from zero
        # corrected rate so the first sample cannot smear bias into motion.
        self._last_rate_radps = 0.0
        self._valid = True
        return self._yaw_rad

    def update(self, source_stamp_s: float, gyro_z_radps: float) -> float:
        """Integrate one fresh gyro sample and return normalized yaw."""
        if not self._valid or self._yaw_rad is None:
            raise RuntimeError('estimator is not valid; start or reanchor it')
        stamp, raw_rate = self._validate_sample(source_stamp_s, gyro_z_radps)
        assert self._last_stamp is not None and self._bias_radps is not None
        dt = stamp - self._last_stamp
        if dt <= 0.0:
            raise ImuTimingError('IMU source timestamp did not advance')
        if dt > self.maximum_sample_gap_s:
            self._valid = False
            raise ImuTimingError('IMU source gap invalidated relative yaw; '
                                 'reanchor from trusted odometry')
        rate = raw_rate - self._bias_radps
        self._yaw_rad = norm_angle(
            self._yaw_rad + 0.5 * (self._last_rate_radps + rate) * dt)
        self._last_rate_radps = rate
        self._last_stamp = stamp
        return self._yaw_rad

    def reanchor(self, odom_yaw_rad: float, source_stamp_s: float) -> float:
        """Restore validity after a gap using a fresh trusted odometry yaw."""
        if self._yaw_rad is None:
            raise RuntimeError('start the estimator before reanchoring')
        yaw = self._finite(odom_yaw_rad, 'odometry yaw')
        stamp = self._finite(source_stamp_s, 'source timestamp')
        if self._last_stamp is not None and stamp <= self._last_stamp:
            raise ImuTimingError('reanchor timestamp must advance')
        self._yaw_rad = norm_angle(yaw)
        self._last_stamp = stamp
        self._last_rate_radps = 0.0
        self._valid = True
        return self._yaw_rad

    def _validate_sample(self, source_stamp_s: float,
                         gyro_z_radps: float) -> tuple[float, float]:
        stamp = self._finite(source_stamp_s, 'source timestamp')
        rate = self._finite(gyro_z_radps, 'gyro-z')
        if abs(rate) > self.maximum_abs_rate_radps:
            raise ValueError('gyro-z exceeds configured physical rate limit')
        return stamp, rate

    @staticmethod
    def _finite(value: float, label: str) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f'{label} must be finite')
        return value
