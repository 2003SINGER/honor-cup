"""Deterministic ideal and lower-loop M3Pro chassis plants.

The lower-loop constants follow the archived Subscriber_twist firmware. Motor
gain and time constant are assumptions because the archive contains no measured
PWM-to-wheel-speed transfer function.
"""

from dataclasses import dataclass
import math

from .pose import Pose2D, Twist2D, norm_angle


WHEEL_RADIUS_ARM_M = 0.167
WHEEL_CIRCUMFERENCE_M = 0.2513
ENCODER_PULSES_PER_REV = 2464
LOWER_PERIOD_S = 0.010
MAX_LINEAR_M_S = 0.7
MAX_ANGULAR_RAD_S = 4.2
MAX_WHEEL_MM_S = 750
MAX_PWM = 1000
MAX_MOTOR_PULSE = 2000
MOTOR_IGNORE_PULSE = 1000
PID_KP, PID_KI, PID_KD = 0.8, 0.06, 0.5
MOTOR_TAU_S_UNCALIBRATED_DEFAULT = 0.12
MOTOR_GAIN_UNCALIBRATED_DEFAULT = 1.0
PHYSICAL_DEADZONE_UNCALIBRATED_DEFAULT = 1000.0


@dataclass(frozen=True)
class PlantStep:
    pose: Pose2D
    twist: Twist2D
    wheel_target_mm_s: tuple[float, float, float, float]
    wheel_actual_mm_s: tuple[float, float, float, float]
    pwm: tuple[float, float, float, float]  # Applied signed driver pulse.
    encoder_delta: tuple[int, int, int, int]


def _wheel_speeds_mm_s(twist: Twist2D) -> tuple[float, float, float, float]:
    """Firmware wheel order M1..M4, matching Motion_Ctrl in app_motion.c."""
    vx, vy, wz = twist.vx * 1000.0, twist.vy * 1000.0, twist.wz
    spin = wz * WHEEL_RADIUS_ARM_M * 1000.0
    return (vx - vy - spin, vx + vy - spin,
            vx + vy + spin, vx - vy + spin)


def _body_twist(wheels_mm_s: tuple[float, float, float, float]) -> Twist2D:
    m1, m2, m3, m4 = (v / 1000.0 for v in wheels_mm_s)
    return Twist2D((m1 + m2 + m3 + m4) / 4.0,
                  (-m1 + m2 + m3 - m4) / 4.0,
                  (-m1 - m2 + m3 + m4) / (4.0 * WHEEL_RADIUS_ARM_M))


def _integrate_pose(pose: Pose2D, twist: Twist2D, dt: float) -> None:
    """Integrate constant body-frame twist exactly over dt."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    if abs(twist.wz) < 1e-12:
        dx_body, dy_body = twist.vx * dt, twist.vy * dt
    else:
        end_yaw = twist.wz * dt
        dx_body = (twist.vx * math.sin(end_yaw) +
                   twist.vy * (math.cos(end_yaw) - 1.0)) / twist.wz
        dy_body = (twist.vx * (1.0 - math.cos(end_yaw)) +
                   twist.vy * math.sin(end_yaw)) / twist.wz
    pose.x += c * dx_body - s * dy_body
    pose.y += s * dx_body + c * dy_body
    pose.yaw = norm_angle(pose.yaw + twist.wz * dt)


def _validate_dt(dt: float) -> None:
    if not math.isfinite(dt) or dt < 0:
        raise ValueError("dt must be a finite non-negative number")


class IdealPlant:
    """Perfect mecanum body-twist plant without actuator limits or lag."""

    def __init__(self, initial_pose: Pose2D | None = None):
        self.pose = (initial_pose or Pose2D(0.0, 0.0, 0.0)).copy()
        self.twist = Twist2D()

    def step(self, command: Twist2D, dt: float) -> PlantStep:
        _validate_dt(dt)
        self.twist = Twist2D(command.vx, command.vy, command.wz)
        _integrate_pose(self.pose, self.twist, dt)
        wheels = _wheel_speeds_mm_s(self.twist)
        return PlantStep(self.pose.copy(), Twist2D(self.twist.vx, self.twist.vy, self.twist.wz),
                         wheels, wheels, (0.0,) * 4, (0,) * 4)


class LowerLoopPlant:
    """10 ms lower controller with encoder quantization and assumed motor lag.

    ``motor_gain_mm_s_per_pwm`` and ``motor_tau_s`` are uncalibrated modeling
    assumptions, not values recovered from the firmware. Commands are held
    until the next firmware tick. A partial tick remains accumulated for the
    next call, matching a periodic 100 Hz controller.
    """

    def __init__(self, initial_pose: Pose2D | None = None, *,
                 motor_tau_s: float = MOTOR_TAU_S_UNCALIBRATED_DEFAULT,
                 motor_gain_mm_s_per_pwm: float = MOTOR_GAIN_UNCALIBRATED_DEFAULT,
                 physical_deadzone_pwm: float = PHYSICAL_DEADZONE_UNCALIBRATED_DEFAULT):
        if not math.isfinite(motor_tau_s) or motor_tau_s <= 0:
            raise ValueError("motor_tau_s must be finite and positive")
        if not math.isfinite(motor_gain_mm_s_per_pwm) or motor_gain_mm_s_per_pwm <= 0:
            raise ValueError("motor_gain_mm_s_per_pwm must be finite and positive")
        if not math.isfinite(physical_deadzone_pwm) or physical_deadzone_pwm < 0:
            raise ValueError("physical_deadzone_pwm must be finite and non-negative")
        self.pose = (initial_pose or Pose2D(0.0, 0.0, 0.0)).copy()
        self.twist = Twist2D()
        self.motor_tau_s = motor_tau_s
        self.motor_gain_mm_s_per_pwm = motor_gain_mm_s_per_pwm
        self.physical_deadzone_pwm = physical_deadzone_pwm
        self._time_remainder = 0.0
        self._target = (0.0,) * 4
        self._stopped = True
        self._actual = [0.0] * 4
        self._pwm = [0.0] * 4
        self._err = [0.0] * 4
        self._err_prev = [0.0] * 4
        self._err_prev2 = [0.0] * 4
        self._encoder_fraction = [0.0] * 4
        self._encoder_total = [0] * 4
        self._last_encoder_total = [0] * 4
        self._encoder_delta = (0, 0, 0, 0)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _set_target(self, command: Twist2D) -> None:
        bounded_mm = (
            int(self._clamp(command.vx, -MAX_LINEAR_M_S, MAX_LINEAR_M_S) * 1000.0),
            int(self._clamp(command.vy, -MAX_LINEAR_M_S, MAX_LINEAR_M_S) * 1000.0),
            int(self._clamp(command.wz, -MAX_ANGULAR_RAD_S, MAX_ANGULAR_RAD_S) * 1000.0))
        vx, vy, wz = bounded_mm
        spin = wz * WHEEL_RADIUS_ARM_M
        raw_wheels = (vx - vy - spin, vx + vy - spin,
                      vx + vy + spin, vx - vy + spin)
        self._target = tuple(int(self._clamp(v, -MAX_WHEEL_MM_S, MAX_WHEEL_MM_S))
                             for v in raw_wheels)
        self._stopped = bounded_mm == (0, 0, 0)
        if self._stopped:
            # Motion_Ctrl calls Motion_Stop for an all-zero integer command.
            self._pwm = [0.0] * 4
            self._err = [0.0] * 4
            self._err_prev = [0.0] * 4
            self._err_prev2 = [0.0] * 4

    def _tick(self) -> None:
        dt = LOWER_PERIOD_S
        encoder_delta = []
        for i, target in enumerate(self._target):
            actual_measured = (self._encoder_delta[i] * 100.0 *
                               WHEEL_CIRCUMFERENCE_M * 1000.0 /
                               ENCODER_PULSES_PER_REV)
            if not self._stopped:
                error = target - actual_measured
                self._pwm[i] = self._clamp(
                    self._pwm[i] + PID_KP * (error - self._err_prev[i]) +
                    PID_KI * error + PID_KD *
                    (error - 2.0 * self._err_prev[i] + self._err_prev2[i]),
                    -MAX_PWM, MAX_PWM)
                self._err_prev2[i], self._err_prev[i] = self._err_prev[i], error
                self._err[i] = error

            # app_motor.c adds MOTOR_IGNORE_PULSE to each nonzero signed PID
            # output, then clamps the bridge pulse to +/-MOTOR_MAX_PULSE.
            applied = (math.copysign(MOTOR_IGNORE_PULSE + abs(self._pwm[i]), self._pwm[i])
                       if self._pwm[i] else 0.0)
            applied = self._clamp(applied, -MAX_MOTOR_PULSE, MAX_MOTOR_PULSE)
            # First-order actuator response is a deliberately simple model.
            effective_drive = math.copysign(
                max(0.0, abs(applied) - self.physical_deadzone_pwm), applied)
            equilibrium = self._clamp(effective_drive * self.motor_gain_mm_s_per_pwm,
                                      -MAX_WHEEL_MM_S, MAX_WHEEL_MM_S)
            self._actual[i] += (equilibrium - self._actual[i]) * (1.0 - math.exp(-dt / self.motor_tau_s))
            pulse_increment = (self._actual[i] * dt / 1000.0 /
                               WHEEL_CIRCUMFERENCE_M * ENCODER_PULSES_PER_REV)
            self._encoder_fraction[i] += pulse_increment
            whole_pulses = math.trunc(self._encoder_fraction[i])
            self._encoder_fraction[i] -= whole_pulses
            self._encoder_total[i] += whole_pulses
            encoder_delta.append(self._encoder_total[i] - self._last_encoder_total[i])
            self._last_encoder_total[i] = self._encoder_total[i]

        self._encoder_delta = tuple(encoder_delta)
        self.twist = _body_twist(tuple(self._actual))
        _integrate_pose(self.pose, self.twist, dt)

    def step(self, command: Twist2D, dt: float) -> PlantStep:
        _validate_dt(dt)
        self._set_target(command)
        self._time_remainder += dt
        ticks = int((self._time_remainder + 1e-12) / LOWER_PERIOD_S)
        for _ in range(ticks):
            self._tick()
        self._time_remainder -= ticks * LOWER_PERIOD_S
        self._time_remainder = max(0.0, self._time_remainder)
        return PlantStep(self.pose.copy(), Twist2D(self.twist.vx, self.twist.vy, self.twist.wz),
                         tuple(self._target), tuple(self._actual),
                         tuple((math.copysign(MOTOR_IGNORE_PULSE + abs(v), v)
                                if v else 0.0) for v in self._pwm),
                         tuple(self._encoder_delta))
