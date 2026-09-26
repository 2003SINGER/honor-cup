"""Multi-rate fixed-trajectory follower shared by deterministic plant gates.

The upper position loop runs at its configured period; the plant owns its
lower-loop period.  Navigation and map state never enter this control layer.
"""

from dataclasses import dataclass
import math

from .pose import Twist2D
from .position_controller import PositionController
from .speed_profile import SpeedProfile
from .trajectory_reference import TrajectoryReference


@dataclass(frozen=True)
class ControlSnapshot:
    time: float
    reference: object
    command: Twist2D
    plant_step: object


class ControlChain:
    def __init__(self, primitives, plant, *, yaw_ref, upper_period=0.02,
                 start_speed=0.0, a_acc=1.0, a_dec=1.0, controller=None):
        if not math.isfinite(upper_period) or upper_period <= 0:
            raise ValueError("upper_period must be positive and finite")
        self.profile = SpeedProfile(primitives, start_speed=start_speed,
                                    a_acc=a_acc, a_dec=a_dec)
        self.reference = TrajectoryReference(primitives, yaw_ref=yaw_ref)
        self.controller = controller or PositionController()
        self.plant = plant
        self.upper_period = upper_period
        self.time = 0.0
        self._until_upper = 0.0
        self.command = Twist2D()
        self.last_step = None

    def target(self):
        speed = self.profile.sample(self.time)
        return self.reference.sample(speed.progress_s, speed.speed,
                                     speed.acceleration)

    def _update_command(self):
        ref = self.target()
        body = getattr(self.plant, 'twist', Twist2D())
        pose = self.plant.pose
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        measured_world = (c * body.vx - s * body.vy,
                          s * body.vx + c * body.vy)
        self.command = self.controller.update(
            ref, pose, measured_velocity_world=measured_world,
            yaw_rate=body.wz)
        self._until_upper = self.upper_period

    def step(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("step dt must be positive and finite")
        remaining = dt
        while remaining > 1e-12:
            if self._until_upper <= 1e-12:
                self._update_command()
            chunk = min(remaining, self._until_upper)
            self.last_step = self.plant.step(self.command, chunk)
            self.time += chunk
            self._until_upper -= chunk
            remaining -= chunk
        return ControlSnapshot(self.time, self.target(), self.command,
                               self.last_step)
