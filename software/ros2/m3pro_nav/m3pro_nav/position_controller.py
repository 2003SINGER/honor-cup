"""Outer pose/heading feedback controller producing body-frame velocity."""

from dataclasses import dataclass
import math

from .pose import Pose2D, Twist2D, norm_angle
from .trajectory_reference import ReferenceState


@dataclass
class PositionController:
    kp_pos: float = 1.0
    kd_vel: float = 0.0
    kp_yaw: float = 2.0
    kd_yaw: float = 0.0

    def update(self, reference: ReferenceState, measured_pose: Pose2D,
               measured_velocity_world=None, yaw_rate: float = 0.0) -> Twist2D:
        """Compute body-frame vx/vy/wz; gains require later calibration.

        No integral state is kept. ``measured_velocity_world`` may be a pair
        ``(vx, vy)``; when omitted, the optional velocity damping term is zero.
        """
        if measured_velocity_world is None:
            mvx, mvy = reference.vx_world, reference.vy_world
        else:
            mvx, mvy = measured_velocity_world
        vals = (reference.x, reference.y, reference.yaw_ref,
                reference.vx_world, reference.vy_world, measured_pose.x,
                measured_pose.y, measured_pose.yaw, mvx, mvy, yaw_rate,
                self.kp_pos, self.kd_vel, self.kp_yaw, self.kd_yaw)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('controller inputs and gains must be finite')

        ex, ey = reference.x - measured_pose.x, reference.y - measured_pose.y
        vx_world = (reference.vx_world + self.kp_pos * ex
                    + self.kd_vel * (reference.vx_world - mvx))
        vy_world = (reference.vy_world + self.kp_pos * ey
                    + self.kd_vel * (reference.vy_world - mvy))
        yaw_error = norm_angle(reference.yaw_ref - measured_pose.yaw)
        wz = self.kp_yaw * yaw_error - self.kd_yaw * yaw_rate

        c, s = math.cos(measured_pose.yaw), math.sin(measured_pose.yaw)
        return Twist2D(vx=c * vx_world + s * vy_world,
                       vy=-s * vx_world + c * vy_world,
                       wz=wz)
