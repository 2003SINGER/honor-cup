"""Pure planar rigid transforms between explicitly anchored frames."""

from dataclasses import dataclass, replace
import math

from .pose import Pose2D, norm_angle


@dataclass(frozen=True)
class RigidFrameTransform:
    """Transform coordinates and vectors from ``source`` into ``target``.

    Each anchor describes the same physical point and frame heading in its own
    coordinates: ``source`` is its pose in the source frame, and ``target`` is
    its pose in the target frame. Reference headings are transformed as
    angles; velocity and tangent are rotated without translating them.
    """

    source: Pose2D
    target: Pose2D

    def __post_init__(self):
        for name, pose in (('source', self.source), ('target', self.target)):
            if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
                raise ValueError(f'{name} anchor must be finite')
        object.__setattr__(self, 'source', self.source.copy())
        object.__setattr__(self, 'target', self.target.copy())

    @property
    def rotation(self) -> float:
        """Yaw rotation from the source frame axes to target frame axes."""
        return norm_angle(self.target.yaw - self.source.yaw)

    def transform_pose(self, pose: Pose2D) -> Pose2D:
        """Map a pose expressed in source coordinates into target coordinates."""
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        dx, dy = pose.x - self.source.x, pose.y - self.source.y
        return Pose2D(self.target.x + c * dx - s * dy,
                      self.target.y + s * dx + c * dy,
                      norm_angle(pose.yaw + self.rotation))

    def transform_reference(self, reference):
        """Map a sampled ReferenceState into the target frame."""
        c, s = math.cos(self.rotation), math.sin(self.rotation)

        def rotate(x, y):
            return c * x - s * y, s * x + c * y

        point = self.transform_pose(Pose2D(reference.x, reference.y,
                                           reference.yaw_ref))
        vx, vy = rotate(reference.vx_world, reference.vy_world)
        tx, ty = rotate(reference.tangent_x, reference.tangent_y)
        return replace(reference, x=point.x, y=point.y,
                       yaw_ref=point.yaw, vx_world=vx, vy_world=vy,
                       tangent_x=tx, tangent_y=ty)
