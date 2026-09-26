import math

from m3pro_nav.motion_primitive import ARC_RADIUS, MotionPrimitive
from m3pro_nav.pose import Pose2D
from m3pro_nav.position_controller import PositionController
from m3pro_nav.trajectory_reference import TrajectoryReference


def straight(p0, p1):
    length = math.dist(p0, p1)
    return MotionPrimitive('STRAIGHT', Pose2D(*p0, 0.0), p0=p0, p1=p1,
                           yaw0=0.0, length=length)


def quarter_arc(center=(0.0, 0.0), angle=0.0, sweep=math.pi / 2):
    return MotionPrimitive('ARC', Pose2D(0.0, 0.0, 0.0), p0=center,
                           yaw0=angle, yaw1=sweep,
                           length=ARC_RADIUS * abs(sweep),
                           meta={'r': ARC_RADIUS})


def test_reference_samples_straight_and_world_feedforward():
    trajectory = TrajectoryReference([straight((0, 0), (1, 0))], yaw_ref=math.pi / 2)
    ref = trajectory.sample(0.25, speed=0.4)
    assert (ref.x, ref.y) == (0.25, 0.0)
    assert (ref.tangent_x, ref.tangent_y) == (1.0, 0.0)
    assert (ref.vx_world, ref.vy_world) == (0.4, 0.0)
    assert ref.yaw_ref == math.pi / 2
    assert ref.curvature == 0.0


def test_reference_arc_tangent_does_not_change_body_yaw():
    trajectory = TrajectoryReference([quarter_arc()], yaw_ref=0.0)
    ref = trajectory.sample(ARC_RADIUS * math.pi / 4, speed=0.2)
    assert math.isclose(ref.x, ARC_RADIUS / math.sqrt(2))
    assert math.isclose(ref.y, ARC_RADIUS / math.sqrt(2))
    assert math.isclose(ref.tangent_x, -1 / math.sqrt(2))
    assert math.isclose(ref.tangent_y, 1 / math.sqrt(2))
    assert math.isclose(ref.curvature, 1 / ARC_RADIUS)
    assert ref.yaw_ref == 0.0


def test_controller_rotates_world_command_into_body_without_yaw_from_tangent():
    ref = TrajectoryReference([quarter_arc()], yaw_ref=0.0).sample(
        ARC_RADIUS * math.pi / 4, speed=0.2)
    controller = PositionController(kp_pos=0.0, kp_yaw=2.0)
    pose = Pose2D(ref.x, ref.y, 0.0)
    cmd = controller.update(ref, pose)
    assert math.isclose(cmd.vx, ref.vx_world)
    assert math.isclose(cmd.vy, ref.vy_world)
    assert cmd.wz == 0.0


def test_controller_position_feedback_and_wrapped_yaw_error():
    ref = TrajectoryReference([straight((1, 0), (2, 0))], yaw_ref=-math.pi + 0.1).sample(0)
    controller = PositionController(kp_pos=2.0, kp_yaw=3.0)
    cmd = controller.update(ref, Pose2D(0.5, 0.0, math.pi - 0.1))
    assert math.isclose(cmd.vx, -math.cos(0.1))
    assert math.isclose(cmd.vy, -math.sin(0.1))
    assert math.isclose(cmd.wz, 0.6)


def test_optional_velocity_damping():
    ref = TrajectoryReference([straight((0, 0), (1, 0))], yaw_ref=0.0).sample(0, speed=1.0)
    controller = PositionController(kp_pos=0.0, kd_vel=0.5)
    cmd = controller.update(ref, Pose2D(0, 0, 0), measured_velocity_world=(0.4, 0.0))
    assert math.isclose(cmd.vx, 1.3)
    assert cmd.vy == 0.0
    # Without a velocity observation, omit the D term rather than inventing
    # zero actual speed and boosting feedforward.
    no_velocity = controller.update(ref, Pose2D(0, 0, 0))
    assert math.isclose(no_velocity.vx, 1.0)
