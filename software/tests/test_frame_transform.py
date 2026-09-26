"""Planar frame transform contracts for sampled trajectory references."""

import math

import pytest

from m3pro_nav.frame_transform import RigidFrameTransform
from m3pro_nav.motion_planner import MotionPlanner
from m3pro_nav.pose import Pose2D
from m3pro_nav.trajectory_reference import ReferenceState, TrajectoryReference


def test_noncardinal_frame_transform_rotates_position_velocity_tangent_and_yaw():
    transform = RigidFrameTransform(
        Pose2D(1.0, -2.0, 0.3), Pose2D(-0.4, 0.8, 1.1))
    reference = ReferenceState(
        x=2.0, y=1.0, yaw_ref=0.3, vx_world=0.6, vy_world=0.8,
        tangent_x=0.6, tangent_y=0.8, curvature=0.2, progress_s=3.4)

    result = transform.transform_reference(reference)
    angle = transform.rotation
    expected_position = (
        -0.4 + math.cos(angle) - math.sin(angle) * 3.0,
        0.8 + math.sin(angle) + math.cos(angle) * 3.0)
    expected_vector = (
        math.cos(angle) * 0.6 - math.sin(angle) * 0.8,
        math.sin(angle) * 0.6 + math.cos(angle) * 0.8)

    assert (result.x, result.y) == pytest.approx(expected_position)
    assert result.yaw_ref == pytest.approx(1.1)
    assert (result.vx_world, result.vy_world) == pytest.approx(expected_vector)
    assert (result.tangent_x, result.tangent_y) == pytest.approx(expected_vector)
    assert math.hypot(result.vx_world, result.vy_world) == pytest.approx(1.0)
    assert result.curvature == reference.curvature
    assert result.progress_s == reference.progress_s


def test_explicit_anchor_transform_roundtrips_pose_and_reference():
    source = Pose2D(0.2, -0.7, -0.4)
    target = Pose2D(1.2, 2.3, 0.9)
    forward = RigidFrameTransform(source, target)
    inverse = RigidFrameTransform(target, source)
    pose = Pose2D(2.1, 0.5, 0.7)
    reference = ReferenceState(2.1, 0.5, 0.7, 0.4, -0.2, 0.8, -0.6,
                               0.1, 2.0)

    restored_pose = inverse.transform_pose(forward.transform_pose(pose))
    restored_reference = inverse.transform_reference(
        forward.transform_reference(reference))
    assert (restored_pose.x, restored_pose.y, restored_pose.yaw) == pytest.approx(
        (pose.x, pose.y, pose.yaw))
    assert tuple(restored_reference.__dict__.values()) == pytest.approx(
        tuple(reference.__dict__.values()))


def test_frame_transform_rejects_nonfinite_anchors():
    with pytest.raises(ValueError, match='finite'):
        RigidFrameTransform(Pose2D(0.0, math.nan, 0.0), Pose2D(0.0, 0.0, 0.0))


def test_frame_transform_copies_mutable_anchor_values():
    source = Pose2D(0.0, 0.0, 0.0)
    target = Pose2D(1.0, 2.0, 0.5)
    transform = RigidFrameTransform(source, target)
    source.x = target.x = 100.0
    result = transform.transform_pose(Pose2D(0.0, 0.0, 0.0))
    assert (result.x, result.y, result.yaw) == pytest.approx((1.0, 2.0, 0.5))


def test_fixed_quarter_arc_reference_transforms_without_changing_primitive_geometry():
    primitive, _, _ = MotionPlanner().template(
        (0, 0), (0, 1), (1, 1), Pose2D(0.2, 0.4, math.pi / 2))
    assert len(primitive) == 1 and primitive[0].kind == 'ARC'
    original_center = primitive[0].p0
    local = TrajectoryReference(primitive, yaw_ref=math.pi / 2).sample(
        primitive[0].length / 2, speed=0.1)
    transform = RigidFrameTransform(Pose2D(0.2, 0.4, math.pi / 2),
                                    Pose2D(1.0, 2.0, 0.7))
    odom = transform.transform_reference(local)
    assert (odom.vx_world, odom.vy_world) == pytest.approx(
        (0.1 * odom.tangent_x, 0.1 * odom.tangent_y))
    assert odom.yaw_ref == pytest.approx(0.7)
    assert primitive[0].p0 == original_center
