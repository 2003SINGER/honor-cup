import math

import pytest

from m3pro_nav.feedback_trajectory_follower import FeedbackTrajectoryFollower
from m3pro_nav.motion_planner import MotionPlanner
from m3pro_nav.motion_primitive import MotionPrimitive
from m3pro_nav.odometry_adapter import OdometryState
from m3pro_nav.pose import Pose2D
from m3pro_nav.position_controller import PositionController


def compiled_chain():
    planner = MotionPlanner()
    pose = Pose2D(0.2, 0.2, 0.37)
    first, point, cursor = planner.template(None, (0, 0), (0, 1), pose)
    arc, point, cursor = planner.template((0, 0), (0, 1), (1, 1),
                                          Pose2D(*point, pose.yaw))
    last, point, cursor = planner.template((0, 1), (1, 1), (1, 2),
                                           Pose2D(*point, pose.yaw))
    chain = first + arc + last
    chain[-1].v_end = 0.0
    chain.append(MotionPrimitive('STOP', Pose2D(*point, pose.yaw),
                                 p0=point, yaw0=pose.yaw, duration=0.2))
    return chain, pose


def odom(pose, *, vx=0.0, vy=0.0, wz=0.0, stamp=1.0):
    return OdometryState(pose, vx, vy, wz, stamp, 'odom', 'base_link')


def follower(chain, planner_start, odom_start):
    return FeedbackTrajectoryFollower(
        chain, planner_start=planner_start, odom_start=odom_start,
        a_acc=1.0, a_dec=1.0, controller=PositionController(),
        position_tolerance=0.01,
        yaw_tolerance=0.02, velocity_tolerance=0.01,
        yaw_rate_tolerance=0.02, settle_time=0.1,
        expected_odom_frame='odom', expected_base_frame='base_link')


def test_multisegment_following_transforms_reference_and_keeps_yaw_held():
    chain, planner_start = compiled_chain()
    odom_start = Pose2D(3.0, -2.0, 1.13)
    follow = follower(chain, planner_start, odom_start)
    arc_start = chain[0].length
    arc_end = arc_start + chain[1].length
    arc_time = next(t for i in range(1, 1000)
                    for t in (follow.profile.duration * i / 1000,)
                    if arc_start < follow.profile.sample(t).progress_s < arc_end)
    speed = follow.profile.sample(arc_time)
    local_ref = follow.reference.sample(speed.progress_s, speed.speed,
                                        speed.acceleration)
    transformed = follow.transform.transform_reference(local_ref)
    result = follow.update(arc_time,
                          odom(transformed_pose(transformed), vx=transformed.vx_world,
                               vy=transformed.vy_world))
    assert (result.reference.x, result.reference.y) == pytest.approx(
        (transformed.x, transformed.y))
    assert result.reference.yaw_ref == pytest.approx(odom_start.yaw)
    assert result.yaw_error == pytest.approx(0.0)
    assert result.reference.curvature != 0.0
    assert math.hypot(result.command.vx, result.command.vy) == pytest.approx(speed.speed)
    assert result.reference.curvature == local_ref.curvature
    assert result.reference.progress_s == pytest.approx(local_ref.progress_s)


def transformed_pose(ref):
    return Pose2D(ref.x, ref.y, ref.yaw_ref)


def test_completion_requires_terminal_stop_and_measured_settling():
    chain, source = compiled_chain()
    target = Pose2D(2.0, 4.0, 1.13)
    follow = follower(chain, source, target)
    end = follow.transform.transform_pose(chain[-1].start_pose.copy())
    before = follow.update(follow.profile.duration,
                           odom(Pose2D(end.x + 0.2, end.y, end.yaw)))
    assert before.schedule_complete and not before.complete
    at_target = follow.update(follow.profile.duration + 0.05, odom(end, stamp=2.0))
    assert not at_target.complete
    done = follow.update(follow.profile.duration + 0.16, odom(end, stamp=3.0))
    assert done.complete
    assert done.command.vx == done.command.vy == done.command.wz == 0.0


def test_invalid_feedback_time_and_unstopped_chain_are_rejected():
    chain, source = compiled_chain()
    follow = follower(chain, source, source)
    with pytest.raises(ValueError, match='feedback'):
        follow.update(0.0, None)
    with pytest.raises(ValueError, match='finite'):
        follow.update(math.nan, odom(source))
    follow.update(0.0, odom(source))
    with pytest.raises(ValueError, match='nonnegative'):
        follow.update(-0.1, odom(source))
    follow.update(0.2, odom(source))
    with pytest.raises(ValueError, match='monotonic'):
        follow.update(0.1, odom(source))
    with pytest.raises(ValueError, match='STOP'):
        FeedbackTrajectoryFollower(chain[:-1], planner_start=source,
                                   odom_start=source, a_acc=1.0, a_dec=1.0,
                                   controller=PositionController(),
                                   position_tolerance=0.01, yaw_tolerance=0.02,
                                   velocity_tolerance=0.01,
                                   yaw_rate_tolerance=0.02, settle_time=0.1)


def test_chain_is_snapshotted_and_bad_planner_start_is_rejected():
    chain, source = compiled_chain()
    follow = follower(chain, source, source)
    original_vmax = follow.primitives[0].v_max
    chain[0].v_max = original_vmax * 0.5
    assert follow.primitives[0].v_max == original_vmax
    with pytest.raises(ValueError, match='planner_start'):
        follower(follow.primitives, Pose2D(source.x + 0.01, source.y, source.yaw),
                 source)


def test_stop_only_chain_waits_with_zero_command_and_measured_settling():
    source = Pose2D(0.2, 0.2, 0.37)
    stop = MotionPrimitive('STOP', source.copy(), p0=(source.x, source.y),
                           yaw0=source.yaw, duration=0.2)
    target = Pose2D(3.0, -2.0, 1.13)
    follow = follower([stop], source, target)
    early = follow.update(0.1, odom(target))
    assert not early.schedule_complete and not early.complete
    assert early.command.vx == early.command.vy == early.command.wz == 0.0
    scheduled = follow.update(0.2, odom(target, stamp=2.0))
    assert scheduled.schedule_complete and not scheduled.complete
    done = follow.update(0.31, odom(target, stamp=3.0))
    assert done.complete
    assert done.command.vx == done.command.vy == done.command.wz == 0.0


def test_follower_phases_track_hold_finish():
    """GPT 交接单: follower 必须显式区分 TRACKING / HOLDING / FINISHED."""
    from m3pro_nav.feedback_trajectory_follower import FollowerPhase

    chain, source = compiled_chain()
    target = Pose2D(2.0, 4.0, 1.13)
    follow = follower(chain, source, target)
    end = follow.transform.transform_pose(chain[-1].start_pose.copy())
    # 轨迹中段: TRACKING
    mid = follow.update(follow.profile.duration * 0.5,
                        odom(Pose2D(source.x, source.y, source.yaw)))
    assert mid.phase == FollowerPhase.TRACKING
    # 计划耗尽但未 settle: HOLDING (位置环保持终端参考, 指令不放弃)
    holding = follow.update(follow.profile.duration + 0.05,
                            odom(end, stamp=2.0))
    assert holding.phase == FollowerPhase.HOLDING
    assert holding.schedule_complete and not holding.complete
    # settle 后: FINISHED 且零指令
    done = follow.update(follow.profile.duration + 0.16, odom(end, stamp=3.0))
    assert done.phase == FollowerPhase.FINISHED
    assert done.command.vx == done.command.vy == done.command.wz == 0.0


def test_stop_only_chain_is_holding_never_tracking():
    """STOP-only 是 HOLDING, 不是 TRACKING; 不异常、不移动 (GPT 交接单第 2 条)."""
    from m3pro_nav.feedback_trajectory_follower import FollowerPhase

    source = Pose2D(0.2, 0.2, 0.37)
    stop = MotionPrimitive('STOP', source.copy(), p0=(source.x, source.y),
                           yaw0=source.yaw, duration=0.2)
    follow = follower([stop], source, source)
    state = follow.update(0.1, odom(source, stamp=1.0))
    assert state.phase == FollowerPhase.HOLDING
    assert state.command.vx == state.command.vy == state.command.wz == 0.0
    settled = follow.update(0.25, odom(source, stamp=2.0))
    assert settled.phase == FollowerPhase.HOLDING
    done = follow.update(0.36, odom(source, stamp=3.0))
    assert done.phase == FollowerPhase.FINISHED
