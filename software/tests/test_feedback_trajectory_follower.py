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


def test_settle_dwell_does_not_complete_from_a_frozen_odometry_stamp():
    source = Pose2D(0.2, 0.2, 0.37)
    stop = MotionPrimitive('STOP', source.copy(), p0=(source.x, source.y),
                           yaw0=source.yaw, duration=0.0)
    follow = follower([stop], source, source)
    state = follow.update(0.0, odom(source, stamp=10.0))
    assert state.phase.name == 'HOLDING' and not state.complete
    for tick in range(1, 21):
        state = follow.update(tick * 0.02, odom(source, stamp=10.0))
    assert state.phase.name == 'HOLDING'
    assert not state.complete


def test_settle_dwell_completes_after_advancing_source_time_spans_dwell():
    source = Pose2D(0.2, 0.2, 0.37)
    stop = MotionPrimitive('STOP', source.copy(), p0=(source.x, source.y),
                           yaw0=source.yaw, duration=0.0)
    follow = follower([stop], source, source)
    state = None
    for tick in range(16):
        now = tick * 0.02
        state = follow.update(now, odom(source, stamp=10.0 + now))
    assert state is not None and state.complete
    assert state.phase.name == 'FINISHED'


def test_progress_feedback_limits_reference_lead_when_time_profile_runs_ahead():
    start = Pose2D(0.0, 0.0, 0.0)
    end = Pose2D(0.8, 0.0, 0.0)
    line = MotionPrimitive('STRAIGHT', start, p0=(0.0, 0.0), p1=(0.8, 0.0),
        yaw0=0.0, length=0.8, v_max=0.5, v_end=0.0)
    stop = MotionPrimitive('STOP', end, p0=(0.8, 0.0), yaw0=0.0, duration=0.0)
    follow = FeedbackTrajectoryFollower(
        [line, stop], planner_start=start, odom_start=start,
        a_acc=1.0, a_dec=0.6, controller=PositionController(kp_pos=1.0,
        kd_vel=0.2), position_tolerance=0.025, yaw_tolerance=0.06,
        velocity_tolerance=0.025, yaw_rate_tolerance=0.15, settle_time=0.3,
        max_reference_lead_m=0.08, max_linear_speed_mps=0.5,
        max_command_accel_mps2=1.0, max_command_decel_mps2=0.6)

    # The nominal clock expires while odometry is still at the start. It must
    # not teleport the reference to the terminal or report terminal holding.
    expired = follow.update(follow.profile.duration,
                            odom(start, stamp=2.0))
    assert not expired.schedule_complete
    assert expired.phase.name == 'TRACKING'
    assert expired.reference.progress_s <= 0.08 + 1e-9
    assert not expired.complete


def test_first_tick_is_acceleration_limited_and_frozen_odom_cannot_advance_route():
    start = Pose2D(0.0, 0.0, 0.0)
    end = Pose2D(0.8, 0.0, 0.0)
    line = MotionPrimitive('STRAIGHT', start, p0=(0.0, 0.0), p1=(0.8, 0.0),
        yaw0=0.0, length=0.8, v_max=0.5, v_end=0.0)
    stop = MotionPrimitive('STOP', end, p0=(0.8, 0.0), yaw0=0.0, duration=0.0)
    follow = FeedbackTrajectoryFollower(
        [line, stop], planner_start=start, odom_start=start,
        a_acc=1.0, a_dec=0.6, controller=PositionController(kp_pos=1.0,
        kd_vel=0.2), position_tolerance=0.025, yaw_tolerance=0.06,
        velocity_tolerance=0.025, yaw_rate_tolerance=0.15, settle_time=0.3,
        max_reference_lead_m=0.08, max_linear_speed_mps=0.5,
        max_command_accel_mps2=1.0, max_command_decel_mps2=0.6)
    first = follow.update(0.02, odom(start, stamp=1.0))
    assert math.hypot(first.command.vx, first.command.vy) <= 0.02 + 1e-9

    state = first
    for tick in range(1, 81):
        # New control ticks deliberately reuse one stale pose and source stamp.
        state = follow.update(0.02 * (tick + 1), odom(start, stamp=1.0))
    assert state.measured_progress_s == 0.0
    assert state.reference.progress_s <= 0.08 + 1e-9
    assert state.phase.name == 'TRACKING'
    assert not state.complete


def test_slow_plant_tracks_target_and_brakes_from_measured_progress():
    """A 0.4 m/s plant lags a 0.5 m/s plan but still stops at the target."""
    start = Pose2D(0.0, 0.0, 0.0)
    end = Pose2D(0.8, 0.0, 0.0)
    line = MotionPrimitive('STRAIGHT', start, p0=(0.0, 0.0), p1=(0.8, 0.0),
        yaw0=0.0, length=0.8, v_max=0.5, v_end=0.0)
    stop = MotionPrimitive('STOP', end, p0=(0.8, 0.0), yaw0=0.0, duration=0.0)
    follow = FeedbackTrajectoryFollower(
        [line, stop], planner_start=start, odom_start=start,
        a_acc=1.0, a_dec=0.6, controller=PositionController(kp_pos=1.0,
        kd_vel=0.2), position_tolerance=0.025, yaw_tolerance=0.06,
        velocity_tolerance=0.025, yaw_rate_tolerance=0.15, settle_time=0.3,
        max_reference_lead_m=0.08, max_linear_speed_mps=0.5,
        max_command_accel_mps2=1.0, max_command_decel_mps2=0.6)

    x = measured_speed = 0.0
    max_lead = max_command = 0.0
    previous_command = 0.0
    previous_t = 0.0
    final = None
    for tick in range(400):
        t = tick * 0.025
        measured = odom(Pose2D(x, 0.0, 0.0), vx=measured_speed, stamp=t)
        final = follow.update(t, measured)
        max_lead = max(max_lead,
                       final.reference.progress_s-final.measured_progress_s)
        max_command = max(max_command, abs(final.command.vx))
        if tick:
            assert abs(final.command.vx-previous_command) <= 1.0 * (t-previous_t) + 1e-9
        previous_command, previous_t = final.command.vx, t

        # Simplified lagging drive: actuator saturates at 0.4 m/s and has a
        # lower acceleration than the reference planner.
        drive_target = min(0.4, max(-0.4, final.command.vx))
        dv = min(0.8*0.025, max(-0.8*0.025, drive_target-measured_speed))
        measured_speed += dv
        x += measured_speed * 0.025
        if final.complete:
            break

    assert final is not None and final.complete
    assert final.command.vx == final.command.vy == 0.0
    assert max_lead <= 0.08 + 1e-9
    assert max_command <= 0.5 + 1e-9
    assert x == pytest.approx(0.8, abs=0.025)
    assert abs(measured_speed) <= 0.025


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
