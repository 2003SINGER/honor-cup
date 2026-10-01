import math

from m3pro_nav.ground_loop_trial import (compile_loop_route, nominal_route_end,
                                         validate_trial_limits)
from m3pro_nav.feedback_trajectory_follower import FeedbackTrajectoryFollower
from m3pro_nav.motion_planner import MotionPlanner
from m3pro_nav.odometry_adapter import OdometryState
from m3pro_nav.pose import Pose2D
from m3pro_nav.position_controller import PositionController
from m3pro_nav.speed_profile import SpeedProfile
from m3pro_nav.trajectory_reference import TrajectoryReference


def test_ground_loop_route_uses_fixed_left_and_reverse_turn_templates_and_closes():
    primitives = compile_loop_route(speed=0.15)
    start = (6.5 * 0.4, 2.5 * 0.4)
    end, length = nominal_route_end(primitives)
    assert math.dist(end, start) < 1e-9
    assert math.isclose(length, 1.4283185307179587, abs_tol=1e-9)
    assert [p.kind for p in primitives] == [
        'STRAIGHT', 'ARC', 'STRAIGHT', 'STRAIGHT', 'ARC', 'STRAIGHT', 'STOP']
    arcs = [p for p in primitives if p.kind == 'ARC']
    assert [p.meta['template'] for p in arcs] == [
        ((6, 2), (6, 3), (5, 3)), ((5, 3), (6, 3), (6, 2))]
    assert [p.meta['r'] for p in arcs] == [0.2, 0.2]


def test_ground_loop_profile_fits_one_closed_route_and_holds_heading():
    primitives = compile_loop_route(speed=0.50)
    profile = SpeedProfile(primitives, start_speed=0.0, a_acc=1.0, a_dec=1.0)
    reference = TrajectoryReference(primitives, yaw_ref=math.pi / 2)
    assert profile.duration < 5.0
    assert max(profile.sample(profile.duration * tick / 1000).speed
               for tick in range(1001)) >= 0.499
    assert [p.v_max for p in primitives if p.kind == 'ARC'] == [0.45, 0.45]
    for s in (0.2, 0.2 + math.pi * 0.2 / 4, 0.2 + math.pi * 0.2 / 2,
              1.0, profile.length):
        sample = reference.sample(s)
        assert math.isclose(sample.yaw_ref, math.pi / 2)


def test_route_builder_rejects_wrong_origin_or_unbounded_speed():
    try:
        compile_loop_route(start_cell=(5, 2))
    except ValueError as exc:
        assert 'only for start cell' in str(exc)
    else:
        raise AssertionError('wrong origin accepted')
    try:
        compile_loop_route(speed=0.51)
    except ValueError as exc:
        assert 'speed must be in' in str(exc)
    else:
        raise AssertionError('speed above field cap accepted')


def test_speed_and_command_cap_allow_bounded_high_speed_profile():
    assert validate_trial_limits(0.15, 0.25) == (0.15, 0.25)
    assert validate_trial_limits(0.20, 0.25) == (0.20, 0.25)
    assert validate_trial_limits(0.50, 0.50) == (0.50, 0.50)
    assert validate_trial_limits(0.50, 0.70) == (0.50, 0.70)
    compile_loop_route(speed=0.50)
    for speed, cap in ((0.50, 0.49), (0.51, 0.70), (0.15, 0.71)):
        try:
            validate_trial_limits(speed, cap)
        except ValueError:
            pass
        else:
            raise AssertionError(f'unsafe limits accepted: speed={speed}, cap={cap}')


def test_follower_accumulates_settle_time_after_profile_duration():
    primitives = compile_loop_route()
    start = Pose2D(6.5 * 0.4, 2.5 * 0.4, math.pi / 2)
    follower = FeedbackTrajectoryFollower(
        primitives, planner_start=start, odom_start=start,
        a_acc=0.20, a_dec=0.20, controller=PositionController(kd_vel=0.20),
        start_speed=0.0, position_tolerance=0.025, yaw_tolerance=0.06,
        velocity_tolerance=0.025, yaw_rate_tolerance=0.15, settle_time=0.30,
        expected_odom_frame='odom', expected_base_frame='base_footprint')
    profile_end = follower.profile.duration
    measured = OdometryState(start, 0.0, 0.0, 0.0, 1.0,
                             'odom', 'base_footprint')
    states = [follower.update(profile_end + tick * 0.02, measured)
              for tick in range(20)]
    assert states[-1].complete
    assert states[-1].command.vx == 0.0
    assert states[-1].command.vy == 0.0
