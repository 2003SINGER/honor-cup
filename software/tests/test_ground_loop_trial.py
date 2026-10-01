import math
import pytest

from m3pro_nav.ground_loop_trial import (compile_loop_route, nominal_route_end,
                                         validate_controller_gains,
                                         split_route_at_west_center,
                                         validate_trial_limits, parser)
from m3pro_nav.feedback_trajectory_follower import FeedbackTrajectoryFollower
from m3pro_nav.frame_transform import RigidFrameTransform
from m3pro_nav.ground_loop_trial import (distance_to_polyline,
                                         route_corridor_waypoints)
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


def test_route_projection_recovers_line_arc_line_progress_without_reversing():
    primitives = compile_loop_route(speed=0.50)
    reference = TrajectoryReference(primitives, yaw_ref=math.pi / 2)
    distances = (0.05, 0.20, 0.20 + math.pi * 0.2 / 4,
                 0.20 + math.pi * 0.2 / 2, 0.8, reference.length - 0.04)
    previous = 0.0
    for progress in distances:
        point = reference.sample(progress)
        projected, error = reference.project(
            point.x, point.y, minimum_progress=previous,
            direction=(point.tangent_x, point.tangent_y),
            preferred_progress=progress)
        assert projected == pytest.approx(progress, abs=1e-8)
        assert error == pytest.approx(0.0, abs=1e-8)
        assert projected >= previous
        previous = projected


def test_west_center_split_preserves_route_and_adds_zero_speed_terminal_stop():
    primitives = compile_loop_route(speed=0.50)
    outbound, returning, return_index, midpoint = \
        split_route_at_west_center(primitives)
    assert return_index == 3
    assert [p.meta.get('field_trial_segment') for p in outbound[:-1]] == [
        None, 'left_arc_out', 'west_to_cell_center']
    assert outbound[-1].kind == 'STOP'
    assert outbound[-1].meta['field_trial_phase_stop'] == 'midpoint'
    assert outbound[-2].v_end == 0.0
    assert returning[0].meta['field_trial_segment'] == 'east_back_to_arc'
    assert math.dist((outbound[-1].start_pose.x, outbound[-1].start_pose.y),
                     (midpoint.x, midpoint.y)) < 1e-12
    assert math.dist((returning[0].p0[0], returning[0].p0[1]),
                     (midpoint.x, midpoint.y)) < 1e-12
    original_length = nominal_route_end(primitives)[1]
    split_length = nominal_route_end(outbound)[1] + nominal_route_end(returning)[1]
    assert math.isclose(split_length, original_length, abs_tol=1e-12)


def test_midpoint_phase_holds_for_measured_settle_and_return_reuses_transform():
    primitives = compile_loop_route(speed=0.50)
    outbound, returning, _, midpoint = split_route_at_west_center(primitives)
    start = Pose2D(6.5 * 0.4, 2.5 * 0.4, math.pi / 2)
    outbound_follower = FeedbackTrajectoryFollower(
        outbound, planner_start=start, odom_start=start,
        a_acc=1.0, a_dec=0.6, controller=PositionController(kp_pos=1.0,
        kd_vel=0.2), start_speed=0.0, position_tolerance=0.025,
        yaw_tolerance=0.06, velocity_tolerance=0.025,
        yaw_rate_tolerance=0.15, settle_time=0.30,
        expected_odom_frame='odom', expected_base_frame='base_footprint')
    midpoint_odom_pose = outbound_follower.transform.transform_pose(midpoint)
    moving = OdometryState(midpoint_odom_pose, 0.0, 0.40, 0.0, 1.0,
                           'odom', 'base_footprint')
    held = outbound_follower.update(outbound_follower.profile.duration, moving)
    assert held.phase.name == 'HOLDING'
    assert not held.complete

    stopped = OdometryState(midpoint_odom_pose, 0.0, 0.0, 0.0, 1.1,
                            'odom', 'base_footprint')
    for tick in range(17):
        stopped = OdometryState(midpoint_odom_pose, 0.0, 0.0, 0.0,
                                1.1 + 0.02 * tick,
                                'odom', 'base_footprint')
        held = outbound_follower.update(
            outbound_follower.profile.duration + 0.02 * tick, stopped)
    assert held.complete

    returning_follower = FeedbackTrajectoryFollower(
        returning, planner_start=Pose2D(midpoint.x, midpoint.y, math.pi / 2),
        odom_start=None, transform=outbound_follower.transform,
        a_acc=1.0, a_dec=0.6,
        controller=outbound_follower.controller, start_speed=0.0,
        position_tolerance=0.025, yaw_tolerance=0.06,
        velocity_tolerance=0.025, yaw_rate_tolerance=0.15,
        settle_time=0.30, expected_odom_frame='odom',
        expected_base_frame='base_footprint')
    state = returning_follower.update(0.0, stopped)
    assert math.dist((state.reference.x, state.reference.y),
                     (midpoint_odom_pose.x, midpoint_odom_pose.y)) < 1e-12
    assert returning_follower.transform is outbound_follower.transform


def test_2d_line_arc_line_lagging_plant_returns_without_exceeding_corridor():
    """Exercise the actual straight/quarter-arc/straight loop with 0.4 m/s plant."""
    primitives = compile_loop_route(speed=0.50)
    outbound, returning, _, midpoint = split_route_at_west_center(primitives)
    start = Pose2D(6.5 * 0.4, 2.5 * 0.4, math.pi / 2)
    controller = PositionController(kp_pos=1.0, kd_vel=0.2)
    limits = dict(a_acc=1.0, a_dec=0.6,
        position_tolerance=0.025, yaw_tolerance=0.06,
        velocity_tolerance=0.025, yaw_rate_tolerance=0.15, settle_time=0.30,
        max_reference_lead_m=0.08, max_linear_speed_mps=0.50,
        max_command_accel_mps2=1.0, max_command_decel_mps2=0.6)
    first = FeedbackTrajectoryFollower(
        outbound, planner_start=start, odom_start=start,
        controller=controller, expected_odom_frame='odom',
        expected_base_frame='base_footprint', **limits)
    corridor = route_corridor_waypoints(primitives, first.transform, samples=240)

    x, y, vx, vy = start.x, start.y, 0.0, 0.0
    global_time = 0.0
    max_corridor_error = 0.0
    max_command_speed = 0.0
    previous_command = (0.0, 0.0)
    previous_command_time = None

    def simulate(follow, *, overshoot_axis, stop_coordinate):
        nonlocal x, y, vx, vy, global_time
        nonlocal max_corridor_error, max_command_speed
        nonlocal previous_command, previous_command_time
        latest_odom = None
        max_overshoot = 0.0
        hold_entry_speed = None
        for tick in range(400):
            local_time = tick * 0.05
            global_time = global_time + 0.05 if tick else global_time
            # Model the real ~10 Hz odometry by holding each sample for two
            # 20 Hz command ticks.
            if tick % 2 == 0 or latest_odom is None:
                latest_odom = OdometryState(Pose2D(x, y, start.yaw), vx, vy,
                    0.0, global_time, 'odom', 'base_footprint')
            state = follow.update(local_time, latest_odom)
            if state.phase.name == 'HOLDING' and hold_entry_speed is None:
                hold_entry_speed = math.hypot(vx, vy)
            error = distance_to_polyline((x, y), corridor)
            max_corridor_error = max(max_corridor_error, error)
            command_speed = math.hypot(state.command.vx, state.command.vy)
            max_command_speed = max(max_command_speed, command_speed)
            if previous_command_time is not None:
                dt = global_time - previous_command_time
                delta = math.hypot(state.command.vx-previous_command[0],
                                   state.command.vy-previous_command[1])
                assert delta <= 1.0 * dt + 1e-9
            previous_command = (state.command.vx, state.command.vy)
            previous_command_time = global_time

            # yaw is held north. Convert body x/y command to world and model
            # a slower omnidirectional chassis with 0.4 m/s speed and 0.8 m/s².
            world_x = -state.command.vy
            world_y = state.command.vx
            ax, ay = world_x-vx, world_y-vy
            acceleration = math.hypot(ax, ay)
            if acceleration > 0.8 * 0.05:
                scale = 0.8 * 0.05 / acceleration
                ax, ay = ax*scale, ay*scale
            vx, vy = vx+ax, vy+ay
            speed = math.hypot(vx, vy)
            if speed > 0.4:
                vx, vy = vx*0.4/speed, vy*0.4/speed
            x, y = x+vx*0.05, y+vy*0.05
            coordinate = x if overshoot_axis == 'x' else y
            max_overshoot = max(max_overshoot, stop_coordinate - coordinate)
            if state.complete:
                assert state.command.vx == state.command.vy == 0.0
                global_time += 0.05
                return state, max_overshoot, hold_entry_speed
        raise AssertionError('lagging 2D plant failed to settle before timeout')

    first_done, first_overshoot, first_hold_speed = simulate(
        first, overshoot_axis='x', stop_coordinate=midpoint.x)
    assert first_done.measured_progress_s >= first.reference.length - 0.025
    assert first_overshoot <= 0.025
    assert first_hold_speed is not None and first_hold_speed < 0.15
    second = FeedbackTrajectoryFollower(
        returning,
        planner_start=Pose2D(midpoint.x, midpoint.y, math.pi / 2),
        odom_start=None, transform=first.transform, controller=controller,
        expected_odom_frame='odom', expected_base_frame='base_footprint',
        **limits)
    final, final_overshoot, final_hold_speed = simulate(
        second, overshoot_axis='y', stop_coordinate=start.y)
    expected = (start.x, start.y)
    assert math.hypot(x-expected[0], y-expected[1]) <= 0.025
    assert max_corridor_error <= 0.08
    assert max_command_speed <= 0.50 + 1e-9
    assert math.hypot(vx, vy) <= 0.025
    assert final_overshoot <= 0.025
    assert final_hold_speed is not None and final_hold_speed < 0.15
    assert final.complete


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
    assert validate_trial_limits(0.50, 0.50) == (0.50, 0.50)
    compile_loop_route(speed=0.50)
    for speed, cap in ((0.50, 0.49), (0.51, 0.50), (0.15, 0.51)):
        try:
            validate_trial_limits(speed, cap)
        except ValueError:
            pass
        else:
            raise AssertionError(f'unsafe limits accepted: speed={speed}, cap={cap}')


def test_field_trial_default_gains_change_kp_only_and_overrides_are_bounded():
    # Restore the successful 0.5 m/s baseline after kp=1.5 exceeded the
    # geometric corridor; CLI overrides retain bounded follow-up tuning.
    assert validate_controller_gains(1.0, 0.2) == (1.0, 0.2)
    defaults = parser().parse_args([
        '--run', '--expected-odom-frame', 'odom',
        '--expected-base-frame', 'base_footprint'])
    assert (defaults.kp_pos, defaults.kd_vel) == (1.0, 0.2)
    overrides = parser().parse_args([
        '--run', '--expected-odom-frame', 'odom',
        '--expected-base-frame', 'base_footprint',
        '--kp-pos', '1.25', '--kd-vel', '0.3'])
    assert (overrides.kp_pos, overrides.kd_vel) == (1.25, 0.3)
    for kp, kd in ((-0.01, 0.2), (2.01, 0.2), (1.5, -0.01),
                   (1.5, 1.01), (math.inf, 0.2), (1.5, math.nan)):
        try:
            validate_controller_gains(kp, kd)
        except ValueError:
            pass
        else:
            raise AssertionError(f'unsafe gain override accepted: kp={kp}, kd={kd}')


def test_follower_does_not_finish_when_odometry_has_not_progressed():
    primitives = compile_loop_route()
    start = Pose2D(6.5 * 0.4, 2.5 * 0.4, math.pi / 2)
    follower = FeedbackTrajectoryFollower(
        primitives, planner_start=start, odom_start=start,
        a_acc=0.20, a_dec=0.20, controller=PositionController(kd_vel=0.20),
        start_speed=0.0, position_tolerance=0.025, yaw_tolerance=0.06,
        velocity_tolerance=0.025, yaw_rate_tolerance=0.15, settle_time=0.30,
        expected_odom_frame='odom', expected_base_frame='base_footprint',
        max_reference_lead_m=0.08, max_linear_speed_mps=0.50,
        max_command_accel_mps2=1.0, max_command_decel_mps2=0.6)
    profile_end = follower.profile.duration
    states = []
    for tick in range(20):
        measured = OdometryState(start, 0.0, 0.0, 0.0,
                                 1.0 + tick * 0.02,
                                 'odom', 'base_footprint')
        states.append(follower.update(profile_end + tick * 0.02, measured))
    assert not states[-1].complete
    assert states[-1].measured_progress_s == 0.0
    assert states[-1].reference.progress_s <= 0.08


def test_geometric_corridor_accepts_time_lag_on_route_and_rejects_lateral_deviation():
    primitives = compile_loop_route(speed=0.50)
    anchor = Pose2D(6.5 * 0.4, 2.5 * 0.4, math.pi / 2)
    corridor = route_corridor_waypoints(
        primitives, RigidFrameTransform(anchor, anchor), samples=240)
    # This point lies on the westward half-cell segment but is behind a later
    # time reference; the geometric gate deliberately accepts it.
    on_route_behind_reference = (2.30, 1.40)
    assert distance_to_polyline(on_route_behind_reference, corridor) < 0.002
    off_route = (2.30, 1.50)
    assert distance_to_polyline(off_route, corridor) > 0.08
