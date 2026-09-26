import math
import os
import sys
import inspect

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.action_horizon import ActionHorizon  # noqa: E402
from m3pro_nav.motion_planner import (  # noqa: E402
    MotionPlanner, PlanGeometryMismatch, R_ARC, GRAB_V, validate_geometry,
)
from m3pro_nav.pose import C, DIRV, Pose2D  # noqa: E402
from m3pro_nav.motion_primitive import MotionPrimitive  # noqa: E402
from m3pro_nav.edge_map import OPEN  # noqa: E402
from m3pro_nav.stream_nav import StreamNav  # noqa: E402
from m3pro_nav.motion_executor import MotionExecutor  # noqa: E402


def _straight(p0, p1):
    return MotionPrimitive('STRAIGHT', Pose2D(*p0, 0.0), p0=p0, p1=p1,
                           length=math.dist(p0, p1))


def test_straight_rejects_diagonal_motion():
    with pytest.raises(ValueError, match='must be axis aligned'):
        _straight((0.1, 0.2), (0.3, 0.4))


def test_reverse_is_a_compile_time_macro_not_a_runtime_kind():
    """GPT 定稿 (方案 B): REVERSE 是编译期宏 (死路折返 = 两段 STRAIGHT),
    运行时链中不允许出现 kind='REVERSE'."""
    with pytest.raises(ValueError, match='unknown motion primitive kind'):
        MotionPrimitive('REVERSE', Pose2D(0.4, 0.4, 0.0),
                        p0=(0.4, 0.4), p1=(0.0, 0.4), length=0.4)
    # 死路折返由 planner.template 编译为轴对齐 STRAIGHT 对
    planner = MotionPlanner()
    pose = Pose2D(0.2, 0.2, 0.37)
    root, pt, _ = planner.template(None, (0, 0), (1, 0), pose)
    retreat, pt2, _ = planner.template((0, 0), (1, 0), (0, 0),
                                       Pose2D(*pt, 0.37))
    assert all(p.kind == 'STRAIGHT' for p in root + retreat)
    assert all(p.length > 0 for p in retreat)


def test_primitive_rejects_zero_distance_and_wrong_path_length():
    with pytest.raises(ValueError, match='nonzero'):
        _straight((0.1, 0.2), (0.1, 0.2))
    with pytest.raises(ValueError, match='length must match endpoints'):
        MotionPrimitive('STRAIGHT', Pose2D(0.0, 0.0, 0.0),
                        p0=(0.0, 0.0), p1=(0.4, 0.0), length=0.2)
    with pytest.raises(ValueError, match='quarter-circle length'):
        MotionPrimitive('ARC', Pose2D(0.0, 0.0, 0.0), p0=(0.2, 0.2),
                        yaw0=math.pi, yaw1=-math.pi / 2, length=0.1,
                        meta={'r': 0.2})


@pytest.mark.parametrize('radius, sweep, message', [
    (0.25, math.pi / 2, 'radius must be 0.2'),
    (0.2, math.pi / 3, 'sweep must be a signed quarter turn'),
])
def test_arc_rejects_nonstandard_radius_or_sweep_at_construction(radius, sweep, message):
    with pytest.raises(ValueError, match=message):
        MotionPrimitive('ARC', Pose2D(0.0, 0.0, 0.0), p0=(0.0, 0.0),
                        yaw0=0.0, yaw1=sweep, meta={'r': radius})


def test_straight_rejects_motion_outside_its_cell_template():
    prim = _straight((0.6, 0.8), (0.8, 0.8))
    prim.meta['template'] = ((1, 1), (2, 2), (3, 2))
    with pytest.raises(PlanGeometryMismatch, match='leaves its template cell'):
        validate_geometry([prim])


def test_turn_templates_cover_all_eight_cardinal_quarter_arcs():
    planner = MotionPlanner()
    dirs = tuple(DIRV.values())
    cell = (2, 2)
    center = ((cell[0] + 0.5) * C, (cell[1] + 0.5) * C)
    count = 0
    for din in dirs:
        for dout in dirs:
            if dout == din or dout == (-din[0], -din[1]):
                continue
            prev = (cell[0] - din[0], cell[1] - din[1])
            nxt = (cell[0] + dout[0], cell[1] + dout[1])
            entry = (center[0] - din[0] * R_ARC,
                     center[1] - din[1] * R_ARC)
            prims, end, _ = planner.template(prev, cell, nxt,
                                                Pose2D(*entry, 0.0))
            validate_geometry(prims, cursor=(prev, cell))
            arc = next(p for p in prims if p.kind == 'ARC')
            assert arc.meta['r'] == pytest.approx(0.2)
            assert abs(arc.yaw1) == pytest.approx(math.pi / 2)
            assert end == pytest.approx((center[0] + dout[0] * R_ARC,
                                        center[1] + dout[1] * R_ARC))
            count += 1
    assert count == 8


def test_template_entry_anchor_mismatch_is_reported_with_cursor_context():
    planner = MotionPlanner()
    prev, cell, nxt = (1, 2), (2, 2), (3, 2)
    with pytest.raises(PlanGeometryMismatch) as err:
        planner.template(prev, cell, nxt, Pose2D(0.99, 1.0, 0.0))
    assert err.value.context['cursor'] == (prev, cell)
    assert err.value.context['expected'] == pytest.approx((0.8, 1.0))


def test_adjacent_primitives_must_share_exact_anchor():
    with pytest.raises(PlanGeometryMismatch, match='discontinuous'):
        validate_geometry([
            _straight((0.0, 0.0), (0.4, 0.0)),
            _straight((0.4, 0.01), (0.8, 0.01)),
        ])


def test_motion_planner_exposes_geometry_only_api():
    assert hasattr(MotionPlanner, 'template')
    assert not hasattr(MotionPlanner, 'compile_chain')
    assert tuple(inspect.signature(MotionPlanner.compile_home).parameters) == (
        'self', 'pose', 'cursor', 'path_cells', 'exit_dir')


def _confirm_cell(nav, cell, openings):
    for direction in ('N', 'E', 'S', 'W'):
        observe = nav.edges.observe_open if direction in openings else nav.edges.observe_wall
        observe(cell, direction, 0.2)
        observe(cell, direction, 0.2)


def test_terminal_arc_brakes_to_stop_on_same_fixed_quarter_circle():
    nav = StreamNav((0, 0), n=7)
    cell, prev = (3, 3), (3, 2)
    _confirm_cell(nav, cell, {'S', 'E'})
    planner = MotionPlanner()
    pose = Pose2D(*((cell[0] + 0.5) * C, cell[1] * C), 0.31)
    prims, _, _ = ActionHorizon(planner).compile(nav, pose, (prev, cell))
    assert [p.kind for p in prims] == ['ARC', 'STOP']
    arc = prims[0]
    assert arc.v_end == 0.0
    assert arc.meta['r'] == pytest.approx(0.2)
    assert abs(arc.yaw1) == pytest.approx(math.pi / 2)

    executor = MotionExecutor(pose, a_acc=2.0, a_dec=1.0)
    executor.set_plan(prims)
    speeds = []
    yaw = executor.pose.yaw
    while executor.queue and executor.queue[0].kind == 'ARC':
        v_before = executor.v
        executor.step(0.005)
        speeds.append(executor.v)
        assert abs(executor.pose.yaw - yaw) < 1e-12
        assert math.hypot(executor.pose.x - arc.p0[0],
                          executor.pose.y - arc.p0[1]) == pytest.approx(0.2)
        delta_v = executor.v - v_before
        limit = executor.a_acc if delta_v >= 0 else executor.a_dec
        assert abs(delta_v) <= limit * 0.005 + 1e-8
    assert executor.queue and executor.queue[0].kind == 'STOP'
    assert executor.v == 0.0
    assert max(speeds) > 0.3
    peak_index = max(range(len(speeds)), key=speeds.__getitem__)
    assert any(later < earlier for earlier, later in zip(speeds[peak_index:],
                                                        speeds[peak_index + 1:]))
    assert all(later <= earlier + 1e-8
               for earlier, later in zip(speeds[peak_index:], speeds[peak_index + 1:]))


def test_arc_endpoint_roundoff_does_not_leave_a_residual_stop_speed():
    nav = StreamNav((0, 0), n=7)
    cell, prev = (3, 3), (3, 2)
    _confirm_cell(nav, cell, {'S', 'E'})
    pose = Pose2D(*((cell[0] + 0.5) * C, cell[1] * C), 0.0)
    arc = ActionHorizon(MotionPlanner()).compile(nav, pose, (prev, cell))[0][0]

    # This is the last tiny arc interval from seed 153: accumulated distance
    # rounding makes the required deceleration exceed a_dec by ~3e-9 m/s².
    remaining = 1.7483824721331587e-8
    arc.progress = arc.length - remaining
    executor = MotionExecutor(pose, a_acc=1.0, a_dec=1.0)
    executor.v = math.sqrt(2 * remaining * (1 + 2.7e-9))

    executor._advance_arc(arc, 0.02)

    assert arc.done
    assert executor.v == 0.0


def test_arc_followed_by_straight_keeps_tangential_speed_continuity():
    nav = StreamNav((0, 0), n=7)
    first, prev = (3, 3), (3, 2)
    second = (4, 3)
    _confirm_cell(nav, first, {'S', 'E'})
    _confirm_cell(nav, second, {'W', 'E'})
    planner = MotionPlanner()
    pose = Pose2D(*((first[0] + 0.5) * C, first[1] * C), 0.0)
    prims, _, _ = ActionHorizon(planner).compile(nav, pose, (prev, first))
    assert [p.kind for p in prims[:2]] == ['ARC', 'STRAIGHT']
    assert prims[0].v_end == pytest.approx(planner.v_arc)

    executor = MotionExecutor(pose, a_acc=1.0, a_dec=1.0)
    executor.v = planner.v_arc
    executor.set_plan(prims)
    dt = 0.005
    while executor.queue[0] is prims[0]:
        executor.step(dt)
    assert executor.queue[0] is prims[1]
    assert planner.v_arc <= executor.v <= planner.v_arc + executor.a_acc * dt + 1e-8


def test_short_straight_to_arc_keeps_achieved_speed_without_terminal_snap():
    start = Pose2D(0.0, 0.0, 0.0)
    line = MotionPrimitive(
        'STRAIGHT', start, p0=(0.0, 0.0), p1=(0.001, 0.0),
        length=0.001, v_max=0.45, v_end=0.45)
    arc = MotionPrimitive(
        'ARC', Pose2D(0.001, 0.0, 0.0), p0=(-0.199, 0.0),
        yaw0=0.0, yaw1=math.pi / 2, length=0.2 * math.pi / 2,
        v_max=0.45, v_end=0.45, meta={'r': 0.2})
    executor = MotionExecutor(start, a_acc=1.0, a_dec=1.0)
    executor.set_plan([line, arc])

    while executor.queue[0].kind == 'STRAIGHT':
        v_before = executor.v
        executor.step(0.01)
        assert executor.v - v_before <= executor.a_acc * 0.01 + 1e-9
    assert executor.queue[0].kind == 'ARC'
    exit_speed = math.sqrt(2 * executor.a_acc * line.length)
    assert executor.v >= exit_speed
    assert executor.v <= exit_speed + executor.a_acc * 0.01 + 1e-9
    assert executor.v < line.v_end
    v_before = executor.v
    executor.step(0.01)
    assert executor.v - v_before <= executor.a_acc * 0.01 + 1e-9


def test_straight_to_stop_reaches_zero_within_deceleration_limit():
    start = Pose2D(0.0, 0.0, 0.0)
    line = MotionPrimitive(
        'STRAIGHT', start, p0=(0.0, 0.0), p1=(0.4, 0.0),
        length=0.4, v_max=0.45, v_end=0.0)
    stop = MotionPrimitive(
        'STOP', Pose2D(0.4, 0.0, 0.0), p0=(0.4, 0.0), duration=0.1)
    executor = MotionExecutor(start, a_acc=1.0, a_dec=1.0)
    executor.v = 0.45
    executor.set_plan([line, stop])
    speeds = [executor.v]
    while executor.queue[0].kind == 'STRAIGHT':
        v_before = executor.v
        executor.step(0.005)
        delta_v = executor.v - v_before
        assert delta_v >= -executor.a_dec * 0.005 - 1e-9
        speeds.append(executor.v)
    assert executor.queue[0].kind == 'STOP'
    assert executor.v == pytest.approx(0.0, abs=1e-6)
    assert any(later < earlier for earlier, later in zip(speeds, speeds[1:]))


def test_stop_accepts_only_residual_within_one_micron_stopping_distance():
    pose = Pose2D(0.0, 0.0, 0.0)
    stop = MotionPrimitive('STOP', pose, duration=0.1)
    executor = MotionExecutor(pose, a_dec=1.0)
    executor.v = math.sqrt(2 * executor.a_dec * 0.5e-6)
    executor._advance(stop, 0.01)
    assert executor.v == 0.0

    unsafe = MotionExecutor(pose, a_dec=1.0)
    unsafe.v = math.sqrt(2 * unsafe.a_dec * 2e-6)
    with pytest.raises(RuntimeError, match='STOP reached with nonzero speed'):
        unsafe._advance(MotionPrimitive('STOP', pose, duration=0.1), 0.01)


def test_dead_end_reversal_slows_before_entry_and_stops_before_turnaround():
    nav = StreamNav((0, 0), n=7)
    parent, prev, dead_end = (3, 3), (3, 2), (3, 4)
    _confirm_cell(nav, parent, {'S', 'N'})
    _confirm_cell(nav, dead_end, {'S'})
    planner = MotionPlanner()
    pose = Pose2D((parent[0] + 0.5) * C, parent[1] * C, 0.0)
    prims, _, _ = ActionHorizon(planner, max_steps=2).compile(
        nav, pose, (prev, parent))
    assert len(prims) == 3
    assert prims[0].v_end == pytest.approx(GRAB_V)
    assert prims[1].p1 == pytest.approx(prims[2].p0)
    assert prims[1].p1 != prims[2].p1

    executor = MotionExecutor(pose, a_acc=1.0, a_dec=1.0)
    executor.v = 0.7
    executor.set_plan(prims)
    reversal_started = False
    dt = 0.005
    while executor.queue:
        v_before = executor.v
        executor.step(dt)
        assert executor.v - v_before <= executor.a_acc * dt + 1e-8
        assert v_before - executor.v <= executor.a_dec * dt + 1e-8
        if executor.queue and executor.queue[0] is prims[2]:
            reversal_started = True
            assert executor.v <= executor.a_acc * dt + 1e-8
            break
    assert reversal_started


def test_boundary_exit_physical_branch_mark_is_not_a_dfs_branch(monkeypatch):
    nav = StreamNav((0, 0), n=7)
    cell = (6, 3)
    # The boundary exit contributes a physical opening, so CellMark has degree
    # three. Interior graph degree is two and must compile as a WAY.
    for direction, is_open in (('N', True), ('E', False),
                                ('S', True), ('W', False)):
        observe = nav.edges.observe_open if is_open else nav.edges.observe_wall
        observe(cell, direction, 0.2)
        observe(cell, direction, 0.2)
    nav.edges.set_boundary(cell, 'E', OPEN)
    assert nav.mark(cell)['kind'] == 'BRANCH'
    assert len(nav.open_neighbors(cell)) == 2
    assert not nav.is_exploration_branch(cell)

    original_enter = nav.plan_branch_enter
    calls = []

    def track_enter(*args, **kwargs):
        calls.append(args)
        return original_enter(*args, **kwargs)

    monkeypatch.setattr(nav, 'plan_branch_enter', track_enter)
    prev = (6, 2)
    pose = Pose2D((cell[0] + 0.5) * C, cell[1] * C, 0.0)
    prims, _, seq = ActionHorizon(MotionPlanner()).compile(nav, pose, (prev, cell))
    assert calls == []
    assert seq[:2] == [cell, (6, 4)]
    assert prims[0].kind == 'STRAIGHT'
