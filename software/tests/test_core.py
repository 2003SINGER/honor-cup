"""Deterministic acceptance gates for the topology-first navigation contract.

These tests exercise externally visible decisions and forbidden dependencies;
they deliberately avoid reproducing planner implementation details.
"""

import ast
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sim'))

from m3pro_nav.edge_map import EdgeMap, TraversalMap, WALL, OPEN, T_FLIP
from m3pro_nav.pose import Pose2D, C
from m3pro_nav.stream_nav import StreamNav
from m3pro_nav.motion_planner import MotionPlanner, PlanGeometryMismatch
from m3pro_nav.action_horizon import ActionHorizon
from m3pro_nav.cell_classifier import classify, DEAD, WAY, BRANCH
from m3pro_nav import tree_inference


def _cell(nav, cell, open_dirs):
    """Confirm one local cell without marking any traversal events."""
    for _ in range(2):
        for direction in ('N', 'E', 'S', 'W'):
            observer = (nav.edges.observe_open if direction in open_dirs
                        else nav.edges.observe_wall)
            observer(cell, direction, 0.2)


def _edge_cell(walls, cell=(2, 2)):
    nav = StreamNav((0, 0), n=7)
    _cell(nav, cell, {'N', 'E', 'S', 'W'} - set(walls))
    return nav.edges, nav.traversal


def test_cell_classification_and_incomplete_information():
    em, tr = _edge_cell({'N', 'E', 'S'})
    assert classify(em, (2, 2), tr)['kind'] == DEAD
    em, tr = _edge_cell({'N', 'S'})
    assert classify(em, (2, 2), tr)['kind'] == WAY
    em, tr = _edge_cell({'S'})
    assert classify(em, (2, 2), tr)['kind'] == BRANCH
    assert classify(EdgeMap(7), (2, 2), TraversalMap(7)) is None


def test_local_graph_transition_exhaustive_for_every_opening_set_and_entry():
    """All local degrees and incoming sides resolve to a real graph neighbor."""
    from itertools import combinations
    from m3pro_nav.pose import DIRV

    cell = (3, 3)
    for count in range(1, 5):
        for openings in combinations(('N', 'E', 'S', 'W'), count):
            for incoming in openings:
                nav = StreamNav((0, 0), n=7)
                _cell(nav, cell, set(openings))
                dv = DIRV[incoming]
                prev = (cell[0] + dv[0], cell[1] + dv[1])
                actual = nav.resolve_next(prev, cell)
                available = {
                    (cell[0] + DIRV[d][0], cell[1] + DIRV[d][1])
                    for d in openings
                }
                assert actual in available
                if count == 1:
                    assert actual == prev
                elif count == 2:
                    assert actual == next(iter(available - {prev}))
                else:
                    assert actual in available - {prev}
                    assert nav.branch == {}  # preview has no commit side effect


def test_open_edges_are_not_walked_and_walked_edges_are_hard_open():
    em, tr = EdgeMap(7), TraversalMap(7)
    em.observe_open((2, 2), 'E', 0.2)
    em.observe_open((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E', tr) == OPEN
    assert not tr.is_walked((2, 2), 'E')
    tr.mark_crossed((2, 2), 'E')
    assert em.state((3, 2), 'W', tr) == OPEN
    for _ in range(10):
        em.observe_wall((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E', tr) == OPEN


def test_sensor_edge_reversal_removes_frontier():
    em, tr = EdgeMap(7), TraversalMap(7)
    for _ in range(2):
        em.observe_open((2, 2), 'E', 0.2)
    assert 'E' in em.frontier((2, 2), tr)
    for _ in range(T_FLIP + 1):
        em.observe_wall((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E', tr) == WALL
    assert 'E' not in em.frontier((2, 2), tr)


def test_tree_inference_wall_is_retractable():
    em, tr = EdgeMap(7), TraversalMap(7)
    for cell, direction in (((1, 2), 'S'), ((1, 1), 'S'), ((1, 0), 'E'),
                            ((2, 0), 'N'), ((2, 1), 'N')):
        em.observe_open(cell, direction, 0.2)
        em.observe_open(cell, direction, 0.2)
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) == WALL
    assert em.provenance((1, 2), 'E', tr) == 'TREE_INFERENCE'
    for _ in range(T_FLIP + 2):
        em.observe_wall((1, 0), 'E', 0.2)
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) != WALL


def _center(cell):
    return ((cell[0] + 0.5) * C, (cell[1] + 0.5) * C)


def _mid(cell, direction):
    from m3pro_nav.pose import DIRV
    x, y = _center(cell)
    return x + 0.2 * DIRV[direction][0], y + 0.2 * DIRV[direction][1]


@pytest.mark.parametrize(
    'open_dirs, previous_dir, expected_kind',
    [
        ({'N', 'S'}, 'S', 'way'),
        ({'S'}, 'S', 'dead'),
        ({'N', 'S', 'E'}, 'S', 'branch'),
    ],
)
def test_local_graph_resolves_from_previous_cell_without_visit_state(
        open_dirs, previous_dir, expected_kind):
    nav = StreamNav((0, 0), n=7)
    cell = (3, 3)
    from m3pro_nav.pose import DIRV
    dv = DIRV[previous_dir]
    previous = (cell[0] + dv[0], cell[1] + dv[1])
    _cell(nav, cell, open_dirs)

    # No VisitRegistry or EnteredCell event is supplied. The local graph alone
    # determines the transition from the (previous,current) cursor.
    result = nav.resolve_next(previous, cell)
    if expected_kind == 'dead':
        assert result == previous
    elif expected_kind == 'way':
        assert result == (cell[0], cell[1] + 1)
    else:
        assert result in {(cell[0], cell[1] + 1), (cell[0] + 1, cell[1])}
        assert nav.branch == {}  # preview does not commit a branch
    assert nav.visits.get(cell) is None


def test_incomplete_cell_waits_at_cursor_boundary_then_resumes_from_new_map():
    nav = StreamNav((0, 0), n=7)
    prev, cell = (2, 3), (3, 3)
    cursor = (prev, cell)
    pose = Pose2D(*_mid(cell, 'W'), 0.37)

    prims, terminal, seq = ActionHorizon(MotionPlanner()).compile(nav, pose, cursor)
    assert terminal == cursor
    assert seq == [cell]
    assert [p.kind for p in prims] == ['STOP']
    assert (prims[0].start_pose.x, prims[0].start_pose.y) == pytest.approx(
        (pose.x, pose.y))

    # A map update, without an EnteredCell event or pose nudge, releases the
    # wait and makes a forward template available immediately.
    _cell(nav, cell, {'W', 'E'})
    prims2, terminal2, seq2 = ActionHorizon(MotionPlanner()).compile(nav, pose, cursor)
    assert prims2 and prims2[0].kind == 'STRAIGHT'
    assert prims2[-1].kind == 'STOP'  # next cell is still an unknown boundary
    assert terminal2 == (cell, (cell[0] + 1, cell[1]))
    assert seq2[:2] == [cell, (cell[0] + 1, cell[1])]


def test_running_horizon_extends_after_new_map_without_replanning_started_motion():
    from copy import deepcopy
    from m3pro_nav.motion_executor import MotionExecutor

    nav = StreamNav((0, 0), n=7)
    branch, parent, north, east = (3, 3), (3, 2), (3, 4), (4, 3)
    _cell(nav, branch, {'S', 'N', 'E'})
    _cell(nav, north, {'S'})
    start = Pose2D(*_mid(branch, 'S'), 0.4)
    horizon = ActionHorizon(MotionPlanner())
    prims, terminal, seq, state = horizon.compile_with_state(
        nav, start, (parent, branch))
    assert terminal == (branch, east)
    assert seq == [branch, north, branch, east]
    assert prims[-1].kind == 'STOP'

    executor = MotionExecutor(start)
    executor.set_plan(prims)
    last_motion = prims[-2]
    for _ in range(2000):
        if executor.queue[0] is last_motion and last_motion.progress > 0:
            break
        executor.step(0.02)
    else:
        pytest.fail('did not reach the started tail motion before STOP')
    prior = [(p, p.progress, p.v_end) for p in executor.queue]
    old_state = deepcopy(state)

    # The east cell becomes mature while the car is still moving. The suffix
    # must inherit the virtual completion of north, rather than visit it again.
    _cell(nav, east, {'W'})
    suffix, new_terminal, new_seq, new_state = horizon.compile_with_state(
        nav, prims[-1].start_pose, terminal, state)
    assert state == old_state
    assert new_seq[:3] == [east, branch, parent]
    assert new_terminal == (branch, parent)
    assert new_state[branch]['done'] == {north, east}
    assert not executor.extend_plan(suffix)  # started prefix keeps its STOP
    assert [(p, p.progress, p.v_end) for p in executor.queue[:len(prior)]] == prior

    for _ in range(3000):
        if executor.idle:
            break
        executor.step(0.02)
    assert executor.idle
    assert executor.pose.yaw == pytest.approx(start.yaw)


def test_runtime_uses_new_map_before_motion_queue_becomes_idle(monkeypatch):
    import run_semantic_gate as gate
    from m3pro_nav.motion_executor import MotionExecutor

    original = MotionExecutor.extend_plan
    moving_extensions = []

    def record_extension(self, prims):
        if self.queue[0].kind != 'STOP':
            moving_extensions.append(tuple(p.kind for p in prims))
        return original(self, prims)

    monkeypatch.setattr(MotionExecutor, 'extend_plan', record_extension)
    result = gate.run_seed(11, False)
    assert moving_extensions
    assert gate.check(result, False) == []


def test_task_pruning_cannot_turn_a_walked_parent_edge_into_a_bounce():
    from m3pro_nav.task_pruning import prove_empty_dead_branch

    nav = StreamNav((0, 0), n=7, task_mode=True)
    cell, child, parent = (5, 2), (4, 2), (5, 1)
    _cell(nav, cell, {'S', 'W'})
    _cell(nav, parent, {'N'})
    nav.observe_blocks({parent: 'EMPTY'})
    assert prove_empty_dead_branch(nav, cell, parent)

    # Before traversing, skipping the known empty spur is legal. Once this
    # edge has been walked, it is the return route and must remain traversable.
    assert nav.resolve_next(child, cell) == child
    nav.traversal.mark_crossed(cell, 'S')
    assert nav.resolve_next(child, cell) == parent


def test_motion_contract_rejects_non_axis_aligned_straight():
    from m3pro_nav.motion_primitive import MotionPrimitive

    with pytest.raises(ValueError, match='axis'):
        MotionPrimitive(
            kind='STRAIGHT', start_pose=Pose2D(0.0, 0.0, 0.0),
            p0=(0.0, 0.0), p1=(0.1, 0.1), length=math.sqrt(0.02),
            v_max=0.2, v_end=0.0)


def test_compiled_template_chain_obeys_axis_and_standard_arc_contracts():
    nav = StreamNav((0, 0), n=7)
    cell, prev = (3, 3), (3, 2)
    _cell(nav, cell, {'S', 'E'})
    # For this corner, both open directions are known and the cell is complete;
    # the outgoing neighbor is chosen by the local graph policy.
    prims, _, _ = ActionHorizon(MotionPlanner()).compile(
        nav, Pose2D(*_mid(cell, 'S'), 0.0), (prev, cell))
    assert prims
    for p in prims:
        if p.kind == 'STRAIGHT':
            assert (abs(p.p0[0] - p.p1[0]) < 1e-9 or
                    abs(p.p0[1] - p.p1[1]) < 1e-9)
        elif p.kind == 'ARC':
            assert p.meta['r'] == pytest.approx(0.2)
            assert abs(p.yaw1) == pytest.approx(math.pi / 2)
    arc = next(p for p in prims if p.kind == 'ARC')
    start = (arc.p0[0] + arc.meta['r'] * math.cos(arc.yaw0),
             arc.p0[1] + arc.meta['r'] * math.sin(arc.yaw0))
    end_angle = arc.yaw0 + arc.yaw1
    end = (arc.p0[0] + arc.meta['r'] * math.cos(end_angle),
           arc.p0[1] + arc.meta['r'] * math.sin(end_angle))
    assert start == pytest.approx(_mid(cell, 'S'))
    assert end == pytest.approx(_mid(cell, 'E'))


def test_pose_mismatch_fails_instead_of_getting_a_connector():
    nav = StreamNav((0, 0), n=7)
    cell, prev = (3, 3), (3, 2)
    _cell(nav, cell, {'S', 'N'})
    wrong_pose = Pose2D(_mid(cell, 'S')[0] + 0.01,
                        _mid(cell, 'S')[1] + 0.01, 0.0)
    with pytest.raises(PlanGeometryMismatch):
        ActionHorizon(MotionPlanner()).compile(nav, wrong_pose, (prev, cell))


def test_primitive_execution_preserves_body_yaw_and_speed_bound():
    from m3pro_nav.motion_executor import MotionExecutor

    nav = StreamNav((0, 0), n=7)
    cell, prev = (3, 3), (3, 2)
    _cell(nav, cell, {'S', 'E'})
    yaw = 0.7
    pose = Pose2D(*_mid(cell, 'S'), yaw)
    executor = MotionExecutor(pose)
    executor.set_plan(ActionHorizon(MotionPlanner()).compile(nav, pose, (prev, cell))[0])
    max_step = 0.0
    for _ in range(3000):
        if executor.idle:
            break
        before = executor.pose.copy()
        after = executor.step(0.02)
        assert after.yaw == pytest.approx(yaw)
        max_step = max(max_step, math.hypot(after.x - before.x, after.y - before.y))
    assert max_step <= 0.7 * 0.02 + 1e-9


def test_grid_event_detector_only_reports_crossed_grid_edges():
    from m3pro_nav.event_detector import GridEventDetector

    detector = GridEventDetector(n=7)
    assert detector.detect(Pose2D(1.0, 1.0, 0.0), Pose2D(1.0, 1.0, 0.0)) == []
    events = detector.detect(Pose2D(0.9, 1.0, 0.0), Pose2D(1.7, 1.0, 0.0))
    assert [e.direction for e in events] == ['E', 'E']
    assert [(e.from_cell, e.to_cell) for e in events] == [((2, 2), (3, 2)),
                                                          ((3, 2), (4, 2))]


def test_home_compilation_requires_predecessor_context_for_exit_cell():
    nav = StreamNav((0, 0), n=7)
    planner = MotionPlanner()
    exit_cell = (6, 3)
    nav.edges.observe_open(exit_cell, 'E', 0.2)
    # A one-cell route has no incoming template context and must be rejected.
    with pytest.raises(PlanGeometryMismatch):
        planner.compile_home(Pose2D(*_center(exit_cell), 0.0),
                             (None, exit_cell), [exit_cell], 'E')


def test_home_one_cell_route_uses_preserved_predecessor_anchor():
    nav = StreamNav((0, 0), n=7)
    exit_cell, previous = (6, 3), (5, 3)
    nav.edges.observe_open(exit_cell, 'E', 0.2)
    prims = MotionPlanner().compile_home(
        Pose2D(*_mid(exit_cell, 'W'), 0.2),
        (previous, exit_cell), [exit_cell], 'E')
    assert prims[0].kind == 'STRAIGHT'
    assert prims[0].p0 == pytest.approx(_mid(exit_cell, 'W'))
    assert prims[0].p1 == pytest.approx(_mid(exit_cell, 'E'))
    assert prims[-1].meta.get('route_exit') is True


def test_exit_candidate_retracts_when_boundary_edge_vote_flips():
    nav = StreamNav((0, 0), n=7)
    exit_cell = (6, 3)
    for _ in range(2):
        nav.observe({}, [((6, 3), 'E', 0.2, 0.01)])
    assert exit_cell in nav.exit_cells()
    for _ in range(8):
        nav.observe({((6, 3), 'E'): (0.2, 0.01)}, [])
    assert exit_cell not in nav.exit_cells()


def test_block_visibility_requires_open_straight_corridor_and_confirms_empty():
    import runtime_v2

    walls = {(i, j): {'N', 'E', 'S', 'W'} for i in range(7) for j in range(7)}
    # A straight corridor from (2,2) east to (4,2); nearby north cell is a
    # turn-only target and the east ray stops at the first wall after (4,2).
    for x in (2, 3, 4):
        walls[(x, 2)].discard('E')
        walls[(x + 1, 2)].discard('W')
    walls[(2, 2)].discard('N')
    walls[(2, 3)].discard('S')
    walls[(2, 3)].discard('E')
    walls[(3, 3)].discard('W')
    world = runtime_v2.World(walls, {(4, 2), (3, 3)})
    obs = runtime_v2.block_observe_from(
        world, Pose2D(1.0, 1.0, 0.0), cam_range=0.8, cell_hint=(2, 2))
    assert obs[(3, 2)] == 'EMPTY'
    assert obs[(4, 2)] == 'BLOCK'
    assert obs[(2, 3)] == 'EMPTY'
    assert (3, 3) not in obs  # reachable only by turning north then east
    assert (5, 2) not in obs  # beyond configured range

    walled = {(i, j): {'N', 'E', 'S', 'W'} for i in range(7) for j in range(7)}
    walled[(2, 2)].discard('E')
    walled[(3, 2)].discard('W')
    blocked_obs = runtime_v2.block_observe_from(
        runtime_v2.World(walled, {(4, 2)}), Pose2D(1.0, 1.0, 0.0),
        cam_range=1.5, cell_hint=(2, 2))
    assert (4, 2) not in blocked_obs  # a wall blocks the ray despite range

    nav = StreamNav((0, 0), n=7, task_mode=True)
    nav.observe_blocks(obs)
    from m3pro_nav.block_map import EMPTY, BLOCK
    assert nav.block_map.state((3, 2)) == EMPTY
    assert nav.block_map.state((4, 2)) == BLOCK


def _confirm_corridor(nav, cells):
    """Confirm a straight parent-to-dead-end route and its unused edges."""
    from m3pro_nav.pose import DIRV
    path_edges = set(zip(cells, cells[1:]))
    for cell in cells:
        for d, dv in DIRV.items():
            nb = (cell[0] + dv[0], cell[1] + dv[1])
            if not (0 <= nb[0] < nav.n and 0 <= nb[1] < nav.n):
                continue
            if ((cell, nb) in path_edges or (nb, cell) in path_edges):
                nav.edges.observe_open(cell, d, 0.2)
                nav.edges.observe_open(cell, d, 0.2)
            else:
                nav.edges.observe_wall(cell, d, 0.2)
                nav.edges.observe_wall(cell, d, 0.2)


def test_task_pruning_requires_confirmed_empty_dead_branch_and_is_reversible():
    from m3pro_nav.block_map import BLOCK, EMPTY
    from m3pro_nav.task_pruning import prove_empty_dead_branch

    nav = StreamNav((0, 0), n=7, task_mode=True)
    parent, child, tip = (2, 2), (3, 2), (4, 2)
    _confirm_corridor(nav, [parent, child, tip])
    assert prove_empty_dead_branch(nav, parent, child) == []  # unknown blocks

    nav.observe_blocks({child: EMPTY, tip: EMPTY})
    assert prove_empty_dead_branch(nav, parent, child) == [child, tip]
    nav.observe_blocks({tip: BLOCK})
    assert prove_empty_dead_branch(nav, parent, child) == []  # proof retracts
    nav.observe_blocks({tip: EMPTY})
    assert prove_empty_dead_branch(nav, parent, child) == [child, tip]


def test_task_pruning_can_turn_back_mid_corridor_but_not_past_unknown_or_block():
    from m3pro_nav.block_map import BLOCK, EMPTY

    nav = StreamNav((0, 0), n=7, task_mode=True)
    previous, current, mid, tip = (1, 2), (2, 2), (3, 2), (4, 2)
    _confirm_corridor(nav, [previous, current, mid, tip])
    assert nav.resolve_next(previous, current) == mid  # block state unknown
    nav.observe_blocks({mid: BLOCK, tip: EMPTY})
    assert nav.resolve_next(previous, current) == mid  # BLOCK in skipped suffix
    nav.observe_blocks({mid: EMPTY})
    assert nav.resolve_next(previous, current) == previous  # now skip remaining suffix

    # The task proof is derived, so a later block observation restores the
    # branch immediately; it is never latched as completed.
    nav.observe_blocks({mid: BLOCK})
    assert nav.resolve_next(previous, current) == mid

    # If topology at the tip is not yet complete, a negative block observation
    # alone cannot prove that the corridor ends there.
    nav.observe_blocks({mid: EMPTY, tip: EMPTY})
    nav.edges.soft.pop(nav.edges.edge_key(tip, 'N'), None)
    assert nav.resolve_next(previous, current) == mid


def test_task_pruning_rejects_a_branched_suffix_even_when_empty():
    from m3pro_nav.block_map import EMPTY
    from m3pro_nav.task_pruning import prove_empty_dead_branch

    nav = StreamNav((0, 0), n=7, task_mode=True)
    current, branch = (2, 2), (2, 3)
    for _ in range(2):
        for direction in ('N', 'E', 'S', 'W'):
            observer = nav.edges.observe_wall if direction == 'W' else nav.edges.observe_open
            observer(branch, direction, 0.2)
    nav.observe_blocks({branch: EMPTY, (2, 4): EMPTY, (3, 3): EMPTY})
    assert prove_empty_dead_branch(nav, current, branch) == []


def test_collected_block_is_terminal_task_knowledge_not_an_active_block():
    from m3pro_nav.block_map import BLOCK, COLLECTED, UNKNOWN

    nav = StreamNav((0, 0), n=7, task_mode=True)
    cell = (2, 2)
    nav.observe_blocks({cell: BLOCK})
    assert nav.block_map.state(cell) == BLOCK
    nav.collect_block(cell)
    assert nav.block_map.state(cell) == COLLECTED
    assert not nav.has_block(cell)
    assert nav.block_map.is_confirmed_empty(cell)
    assert nav.block_map.state((3, 3)) == UNKNOWN


def test_task_pruning_does_not_write_physical_maps():
    nav = StreamNav((0, 0), n=7, task_mode=True)
    before_edges = (dict(nav.edges.soft), dict(nav.edges.hard), dict(nav.edges.derived))
    before_walked = set(nav.traversal.walked)
    nav.observe_blocks({(2, 2): 'EMPTY'})
    from m3pro_nav.task_pruning import prove_empty_dead_branch
    prove_empty_dead_branch(nav, (1, 2), (2, 2))
    assert (nav.edges.soft, nav.edges.hard, nav.edges.derived) == before_edges
    assert nav.traversal.walked == before_walked


def test_dependency_boundaries_are_structural():
    import inspect
    import runtime_v2
    from m3pro_nav.motion_primitive import KINDS
    from m3pro_nav.motion_planner import MotionPlanner

    runtime_inputs = inspect.signature(runtime_v2.explore).parameters
    assert 'ex' not in runtime_inputs  # truth exit belongs to the test driver
    assert runtime_inputs['required_blocks'].kind == inspect.Parameter.KEYWORD_ONLY

    package = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', 'ros2', 'm3pro_nav', 'm3pro_nav')

    def imported_modules(filename):
        tree = ast.parse(open(os.path.join(package, filename), encoding='utf-8').read())
        names = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                names.append(node.module)
        return names

    assert not any('block_map' in n for n in imported_modules('motion_planner.py'))
    assert not any('block_map' in n for n in imported_modules('tree_inference.py'))

    planner_src = open(os.path.join(package, 'motion_planner.py'), encoding='utf-8').read()
    assert 'nudge' not in planner_src.lower()
    assert 'resolve_exit' not in planner_src
    runtime_src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'sim', 'runtime_v2.py'),
                       encoding='utf-8').read()
    runtime_tree = ast.parse(runtime_src)
    for node in ast.walk(runtime_tree):
        # Check executable syntax, ignoring docstrings which may name forbidden
        # legacy concepts while explaining the migration boundary.
        assert not (isinstance(node, ast.Name) and
                    node.id in {'NUDGE', 'SPIN', 'SPIN90', 'CREEP', 'CREEP_OBSERVE'})
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert not (node.func.attr == 'clear' and
                        isinstance(node.func.value, ast.Name) and
                        node.func.value.id == 'executor')

    assert KINDS == ('STRAIGHT', 'ARC', 'STOP')   # REVERSE 是编译期宏 (两段 STRAIGHT)
    assert hasattr(MotionPlanner, 'template')
    assert not hasattr(MotionPlanner, 'compile_chain')  # no generic connector compiler
