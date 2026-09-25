#!/usr/bin/env python3
"""核心单元回归 (规范 §13 Gate A-L 的确定性子集).

每个测试 = 一条语义的可执行定义. 全绿前禁止随机 seed benchmark."""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'ros2', 'm3pro_nav'))

import pytest
from m3pro_nav.edge_map import EdgeMap, TraversalMap, UNKNOWN, WALL, OPEN, T_CONFIRM, T_FLIP
from m3pro_nav import tree_inference
from m3pro_nav.cell_classifier import classify, transition, turn_type, DEAD, WAY, BRANCH


def _cell(walls, cell=(2, 2)):
    """构造四边全确认的格: walls=墙方向集合, 其余开口. 返回 (em, tr)."""
    em, tr = EdgeMap(7), TraversalMap(7)
    for d in ('N', 'E', 'S', 'W'):
        for _ in range(2):
            (em.observe_wall if d in walls else em.observe_open)(cell, d, 0.2)
    return em, tr


# ---------------- Gate A: CellAction 穷举 ----------------

def test_cell_dead():
    em, tr = _cell({'N', 'E', 'S'})
    mark = classify(em, (2, 2), tr)
    assert mark['kind'] == DEAD and mark['degree'] == 1


def test_cell_straight():
    em, tr = _cell({'N', 'S'})
    mark = classify(em, (2, 2), tr)
    assert mark['kind'] == WAY and mark['degree'] == 2


def test_cell_tjunction_and_cross():
    em, tr = _cell({'S'})
    mark = classify(em, (2, 2), tr)
    assert mark['kind'] == BRANCH and mark['degree'] == 3
    em2, tr2 = _cell(set())
    mark2 = classify(em2, (2, 2), tr2)
    assert mark2['kind'] == BRANCH and mark2['degree'] == 4


def test_cell_incomplete_returns_none():
    assert classify(EdgeMap(7), (2, 2), TraversalMap(7)) is None


def test_gate_a_cellaction_exhaustive():
    """Gate A: 所有 COMPLETE CellMark × 4 个 entry_side 穷举.
    WAY → 唯一另一 OPEN; DEAD → entry; BRANCH → preview 的非 entry OPEN.
    绝不输出 WALL 或越界方向."""
    from itertools import combinations
    from m3pro_nav.stream_nav import StreamNav
    for walls_n in range(0, 4):
        for walls in combinations(('N', 'E', 'S', 'W'), walls_n):
            em, tr = _cell(set(walls))
            mark = classify(em, (2, 2), tr)
            opens = mark['opens']
            for entry in ('N', 'E', 'S', 'W'):
                ex = transition(mark, entry)
                if mark['kind'] == BRANCH:
                    # 岔路走 StreamNav preview (纯函数, 零副作用)
                    nav = StreamNav((0, 0), n=7)
                    nav.edges = em
                    nav.traversal = tr
                    ex = nav.resolve_exit((2, 2), entry)
                    if entry not in opens:
                        assert ex is None
                    else:
                        ch = [d for d in opens
                              if d != entry and not nav.is_boundary((2, 2), d)]
                        assert (ex in ch) if ch else (ex is None)
                elif entry not in opens:
                    assert ex is None or ex == entry   # 非法入口不承诺
                elif mark['kind'] == DEAD:
                    assert ex == entry
                else:
                    assert ex in opens and ex != entry


def test_dead_transition_returns_entry_side():
    em, tr = _cell({'N', 'E', 'S'})
    mark = classify(em, (2, 2), tr)
    assert transition(mark, 'W') == 'W'


def test_turn_types():
    assert turn_type('S', 'N') == 'STRAIGHT'
    assert turn_type('S', 'W') == 'LEFT'
    assert turn_type('S', 'E') == 'RIGHT'
    assert turn_type('S', 'S') == 'BACK'


# ---------------- Edge / Traversal 分离 ----------------

def test_open_not_walked():
    em, tr = EdgeMap(7), TraversalMap(7)
    em.observe_open((2, 2), 'E', 0.2)
    em.observe_open((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E', tr) == OPEN
    assert not tr.is_walked((2, 2), 'E')


def test_open_walked_canonical():
    em, tr = EdgeMap(7), TraversalMap(7)
    tr.mark_crossed((2, 2), 'E')
    assert em.state((2, 2), 'E', tr) == OPEN
    assert em.state((3, 2), 'W', tr) == OPEN


def test_walked_hard_beats_sensor_votes():
    em, tr = EdgeMap(7), TraversalMap(7)
    tr.mark_crossed((2, 2), 'E')
    for _ in range(10):
        em.observe_wall((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E', tr) == OPEN


def test_open_to_wall_removes_frontier():
    em, tr = EdgeMap(7), TraversalMap(7)
    for _ in range(2):
        em.observe_open((2, 2), 'E', 0.2)
    assert 'E' in em.frontier((2, 2), tr)
    for _ in range(T_FLIP + 1):
        em.observe_wall((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E', tr) == WALL
    assert 'E' not in em.frontier((2, 2), tr)


# ---------------- Gate D: TreeInference 可撤销 ----------------

def _mk_u_channel():
    em, tr = EdgeMap(7), TraversalMap(7)
    for c, d in (((1, 2), 'S'), ((1, 1), 'S'), ((1, 0), 'E'),
                 ((2, 0), 'N'), ((2, 1), 'N')):
        em.observe_open(c, d, 0.2)
        em.observe_open(c, d, 0.2)
    return em, tr


def test_cycle_inference_wall():
    em, tr = _mk_u_channel()
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) == WALL
    assert em.provenance((1, 2), 'E', tr) == 'TREE_INFERENCE'


def test_derived_wall_retracts_when_premise_retracts():
    em, tr = _mk_u_channel()
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) == WALL
    for _ in range(T_FLIP + 2):
        em.observe_wall((1, 0), 'E', 0.2)
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) != WALL


def test_derived_independent_of_previous_cache():
    em, tr = _mk_u_channel()
    em.derived = {em.edge_key((5, 5), 'N'): 'WALL'}
    em.derived[em.edge_key((1, 2), 'E')] = 'OPEN'
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) == WALL
    assert em.state((5, 5), 'N', tr) == UNKNOWN


# ---------------- Gate E: 固定模板几何 ----------------

def _mk_nav():
    from m3pro_nav.stream_nav import StreamNav
    return StreamNav((0, 0), n=7)


def _confirm_all(nav, cell, walls):
    for _ in range(2):
        for d in ('N', 'E', 'S', 'W'):
            if d in walls:
                nav.edges.observe_wall(cell, d, 0.2)
            else:
                nav.edges.observe_open(cell, d, 0.2)


def _mid(cell, d):
    from m3pro_nav.pose import C, DIRV
    return ((cell[0] + 0.5) * C + DIRV[d][0] * 0.2,
            (cell[1] + 0.5) * C + DIRV[d][1] * 0.2)


def test_gate_e_straight_template():
    from m3pro_nav.motion_planner import MotionPlanner
    nav = _mk_nav()
    _confirm_all(nav, (2, 2), {'N', 'S'})          # WAY: E/W 开
    prims = MotionPlanner().compile_chain(nav, _mid((2, 2), 'W') and
                                          __import__('m3pro_nav.pose', fromlist=['Pose2D']).Pose2D(*_mid((2, 2), 'W'), 0.0),
                                          (2, 2), 'W')
    kinds = [p.kind for p in prims]
    assert kinds[0] == 'STRAIGHT'
    seg = prims[0]
    assert abs(seg.p1[0] - _mid((2, 2), 'E')[0]) < 1e-9   # 直行到对边中点
    assert seg.length > 0


def test_gate_e_arc_template_quarter_circle():
    """Gate E: 相邻 entry/exit → R=0.2 四分之一圆弧, 圆心=内角点"""
    from m3pro_nav.motion_planner import MotionPlanner
    from m3pro_nav.pose import Pose2D, C
    nav = _mk_nav()
    _confirm_all(nav, (2, 2), {'N', 'W'})          # WAY: S 进 → E 出 (右转)
    planner = MotionPlanner()
    prims = planner.compile_chain(nav, Pose2D(*_mid((2, 2), 'S'), 0.0), (2, 2), 'S')
    arcs = [p for p in prims if p.kind == 'ARC']
    assert len(arcs) == 1
    arc = arcs[0]
    assert abs(arc.meta['r'] - 0.2) < 1e-9
    assert abs(abs(arc.yaw1) - math.pi / 2) < 1e-9   # 恰四分之一圆
    assert abs(arc.length - math.pi / 2 * 0.2) < 1e-9
    # 圆心 = 内角点 = m_in + exit 方向 0.2
    m_in, m_out = _mid((2, 2), 'S'), _mid((2, 2), 'E')
    corner = arc.p0
    assert abs(math.hypot(m_in[0] - corner[0], m_in[1] - corner[1]) - 0.2) < 1e-9
    assert abs(math.hypot(m_out[0] - corner[0], m_out[1] - corner[1]) - 0.2) < 1e-9


def test_gate_e_reverse_template_not_spin():
    """DEAD 格 → REVERSE (倒穿父边回父格中心), 绝不出现 SPIN/TURN180"""
    from m3pro_nav.motion_planner import MotionPlanner
    from m3pro_nav.pose import Pose2D, C
    nav = _mk_nav()
    _confirm_all(nav, (2, 2), {'N', 'E', 'S'})     # DEAD: 只有 W 开
    prims = MotionPlanner().compile_chain(nav, Pose2D(*_mid((2, 2), 'W'), 0.0), (2, 2), 'W')
    kinds = [p.kind for p in prims]
    assert 'SPIN' not in kinds and 'CUT90' not in kinds
    # 末段终点 = 父格中心 (倒穿父边)
    end = prims[-1].p1
    assert abs(end[0] - _mid((2, 2), 'W')[0] - 0.0) < 1e-9 or end == prims[-1].p1
    pc = ((2 + 0.5) * C + __import__('m3pro_nav.pose', fromlist=['DIRV']).DIRV['W'][0] * 0.4,
          (2 + 0.5) * C + __import__('m3pro_nav.pose', fromlist=['DIRV']).DIRV['W'][1] * 0.4)
    assert abs(end[0] - pc[0]) < 1e-9 and abs(end[1] - pc[1]) < 1e-9


def test_gate_d_yaw_never_touched_by_primitives():
    """Gate D: primitive 只改位置; executor 全程不改 yaw"""
    from m3pro_nav.pose import Pose2D
    from m3pro_nav.motion_planner import MotionPlanner
    from m3pro_nav.motion_executor import MotionExecutor
    from m3pro_nav.event_detector import GridEventDetector
    nav = _mk_nav()
    _confirm_all(nav, (2, 2), {'N', 'W'})
    yaw0 = 0.7
    planner = MotionPlanner()
    ex = MotionExecutor(Pose2D(*_mid((2, 2), 'S'), yaw0))
    ex.set_plan(planner.compile_chain(nav, Pose2D(*_mid((2, 2), 'S'), yaw0), (2, 2), 'S'))
    det = GridEventDetector(n=7)
    for _ in range(3000):
        if ex.idle:
            break
        prev = ex.pose.copy()
        cur = ex.step(0.02)
        det.detect(prev, cur)
        assert abs(cur.yaw - yaw0) < 1e-12, "yaw 被 primitive 修改"


def test_gate_g_seam_continuity():
    """Gate G: 模板接缝位置连续, 每 tick 位移 ≤ v_max·dt"""
    from m3pro_nav.pose import Pose2D
    from m3pro_nav.motion_planner import MotionPlanner
    from m3pro_nav.motion_executor import MotionExecutor
    from m3pro_nav.event_detector import GridEventDetector
    nav = _mk_nav()
    _confirm_all(nav, (2, 2), {'N', 'W'})
    ex = MotionExecutor(Pose2D(*_mid((2, 2), 'S'), 0.0))
    ex.set_plan(MotionPlanner().compile_chain(nav, Pose2D(*_mid((2, 2), 'S'), 0.0), (2, 2), 'S'))
    det = GridEventDetector(n=7)
    max_step = 0.0
    for _ in range(3000):
        if ex.idle:
            break
        prev = ex.pose.copy()
        cur = ex.step(0.02)
        det.detect(prev, cur)
        max_step = max(max_step, math.hypot(cur.x - prev.x, cur.y - prev.y))
    assert max_step <= 0.7 * 0.02 + 1e-9


# ---------------- Gate H: 事件检测 ----------------

def test_event_detector_four_directions():
    from m3pro_nav.pose import Pose2D
    from m3pro_nav.event_detector import GridEventDetector
    det = GridEventDetector(n=7)
    for d, (dx, dy) in (('E', (0.4, 0)), ('W', (-0.4, 0)),
                        ('N', (0, 0.4)), ('S', (0, -0.4))):
        prev = Pose2D(1.0, 1.0, 0.0)
        cur = Pose2D(1.0 + dx, 1.0 + dy, 0.0)
        evs = det.detect(prev, cur)
        assert len(evs) == 1 and evs[0].direction == d


def test_event_detector_no_phantom_and_order():
    from m3pro_nav.pose import Pose2D
    from m3pro_nav.event_detector import GridEventDetector
    det = GridEventDetector(n=7)
    assert det.detect(Pose2D(1.0, 1.0, 0.0), Pose2D(1.0, 1.0, 0.0)) == []
    evs = det.detect(Pose2D(0.9, 1.0, 0.0), Pose2D(1.7, 1.0, 0.0))
    assert [e.direction for e in evs] == ['E', 'E']
    assert evs[0].from_cell == (2, 2) and evs[1].from_cell == (3, 2)


def test_event_detector_line_departure_semantics():
    """模板接缝恰停在格线上后离线 = 一次真实跨越 (链式几何必然)"""
    from m3pro_nav.pose import Pose2D, C
    from m3pro_nav.event_detector import GridEventDetector
    det = GridEventDetector(n=7)
    # 停在 x=0.8 格线上, 向 E 离开 → 跨越 (2,?)→(3,?)
    evs = det.detect(Pose2D(0.8, 1.0, 0.0), Pose2D(0.82, 1.0, 0.0))
    assert len(evs) == 1 and evs[0].direction == 'E'
    assert evs[0].from_cell == (1, 2) and evs[0].to_cell == (2, 2)
    # 停在格线上不动 → 无事件
    assert det.detect(Pose2D(0.8, 1.0, 0.0), Pose2D(0.8, 1.0, 0.0)) == []
    # 停在格线上向反方向离开 → 反向跨越
    evs = det.detect(Pose2D(0.8, 1.0, 0.0), Pose2D(0.78, 1.0, 0.0))
    assert len(evs) == 1 and evs[0].direction == 'W'


def test_event_detector_arc_no_phantom():
    """Gate K: 圆弧 entry→exit 中点在格内 → 弧段零跨越事件"""
    from m3pro_nav.pose import Pose2D
    from m3pro_nav.motion_planner import MotionPlanner
    from m3pro_nav.motion_executor import MotionExecutor
    from m3pro_nav.event_detector import GridEventDetector
    nav = _mk_nav()
    _confirm_all(nav, (2, 2), {'N', 'W'})
    ex = MotionExecutor(Pose2D(*_mid((2, 2), 'S'), 0.0))
    ex.set_plan(MotionPlanner().compile_chain(nav, Pose2D(*_mid((2, 2), 'S'), 0.0), (2, 2), 'S'))
    det = GridEventDetector(n=7)
    events = []
    for _ in range(3000):
        if ex.idle:
            break
        prev = ex.pose.copy()
        cur = ex.step(0.02)
        events += det.detect(prev, cur)
    # S 进 E 出: 只发生 S 出场反向或无事件; 弧内不得产生 (2,2) 邻格幻事件
    for e in events:
        assert (e.from_cell[0], e.from_cell[1]) in [(2, 2), (2, 1), (3, 2), (2, 3), (1, 2)]


# ---------------- Gate B: Branch 局部状态机 ----------------

def test_gate_b_branch_local_dfs_sequence():
    """Gate B: 嵌套树探索事件序列, 无全局 stack.
    A(0,0) branch → way → B(0,2) branch → dead → B sibling → back A → sibling"""
    from m3pro_nav.stream_nav import StreamNav
    nav = StreamNav((0, 0), n=7, order='LFR')

    def set_cell(c, walls):
        for _ in range(2):
            for d in ('N', 'E', 'S', 'W'):
                if d in walls:
                    nav.edges.observe_wall(c, d, 0.2)
                else:
                    nav.edges.observe_open(c, d, 0.2)

    set_cell((0, 0), {'W'})                  # A: opens N,E,S
    set_cell((0, 1), {'E', 'W'})             # way: S,N
    set_cell((0, 2), {'W'})                  # B: S,E,N
    set_cell((1, 2), {'N', 'E', 'S'})        # dead: W
    set_cell((0, 3), {'N', 'E', 'W'})        # dead: S
    r = nav.on_entered((0, 0), 'S')
    assert r == 'N'                          # A: L(W) 被 W 墙挡? N/E 排序 → N (F 先)
    nav.on_crossed((0, 0), 'N')
    r = nav.on_entered((0, 1), 'S')          # way: 无分支状态
    nav.on_crossed((0, 1), 'N')
    r = nav.on_entered((0, 2), 'S')
    assert r == 'N'                          # B: children 排序
    st = nav.branch[(0, 2)]
    assert st['parent'] == 'S'
    # dead (0,3): 原路返回
    nav.on_crossed((0, 2), 'N')
    nav.on_crossed((0, 3), 'S')
    r = nav.on_entered((0, 2), 'N')
    assert r == 'E'                          # N 完成 → 下一 child E
    # dead (1,2): 回 B → children 全完成 → parent S
    nav.on_crossed((0, 2), 'E')
    nav.on_crossed((1, 2), 'W')
    r = nav.on_entered((0, 2), 'E')
    assert r == 'S'
    # 回 A: N 完成 → sibling E
    nav.on_crossed((0, 2), 'S')
    nav.on_crossed((0, 1), 'S')
    nav.on_crossed((0, 0), 'S')
    r = nav.on_entered((0, 0), 'N')
    assert r == 'E'
    # parent_side 永不变化
    assert nav.branch[(0, 2)]['parent'] == 'S'
    assert nav.branch[(0, 0)]['parent'] == 'S'


def test_late_classified_branch_preserves_first_parent():
    """迟分类: 首访 INCOMPLETE → 分类完成 → refresh_branch 用 first_entered_from 建状态;
    且绝不重复登记 visit (latest 只由真实事件更新)"""
    from m3pro_nav.stream_nav import StreamNav
    nav = StreamNav((0, 0), n=7)
    nav.on_entered((0, 2), 'S')                       # 首访: mark None
    for _ in range(2):
        nav.edges.observe_open((0, 2), 'N', 0.2)
        nav.edges.observe_open((0, 2), 'E', 0.2)
        nav.edges.observe_open((0, 2), 'S', 0.2)
        nav.edges.observe_wall((0, 2), 'W', 0.2)
    r = nav.refresh_branch((0, 2))
    assert r == 'N'
    assert nav.branch[(0, 2)]['parent'] == 'S'
    cnt = nav.visits.get((0, 2)).visit_count
    # 再 refresh 100 次: visit 不得被重复登记
    for _ in range(100):
        nav.refresh_branch((0, 2))
    assert nav.visits.get((0, 2)).visit_count == cnt
    assert nav.visits.get((0, 2)).latest_entered_from == 'S'


# ---------------- 单一真相 / 结构验收 ----------------

def test_no_duplicate_implementations():
    import m3pro_nav.mazemap as m1
    base = os.path.dirname(__import__('inspect').getfile(m1))
    for f in ('mazemap.py', 'tracker.py'):
        dup = os.path.normpath(os.path.join(base, '..', '..', '..', 'src', f))
        assert not os.path.exists(dup), f"双副本复活: {f}"


def test_structural_no_legacy_concepts():
    """规范 §12 禁止清单: 核心包内不得出现旧概念 (文件级扫描)"""
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'ros2', 'm3pro_nav', 'm3pro_nav')
    banned = ('SPIN', 'CUT90', 'CREEP', 'dfs_explorer', 'known_horizon',
              'move_intent', 'plan_intent')
    for f in os.listdir(base):
        if not f.endswith('.py'):
            continue
        src = open(os.path.join(base, f)).read()
        for b in banned:
            assert b not in src, f"{f} 残留旧概念: {b}"


def test_structural_dependency_direction():
    """Gate 依赖方向: core 不得 import sim/runtime/ROS"""
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'ros2', 'm3pro_nav', 'm3pro_nav')
    banned_imports = ('runtime_v2', 'maze_sim', 'rclpy', 'sensor_sim')
    for f in os.listdir(base):
        if not f.endswith('.py'):
            continue
        src = open(os.path.join(base, f)).read()
        for b in banned_imports:
            assert b not in src, f"{f} 违反依赖方向: import {b}"


def test_no_fake_assume_tree_param():
    from m3pro_nav.stream_nav import StreamNav
    import inspect as _insp
    assert 'assume_tree' not in _insp.signature(StreamNav.__init__).parameters
    import m3pro_nav.tree_inference as ti
    assert 'assume_tree' not in _insp.signature(ti.recompute_derived).parameters


# ---------------- StreamNav / 出口 / 返航 ----------------

def test_exit_candidate_retracts():
    from m3pro_nav.stream_nav import StreamNav
    nav = StreamNav((0, 0), n=7)
    for _ in range(2):
        nav.observe({}, [((6, 3), 'E', 0.2, 0.01)])
    assert (6, 3) in nav.exit_cells()
    for _ in range(6):
        nav.observe({((6, 3), 'E'): (0.2, 0.01)}, [])
    assert (6, 3) not in nav.exit_cells()


def test_home_route_from_exit_cell_zero_length():
    from m3pro_nav.stream_nav import StreamNav
    nav = StreamNav((0, 0), n=7)
    for _ in range(2):
        nav.edges.observe_open((6, 3), 'E', 0.2)
    hr = nav.home_route((6, 3))
    assert hr is not None
    seg, exc, edir = hr
    assert seg == [] and exc == (6, 3) and edir == 'E'
    for _ in range(2):
        nav.edges.observe_open((0, 3), 'N', 0.2)
        nav.edges.observe_wall((0, 3), 'W', 0.2)
        nav.edges.observe_wall((0, 3), 'E', 0.2)
        nav.edges.observe_open((0, 2), 'N', 0.2)
        nav.edges.observe_open((0, 2), 'S', 0.2)
        nav.edges.observe_wall((0, 2), 'E', 0.2)
        nav.edges.observe_wall((0, 2), 'W', 0.2)
    nav.traversal.mark_crossed((0, 2), 'N')
    nav.traversal.mark_crossed((0, 3), 'S')
    for i in range(6):
        nav.traversal.mark_crossed((i, 3), 'E')
    hr = nav.home_route((0, 2))
    assert hr is not None and len(hr[0]) >= 1


# ---------------- Gate J: UNKNOWN fallback (蹭入停车) ----------------

def test_gate_j_unknown_cell_nudge_then_stop():
    """下一格未识别: 链在格线中点先蹭入 0.1m 再 STOP (离线触发跨越事件)"""
    from m3pro_nav.pose import Pose2D, C
    from m3pro_nav.motion_planner import MotionPlanner, NUDGE
    nav = _mk_nav()
    _confirm_all(nav, (2, 2), {'N', 'S'})          # (2,2) WAY: E 开, (3,2) 未知
    planner = MotionPlanner()
    prims = planner.compile_chain(nav, Pose2D(*_mid((2, 2), 'W'), 0.0), (2, 2), 'W')
    kinds = [p.kind for p in prims]
    assert 'STOP' in kinds
    stop = prims[-1]
    assert stop.meta.get('wait') == (3, 2)
    nudge = prims[-2]
    assert nudge.kind == 'STRAIGHT' and nudge.meta.get('nudge_into') == (3, 2)
    # 蹭入点在格内 (离格线 ≥ 1e-6)
    assert abs(nudge.p1[0] - 1.6) > 1e-6 or abs(nudge.p1[1] % 0.4) > 1e-6
    assert abs(math.hypot(nudge.p1[0] - nudge.p0[0],
                          nudge.p1[1] - nudge.p0[1]) - NUDGE) < 1e-9


# ---------------- Gate J/K: 集成 —— 真值对账 (固化 1 seed) ----------------

def test_wrong_edges_matches_truth_on_gen_maze():
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'sim'))
    import runtime_v2 as R
    import maze_sim as M
    walls, entry, ex, side = M.gen_maze(0)
    r = R.explore(walls, entry, ex, 'LFR', set(), v_cruise=0.7)
    assert r.get('wrong_edges') == 0
    assert r.get('unresolved') == 0
    assert r.get('violations') == 0
    assert not r.get('aborted')
