#!/usr/bin/env python3
"""G0a 核心单元回归 + G0b 集成回归 (规范 §14 Gate G0).

每个测试 = 一条语义的可执行定义. G0a+G0b 全绿前禁止随机种子 benchmark."""

import sys
import os
import inspect

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'ros2', 'm3pro_nav'))

import pytest
from m3pro_nav.edge_map import EdgeMap, TraversalMap, UNKNOWN, WALL, OPEN, T_CONFIRM, T_FLIP
from m3pro_nav import tree_inference
from m3pro_nav.cell_classifier import classify, transition, turn_type, DEAD, WAY, BRANCH
from m3pro_nav.dfs_explorer import DFSExplorer
from m3pro_nav import known_horizon


def _cell(walls, cell=(2, 2)):
    """构造四边全确认的格: walls=墙方向集合, 其余开口. 返回 (em, tr)."""
    em, tr = EdgeMap(7), TraversalMap(7)
    for d in ('N', 'E', 'S', 'W'):
        for _ in range(2):
            (em.observe_wall if d in walls else em.observe_open)(cell, d, 0.2)
    return em, tr


# ---------------- CellMark: degree 语义五类 ----------------

def test_cell_dead():
    em, tr = _cell({'N', 'E', 'S'})
    mark = classify(em, (2, 2), tr)
    assert mark['kind'] == DEAD and mark['degree'] == 1


def test_cell_straight():
    em, tr = _cell({'N', 'S'})
    mark = classify(em, (2, 2), tr)
    assert mark['kind'] == WAY and mark['degree'] == 2


def test_cell_left():
    em, tr = _cell({'N', 'E'})
    mark = classify(em, (2, 2), tr)
    assert mark['kind'] == WAY and transition(mark, 'S') == 'W'


def test_cell_right():
    em, tr = _cell({'N', 'W'})
    mark = classify(em, (2, 2), tr)
    assert transition(mark, 'S') == 'E'


def test_cell_tjunction():
    em, tr = _cell({'S'})
    mark = classify(em, (2, 2), tr)
    assert mark['kind'] == BRANCH and mark['degree'] == 3
    assert transition(mark, 'S') is None          # 岔路 transition=None, 归 DFS


def test_cell_cross():
    em, tr = _cell(set())
    mark = classify(em, (2, 2), tr)
    assert mark['kind'] == BRANCH and mark['degree'] == 4


def test_cell_incomplete_returns_none():
    assert classify(EdgeMap(7), (2, 2), TraversalMap(7)) is None


def test_dead_transition_returns_entry_side():
    """GPT 四审修复回归: DEAD 的 transition = 原路返回 (旧版返回 None)"""
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
    tr.mark_crossed((2, 2), 'E')                  # (2,2)E ↔ (3,2)W 同一条边
    assert em.state((2, 2), 'E', tr) == OPEN      # hard OPEN
    assert em.state((3, 2), 'W', tr) == OPEN      # 对面视角同状态


def test_walked_hard_beats_sensor_votes():
    em, tr = EdgeMap(7), TraversalMap(7)
    tr.mark_crossed((2, 2), 'E')
    for _ in range(10):
        em.observe_wall((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E', tr) == OPEN      # hard 事实不被投票推翻


def test_open_to_wall_removes_frontier():
    em, tr = EdgeMap(7), TraversalMap(7)
    for _ in range(2):
        em.observe_open((2, 2), 'E', 0.2)
    assert 'E' in em.frontier((2, 2), tr)
    for _ in range(T_FLIP + 1):
        em.observe_wall((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E', tr) == WALL
    assert 'E' not in em.frontier((2, 2), tr)     # 撤销 → frontier 自动消失


# ---------------- TreeInference: derived 可撤销 ----------------

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
    """GPT 四审核心: 前提 OPEN 撤销 → derived WALL 自动消失"""
    em, tr = _mk_u_channel()
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) == WALL
    for _ in range(T_FLIP + 2):                   # 推翻前提 (1,0)E → WALL
        em.observe_wall((1, 0), 'E', 0.2)
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) != WALL      # derived 不残留


# ---------------- DFSExplorer: BRANCH only + 持久 parent ----------------

def test_dfs_goes_deep_before_sibling():
    dx = DFSExplorer((0, 0), order='LFR')
    mark = {'kind': BRANCH, 'opens': ('N', 'E', 'W')}
    d2, mode = dx.commit_enter((0, 0), mark, 'S', lambda c, d: False, arrived_side='S')
    assert (d2, mode) == ('W', 'explore')         # LFR: L 最优先
    assert dx.stack[-1].parent_side == 'S'        # 冻结


def test_dfs_peek_is_pure():
    dx = DFSExplorer((0, 0), order='LFR')
    mark = {'kind': BRANCH, 'opens': ('N', 'E', 'W')}
    before = (len(dx.stack), [set(s.explored) for s in dx.stack])
    for _ in range(100):
        dx.peek_choice((0, 0), mark, 'S', lambda c, d: False)
    after = (len(dx.stack), [set(s.explored) for s in dx.stack])
    assert before == after                        # peek 100 次零副作用


def test_dfs_branch_parent_side_frozen_on_child_return():
    dx = DFSExplorer((0, 0), order='LFR')
    mark = {'kind': BRANCH, 'opens': ('N', 'E', 'W')}
    dx.commit_enter((0, 0), mark, 'S', lambda c, d: False, arrived_side='S')
    d2, mode = dx.commit_enter((0, 0), mark, 'S',      # parent 冻结传 'S'
                               lambda c, d: d == 'N', arrived_side='E')  # 实际从 E 回
    st = dx.stack[-1]
    assert st.parent_side == 'S'                  # 冻结, 不被 'E' 覆盖
    assert 'E' in st.explored                     # E 子树标记完成 (entry=E)
    assert d2 == 'W'                              # 转向未探 child W


def test_dfs_full_depth_trajectory():
    """完整深度轨迹: A→(way)→B→dead→B→sibling→(way back)→A→sibling, 逐步断言"""
    dx = DFSExplorer((0, 0), order='LFR')
    # A=(0,0) 从 S 进, opens N/E/W
    d2, mode = dx.commit_enter((0, 0), {'kind': BRANCH, 'opens': ('N', 'E', 'W')},
                               'S', lambda c, d: False, arrived_side='S')
    assert (d2, mode) == ('W', 'explore')         # A 选 L
    # way 格 (西邻) opens=(N,E): 从 E 进 → 唯一出口 N (transition 决定, 不经 DFS)
    assert transition({'kind': WAY, 'opens': ('N', 'E')}, 'E') == 'N'
    # B=(-1,1) 从 S 进, opens N/E
    d2, mode = dx.commit_enter((-1, 1), {'kind': BRANCH, 'opens': ('N', 'E')},
                               'S', lambda c, d: False, arrived_side='S')
    assert (d2, mode) == ('N', 'explore')         # B 选 E
    # dead end (E 邻格) → transition 原路返回 → 回 B
    assert transition({'kind': DEAD, 'opens': ('E',)}, 'E') == 'E'
    # B: entry=E (从 child 回), N 未探
    d2, mode = dx.commit_enter((-1, 1), {'kind': BRANCH, 'opens': ('N', 'E')},
                               'E', lambda c, d: d == 'E', arrived_side='E')
    assert (d2, mode) == ('N', 'explore')
    # N 子树探完回到 B: children 全 explored → 弹栈, 沿冻结 parent_side='S'
    d2, mode = dx.commit_enter((-1, 1), {'kind': BRANCH, 'opens': ('N', 'E')},
                               'S', lambda c, d: True, arrived_side='S')
    assert (d2, mode) == ('S', 'backtrack')
    # 沿 way 回到 A: way opens=(N,S) 从 N 进 → 出 S
    assert transition({'kind': WAY, 'opens': ('N', 'S')}, 'N') == 'S'
    # A: entry=W (从西边回来), N/W 已探 → sibling E
    d2, mode = dx.commit_enter((0, 0), {'kind': BRANCH, 'opens': ('N', 'E', 'W')},
                               'W', lambda c, d: d in ('N', 'W'), arrived_side='W')
    assert (d2, mode) == ('E', 'explore')


# ---------------- KnownHorizon ----------------

def test_known_horizon_straight_and_stops_at_incomplete():
    em, tr = EdgeMap(7), TraversalMap(7)
    em.observe_open((0, 0), 'N', 0.2); em.observe_open((0, 0), 'N', 0.2)
    em.observe_wall((0, 0), 'E', 0.2); em.observe_wall((0, 0), 'E', 0.2)
    em.observe_wall((0, 0), 'W', 0.2); em.observe_wall((0, 0), 'W', 0.2)
    em.set_boundary((0, 0), 'S', OPEN)            # 入口 (hard)
    h = known_horizon.horizon(em, tr, (0, 0), 'S',
                              lambda c: classify(em, c, tr), dfs_peek=None)
    assert h['cells'][0][1] == 'STRAIGHT'         # S→N
    assert h['stop'] == 'INCOMPLETE' and len(h['cells']) == 1


def test_known_horizon_left_then_incomplete():
    em, tr = EdgeMap(7), TraversalMap(7)
    # (0,0) 直行 N; (0,1) 左转 W (E/N 墙, S 来)
    for c, walls, opens in (((0, 0), ('E', 'W'), ('N', 'S')),
                            ((0, 1), ('E', 'N'), ('W', 'S'))):
        for d in walls:
            em.observe_wall(c, d, 0.2); em.observe_wall(c, d, 0.2)
        for d in opens:
            em.observe_open(c, d, 0.2); em.observe_open(c, d, 0.2)
    h = known_horizon.horizon(em, tr, (0, 0), 'S',
                              lambda c: classify(em, c, tr), dfs_peek=None)
    assert [t for _, t, _ in h['cells']] == ['STRAIGHT', 'LEFT']


def test_known_horizon_passes_through_complete_branch_via_peek():
    """COMPLETE branch 由 peek_choice 预览, horizon 穿过它继续"""
    em, tr = EdgeMap(7), TraversalMap(7)
    # (0,0) 十字全开 (branch), N→(0,1) WAY 直行→(0,2) 死路
    for d in ('N', 'E', 'S', 'W'):
        em.observe_open((0, 0), d, 0.2); em.observe_open((0, 0), d, 0.2)
    em.observe_open((0, 1), 'S', 0.2); em.observe_open((0, 1), 'S', 0.2)
    em.observe_open((0, 1), 'N', 0.2); em.observe_open((0, 1), 'N', 0.2)
    em.observe_wall((0, 1), 'E', 0.2); em.observe_wall((0, 1), 'E', 0.2)
    em.observe_wall((0, 1), 'W', 0.2); em.observe_wall((0, 1), 'W', 0.2)
    em.observe_wall((0, 2), 'S', 0.2); em.observe_wall((0, 2), 'S', 0.2)
    em.observe_wall((0, 2), 'E', 0.2); em.observe_wall((0, 2), 'E', 0.2)
    em.observe_wall((0, 2), 'W', 0.2); em.observe_wall((0, 2), 'W', 0.2)
    em.observe_wall((0, 2), 'N', 0.2); em.observe_wall((0, 2), 'N', 0.2)
    dx = DFSExplorer((0, 0), order='FLR')
    dx.commit_enter((0, 0), classify(em, (0, 0), tr), 'S',
                    lambda c, d: tr.is_walked(c, d))          # A: branch 进栈, 选 N
    h = known_horizon.horizon(em, tr, (0, 0), 'S',
                              lambda c: classify(em, c, tr),
                              dfs_peek=lambda c, m, e: dx.peek_choice(c, m, e,
                                  lambda cc, dd: tr.is_walked(cc, dd)))
    types = [t for _, t, _ in h['cells']]
    assert types[0] == 'STRAIGHT'                 # A: S→N
    assert types[1] == 'STRAIGHT'                 # (0,1) S→N (branch peek 直行)
    assert types[2] == 'DEAD'                     # (0,2) 死路
    assert h['stop'] == 'DEAD'


# ---------------- Tracker ----------------

def test_tracker_empty_path_safe_stop():
    from m3pro_nav.tracker import HolonomicTracker
    t = HolonomicTracker(v_max=0.35)
    vx, vy, wz, done = t.update((0.0, 0.0, 0.0), [])
    assert (vx, vy, wz) == (0.0, 0.0, 0.0) and done


def test_tracker_mutated_path_rebuilds_cache():
    from m3pro_nav.tracker import HolonomicTracker
    t = HolonomicTracker(v_max=0.35)
    path = [[0.0, 0.0], [0.0, 0.4], [0.4, 0.4]]
    t.update((0.0, 0.0, 0.0), path)
    path[1] = [0.0, 2.0]                          # 原地变异
    t.update((0.0, 0.0, 0.0), path)
    assert abs(t._total - 3.6492422502470641) < 1e-6


def test_tracker_single_point():
    from m3pro_nav.tracker import HolonomicTracker
    t = HolonomicTracker(v_max=0.35)
    vx, vy, wz, done = t.update((0.0, 0.0, 0.0), [[1.0, 1.0]])
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)


# ---------------- 单一真相源 ----------------

def test_no_duplicate_implementations():
    import m3pro_nav.mazemap as m1
    base = os.path.dirname(inspect.getfile(m1))
    for f in ('mazemap.py', 'tracker.py'):
        dup = os.path.normpath(os.path.join(base, '..', '..', '..', 'src', f))
        assert not os.path.exists(dup), f"双副本复活: {f}"


# ---------------- StreamNav 薄 coordinator (G0b) ----------------

def test_streamnav_has_no_private_belief_state():
    from m3pro_nav.stream_nav import StreamNav
    nav = StreamNav((0, 0), n=7)
    for legacy in ('beliefs', 'marks', 'm'):
        assert not hasattr(nav, legacy), f"StreamNav 残留第二套认知状态: {legacy}"


def test_streamnav_semanticsim_integration():
    """SemanticSim 实际运行链: observe → classify → plan_edge 全走新模块"""
    from m3pro_nav.stream_nav import StreamNav
    nav = StreamNav((0, 0), n=7, v_cruise=0.7)
    # 模拟: (0,0) N/E 开, W 墙, S 场外入口
    for _ in range(2):
        nav.observe({((0, 0), 'W'): (0.2, 0.01),
                     ((0, 1), 'S'): (0.2, 0.01)},
                    [((0, 0), 'N', 0.2, 0.01), ((0, 0), 'E', 0.2, 0.01),
                     ((0, 0), 'S', 0.2, 0.01)])
    nav.mark_walked((0, 0), 'N')                  # 车向北走过
    plan = nav.plan_edge((0, 0), 'N')
    assert plan['d2'] == 'E'                      # way: N 进 → 唯一另一口 E


# ---------------- R2.5.1 验收 (GPT 四审复核 9 项) ----------------

def test_derived_independent_of_previous_cache():
    """预塞错误 derived 后重算 → 结果只由 base facts 决定 (无自举)"""
    em, tr = _mk_u_channel()
    em.derived = {em.edge_key((5, 5), 'N'): 'WALL'}      # 垃圾缓存
    em.derived[em.edge_key((1, 2), 'E')] = 'OPEN'        # 错误翻转
    em.derived = tree_inference.recompute_derived(em, tr)
    assert em.state((1, 2), 'E', tr) == WALL             # 只由 base 推出
    assert em.state((5, 5), 'N', tr) == UNKNOWN          # 垃圾不残留


def test_exit_candidate_retracts():
    """出口不是旁路真相: 误 OPEN 翻 WALL → exit 候选自动消失"""
    from m3pro_nav.stream_nav import StreamNav
    nav = StreamNav((0, 0), n=7)
    # 远处格 (6,3) 东边界: 先误 OPEN (近距 2 帧 → confirmed)
    for _ in range(2):
        nav.observe({}, [((6, 3), 'E', 0.2, 0.01)])
    assert (6, 3) in nav.exit_cells()
    # 反复墙证据越过 T_FLIP → 翻 WALL
    for _ in range(6):
        nav.observe({((6, 3), 'E'): (0.2, 0.01)}, [])
    assert (6, 3) not in nav.exit_cells()                # 无 stale exit


def test_no_nearest_frontier_fallback():
    """README 宣称 nearest_frontier 已废除 → 探索 fallback 不得存在"""
    from m3pro_nav.stream_nav import StreamNav
    assert not hasattr(StreamNav, '_route_to_frontier')
    assert not hasattr(StreamNav, 'nearest_frontier')


def test_no_fake_assume_tree_param():
    """assume_tree 虚假可配置项已删 (树公理恒成立)"""
    from m3pro_nav.stream_nav import StreamNav
    import inspect as _insp
    assert 'assume_tree' not in _insp.signature(StreamNav.__init__).parameters
    import m3pro_nav.tree_inference as ti
    assert 'assume_tree' not in _insp.signature(ti.recompute_derived).parameters


def test_move_intent_contract():
    """MoveIntent 只含拓扑意图, 无执行细节"""
    from m3pro_nav.move_intent import MoveIntent
    mi = MoveIntent(next_edge='E', mode='EXPLORE', target_cell=(1, 0),
                    preferred_continuation='N', requires_stop=False)
    assert mi.next_edge == 'E' and mi.mode == 'EXPLORE'
    assert not hasattr(mi, 'end_o') and not hasattr(mi, 'v_end')
    assert not hasattr(mi, 'turn_here') and not hasattr(mi, 'far_cut')


def test_coordinator_full_dfs_event_sequence():
    """完整 DFS 轨迹全走 StreamNav 公共 API (observe+crossed+commit_cell+plan_intent):
    A branch → way → B branch → dead → B → B sibling → dead → B → way → A → A sibling"""
    from m3pro_nav.stream_nav import StreamNav

    nav = StreamNav((0, 0), n=7, order='LFR')

    def set_cell(c, walls):
        for _ in range(2):
            for d in ('N', 'E', 'S', 'W'):
                if d in walls:
                    nav.edges.observe_wall(c, d, 0.2)
                else:
                    nav.edges.observe_open(c, d, 0.2)

    set_cell((0, 0), {'W'})                  # A: opens N,E,S(boundary entry)
    set_cell((0, 1), {'E', 'W'})             # way: opens S,N
    set_cell((0, 2), {'W'})                  # B: opens S,E,N
    set_cell((1, 2), {'N', 'E', 'S'})        # dead: opens W
    set_cell((0, 3), {'N', 'E', 'W'})        # dead: opens S
    # A 是 branch (opens N,E + S 场外) → commit
    r = nav.commit_cell((0, 0), 'N')
    assert r is not None and r[0] == 'N'     # LFR: F(N) 无 L 可选 → N? opens 无 W/L → N=F 优先
    assert nav.dfs.stack[-1].parent_side == 'S'
    # → way (0,1): S 进 N 出
    nav.crossed((0, 0), 'N')
    it = nav.plan_intent((0, 1), 'N')
    assert isinstance(it, object) and it.next_edge == 'N' and it.mode == 'EXPLORE'
    # → B (0,2): branch, commit (children=N,E; heading N → L 无, F=N 优先于 R=E)
    nav.crossed((0, 1), 'N')
    r = nav.commit_cell((0, 2), 'N')
    assert r[0] == 'N' and r[1] == 'explore'
    assert nav.dfs.stack[-1].parent_side == 'S'
    # → dead (0,3): 原路返回 S
    nav.crossed((0, 2), 'N')
    it = nav.plan_intent((0, 3), 'N')
    assert it.next_edge == 'S' and it.mode == 'BACKTRACK'
    # 回 B: entry=N ∈ children → explored; 未探 E
    nav.crossed((0, 3), 'S')
    r = nav.commit_cell((0, 2), 'S')
    assert r[0] == 'E' and r[1] == 'explore'
    assert 'N' in nav.dfs.stack[-1].explored
    # → dead (1,2): 原路返回 W
    nav.crossed((0, 2), 'E')
    it = nav.plan_intent((1, 2), 'E')
    assert it.next_edge == 'W' and it.mode == 'BACKTRACK'
    # 回 B: entry=E ∈ children → explored; children 全完成 → 弹栈沿冻结 parent_side=S
    nav.crossed((1, 2), 'W')
    r = nav.commit_cell((0, 2), 'W')
    assert r == ('S', 'backtrack')
    assert nav.dfs.stack[-1].cell == (0, 0)  # 栈顶回到 A
    # → way (0,1) 重访: S 出
    nav.crossed((0, 2), 'S')
    it = nav.plan_intent((0, 1), 'S')
    assert it.next_edge == 'S'
    # 回 A: N ∈ children → explored; 未探 E
    nav.crossed((0, 1), 'S')
    r = nav.commit_cell((0, 0), 'S')
    assert r[0] == 'E' and r[1] == 'explore'
    assert 'N' in nav.dfs.stack[-1].explored


def test_sim_commits_dfs():
    """SemanticSim 真实进格路径必须调用 commit_cell (静态断言)"""
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'sim', 'maze_sim.py')).read()
    assert src.count('nav.commit_cell(') >= 3, \
        f"sim 真实进格事件未接 DFS commit: {src.count('nav.commit_cell(')} 处"
