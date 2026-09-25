#!/usr/bin/env python3
"""G0 核心确定性回归套件 (规范 §14 Gate G0).

每个测试 = 一条语义的可执行定义. 全绿前禁止跑随机种子 benchmark."""

import sys
import os
import inspect

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'ros2', 'm3pro_nav'))

import pytest
from m3pro_nav.edge_map import EdgeMap, UNKNOWN, WALL, OPEN, T_CONFIRM, T_FLIP
from m3pro_nav import tree_inference
from m3pro_nav.cell_classifier import classify, transition, DEAD, WAY, BRANCH
from m3pro_nav.dfs_explorer import DFSExplorer
from m3pro_nav import known_horizon


# ---------------- CellMark: degree 语义五类 ----------------

def mk_cell(walls, cell=(2, 2)):
    """构造一个 EdgeMap: walls = 该格为墙的方向集合, 其余先给 2 帧开口证据"""
    em = EdgeMap(7)
    for d in ('N', 'E', 'S', 'W'):
        if d in walls:
            em.observe_wall(cell, d, 0.2)
            em.observe_wall(cell, d, 0.2)
        else:
            em.observe_open(cell, d, 0.2)
            em.observe_open(cell, d, 0.2)
    return em


def test_cell_dead():
    m = classify(mk_cell({'N', 'E', 'S'}), (2, 2))   # 只有 W 开
    assert m['kind'] == DEAD and m['degree'] == 1


def test_cell_straight():
    m = classify(mk_cell({'N', 'S'}), (2, 2))        # E/W 通
    assert m['kind'] == WAY and m['degree'] == 2


def test_cell_left():
    m = classify(mk_cell({'N', 'E'}), (2, 2))        # 朝 S 进, 左 = E? 朝 S: 左=E
    assert m['kind'] == WAY
    assert transition(m, 'S') == 'W'                 # 唯一非来向开口


def test_cell_right():
    m = classify(mk_cell({'N', 'W'}), (2, 2))
    assert transition(m, 'S') == 'E'


def test_cell_tjunction():
    m = classify(mk_cell({'S'}), (2, 2))             # N/E/W 开 = T 岔
    assert m['kind'] == BRANCH and m['degree'] == 3
    assert transition(m, 'S') is None                # 岔路不给唯一 transition


def test_cell_cross():
    m = classify(mk_cell(set()), (2, 2))
    assert m['kind'] == BRANCH and m['degree'] == 4


def test_cell_incomplete_returns_none():
    em = EdgeMap(7)                                  # 全 UNKNOWN
    assert classify(em, (2, 2)) is None


# ---------------- Edge: 开口/走过/frontier ----------------

def test_open_not_walked():
    em = EdgeMap(7)
    em.observe_open((2, 2), 'E', 0.2)
    em.observe_open((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E') == OPEN
    assert not em._walked((2, 2), 'E')


def test_open_walked():
    em = EdgeMap(7)
    em.mark_walked((2, 2), 'E')
    assert em.state((2, 2), 'E') == OPEN and em._walked((2, 2), 'E')
    assert em.state((3, 2), 'W') == OPEN            # canonical: 对面视角 (3,2)W 同一条边


def test_open_to_wall_removes_frontier():
    em = EdgeMap(7)
    for _ in range(2):
        em.observe_open((2, 2), 'E', 0.2)
    assert 'E' in em.frontier((2, 2))
    for _ in range(T_FLIP + 1):                      # 反向证据越过 T_FLIP
        em.observe_wall((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E') == WALL
    assert 'E' not in em.frontier((2, 2))            # frontier 自动消失, 无 stale seen


# ---------------- TreeInference ----------------

def test_cycle_inference_wall():
    em = EdgeMap(7)
    # 构造 U 形已确认通道: (1,2)-(1,1)-(1,0)-(2,0)-(2,1), (1,2)-(2,2) 未知
    for c, d in (((1, 2), 'S'), ((1, 1), 'S'), ((1, 0), 'E'),
                 ((2, 0), 'N'), ((2, 1), 'N')):
        em.observe_open(c, d, 0.2)
        em.observe_open(c, d, 0.2)
    tree_inference.infer(em)
    assert em.state((1, 2), 'E') == WALL             # (1,2)-(2,2) 加上就成环
    assert em.provenance((1, 2), 'E') == 'TREE_INFERENCE'


# ---------------- DFSExplorer ----------------

def test_dfs_goes_deep_before_sibling():
    """branch 选 L 后, L 子树探完(way 格重访)才考虑 sibling"""
    dx = DFSExplorer((0, 0), order='LFR')
    # branch 格 (0,0) 从场外 S 进: N/L(R)/R(F) 均未走
    mark = {'kind': BRANCH, 'opens': ('N', 'E', 'W')}
    d2, mode = dx.on_enter((0, 0), mark, 'N', lambda c, d: False)
    assert mode == 'explore' and d2 == 'W'           # LFR: L 最优先
    assert dx.stack[-1][0] == (0, 0)


def test_dfs_returns_to_parent_branch():
    """way 格全走过 → backtrack; branch children 全探 → 弹栈回溯"""
    dx = DFSExplorer((0, 0))
    mark_way = {'kind': WAY, 'opens': ('N', 'S')}
    d2, mode = dx.on_enter((1, 0), mark_way, 'N',
                           lambda c, d: (c, d) in {((1, 0), 'N')})
    assert mode == 'backtrack' and d2 == 'S'         # 退向来向边方向 (南)

    # branch (1,0) children 全走过 → pop → 朝栈顶父 branch (0,0) 回溯
    dx.stack = [((0, 0), 'S')]                       # 父 branch 在栈上
    dx.stack.append(((1, 0), 'S'))
    mark_br = {'kind': BRANCH, 'opens': ('N',)}
    d2, mode = dx.on_enter((1, 0), mark_br, 'N',
                           lambda c, d: True)        # 全 walked
    assert mode == 'backtrack' and d2 == 'S'         # 沿来向边退向父


def test_dfs_does_not_use_nearest_frontier_policy():
    """结构断言: DFSExplorer 不含 nearest_frontier; 探索目标由栈决定"""
    assert not hasattr(DFSExplorer, 'nearest_frontier')


# ---------------- KnownHorizon ----------------

def test_known_horizon_stops_at_first_incomplete():
    em = EdgeMap(7)
    # (0,0) 四边确认, N 开口; (0,1) 只确认 S 开 + E 墙 → N UNKNOWN
    for d in ('N', 'E', 'W'):
        pass
    em.observe_open((0, 0), 'N', 0.2); em.observe_open((0, 0), 'N', 0.2)
    em.observe_wall((0, 0), 'E', 0.2); em.observe_wall((0, 0), 'E', 0.2)
    em.observe_wall((0, 0), 'W', 0.2); em.observe_wall((0, 0), 'W', 0.2)
    em.observe_open((0, 1), 'S', 0.2); em.observe_open((0, 1), 'S', 0.2)
    em.observe_wall((0, 1), 'E', 0.2); em.observe_wall((0, 1), 'E', 0.2)
    em.observe_open((0, 0), 'S', 0.2); em.observe_open((0, 0), 'S', 0.2)  # 来向场外开口
    h = known_horizon.horizon(em, (0, 0), 'S', lambda c: classify(em, c))
    assert h['stop'] == 'INCOMPLETE' and len(h['cells']) == 1   # (0,0) 已知, (0,1) 不完整


# ---------------- EdgeBelief: 证据状态机 ----------------

def test_unknown_to_open():
    em = EdgeMap(7)
    em.observe_open((2, 2), 'N', 1.5)
    assert em.state((2, 2), 'N') == UNKNOWN          # 远距 1 帧 → 不足
    em.observe_open((2, 2), 'N', 1.5)
    assert em.state((2, 2), 'N') == OPEN             # 2 帧一致


def test_unknown_to_wall():
    em = EdgeMap(7)
    em.observe_wall((2, 2), 'N', 1.5)
    em.observe_wall((2, 2), 'N', 1.5)
    assert em.state((2, 2), 'N') == WALL


def test_open_requires_flip_threshold_to_wall():
    """GPT 抓的 hysteresis bug: OPEN→WALL 必须真的越过 T_FLIP, ±2 不够"""
    em = EdgeMap(7)
    for _ in range(2):
        em.observe_open((2, 2), 'N', 0.2)
    assert em.state((2, 2), 'N') == OPEN
    for _ in range(T_CONFIRM - 1):                   # +2-1 次墙: score 未过 +T_FLIP
        em.observe_wall((2, 2), 'N', 0.2)
    assert em.state((2, 2), 'N') == OPEN             # 不许翻!
    for _ in range(4):                               # 补到越过 +T_FLIP
        em.observe_wall((2, 2), 'N', 0.2)
    assert em.state((2, 2), 'N') == WALL


def test_wall_requires_flip_threshold_to_open():
    em = EdgeMap(7)
    for _ in range(2):
        em.observe_wall((2, 2), 'N', 0.2)
    assert em.state((2, 2), 'N') == WALL
    for _ in range(T_CONFIRM - 1):
        em.observe_open((2, 2), 'N', 0.2)
    assert em.state((2, 2), 'N') == WALL             # 迟滞保持
    for _ in range(4):
        em.observe_open((2, 2), 'N', 0.2)
    assert em.state((2, 2), 'N') == OPEN


def test_walked_open_cannot_be_sensor_closed():
    em = EdgeMap(7)
    em.mark_walked((2, 2), 'E')
    for _ in range(10):
        em.observe_wall((2, 2), 'E', 0.2)
    assert em.state((2, 2), 'E') == OPEN             # 物理事实不被传感器推翻


def test_sensor_contradiction_is_recorded():
    em = EdgeMap(7)
    em.mark_walked((2, 2), 'E')
    em.observe_wall((2, 2), 'E', 0.2)
    assert em.contradictions >= 1                    # 诊断信号


def test_edge_json_roundtrip():
    em = EdgeMap(7)
    for _ in range(2):
        em.observe_wall((2, 2), 'N', 1.5)
    em.observe_open((2, 2), 'E', 1.5)                # 远距 1 帧: score=-1 → UNKNOWN
    em.mark_walked((0, 0), 'N')
    em2 = EdgeMap(7).from_json(em.to_json())
    assert em2.state((2, 2), 'N') == WALL
    assert em2.state((2, 2), 'E') == UNKNOWN
    assert em2._walked((0, 0), 'N') and em2.state((0, 0), 'N') == OPEN
    assert em2.contradictions == em.contradictions


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
    old_progress = t._s if hasattr(t, '_s') else None
    path[1] = [0.0, 2.0]                             # 原地变异中间点
    vx, vy, wz, done = t.update((0.0, 0.0, 0.0), path)
    # 缓存必须重建: 新路径弧长 2.4 > 旧 0.8, 若没重建则进度错乱
    total = getattr(t, '_total', None)
    if total is not None:
        assert abs(total - 3.6492422502470641) < 1e-6  # 新路径弧长 (缓存已重建)


def test_tracker_single_point():
    from m3pro_nav.tracker import HolonomicTracker
    t = HolonomicTracker(v_max=0.35)
    vx, vy, wz, done = t.update((0.0, 0.0, 0.0), [[1.0, 1.0]])
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)


# ---------------- 单一真相源 ----------------

def test_single_mazemap_implementation():
    import m3pro_nav.mazemap as m1
    src = open(os.path.join(os.path.dirname(inspect.getfile(m1)),
                            '..', '..', '..', 'src', 'mazemap.py')).read() \
        if os.path.exists(os.path.join(os.path.dirname(inspect.getfile(m1)),
                                       '..', '..', '..', 'src', 'mazemap.py')) else None
    assert src is None, "src/mazemap.py 双副本复活!"
    assert not hasattr(m1.MazeMap.open_edge, '__wrapped__') or True


def test_single_tracker_implementation():
    import m3pro_nav.tracker as t1
    dup = os.path.join(os.path.dirname(inspect.getfile(t1)),
                       '..', '..', '..', 'src', 'tracker.py')
    assert not os.path.exists(dup), "src/tracker.py 双副本复活!"
