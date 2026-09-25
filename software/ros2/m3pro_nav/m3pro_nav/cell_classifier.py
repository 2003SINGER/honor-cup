#!/usr/bin/env python3
"""CellClassifier —— CellMark 生成 (规范 §5).

CellMark = 格子静态几何事实, 含"怎么走": complete/degree/kind/transition.
不关心事实来自 SENSOR/WALKED/TREE_INFERENCE.
transition 纯几何: DEAD→原路返回(entry_side); WAY→唯一另一开口;
BRANCH→None (岔路由 DFSExplorer 决策). 是否离场 (出口 policy) 不属本层."""

DIRS = ('N', 'E', 'S', 'W')
DEAD, WAY, BRANCH = 'DEAD', 'WAY', 'BRANCH'


def classify(edges, cell, traversal=None):
    """四边全部确定 → CellMark dict; 否则 None (INCOMPLETE)."""
    edges_state = {d: edges.state(cell, d, traversal) for d in DIRS}
    if any(s == 'UNKNOWN' for s in edges_state.values()):
        return None
    openings = tuple(d for d in DIRS if edges_state[d] == 'OPEN')
    degree = len(openings)
    kind = DEAD if degree <= 1 else (WAY if degree == 2 else BRANCH)
    return {'edges': edges_state, 'complete': True, 'degree': degree,
            'kind': kind, 'opens': openings}


def transition(mark, entry_side):
    """非岔路的运动解析 (纯几何):
    DEAD + entry_side open → 原路返回 entry_side (GPT 四审修复: 旧版返回 None)
    WAY → 唯一另一开口
    BRANCH → None"""
    if not mark or mark['kind'] == BRANCH:
        return None
    if mark['kind'] == DEAD:
        return entry_side if entry_side in mark['opens'] else None
    outs = [d for d in mark['opens'] if d != entry_side]
    return outs[0] if len(outs) == 1 else None


def turn_type(entry_side, exit_side):
    """S→N=STRAIGHT, S→W=LEFT, S→E=RIGHT, S→S=BACK (GPT 四审: 旧版恒 STRAIGHT)"""
    heading = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[entry_side]  # 行进方向
    if exit_side == heading:
        return 'STRAIGHT'
    if exit_side == entry_side:
        return 'BACK'
    left_of = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[heading]
    return 'LEFT' if exit_side == left_of else 'RIGHT'
