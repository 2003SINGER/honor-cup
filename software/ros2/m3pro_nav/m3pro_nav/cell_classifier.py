#!/usr/bin/env python3
"""CellClassifier —— CellMark 生成 (规范 §5).

CellMark = 格子的静态几何事实, 包含"怎么走"的信息:
  complete / degree / kind(DEAD|WAY|BRANCH) / transition(entry→exit).
不关心事实来自 SENSOR/WALKED/TREE_INFERENCE —— CellClassifier 只看最终状态."""

DIRS = ('N', 'E', 'S', 'W')
DEAD, WAY, BRANCH = 'DEAD', 'WAY', 'BRANCH'


def classify(edges, cell):
    """四边全部确定 → CellMark dict; 否则 None (INCOMPLETE)."""
    edges_state = {d: edges.state(cell, d) for d in DIRS}
    if any(s == 'UNKNOWN' for s in edges_state.values()):
        return None
    openings = tuple(d for d in DIRS if edges_state[d] == 'OPEN')
    degree = len(openings)
    kind = DEAD if degree <= 1 else (WAY if degree == 2 else BRANCH)
    return {'edges': edges_state, 'complete': True, 'degree': degree,
            'kind': kind, 'opens': openings}


def transition(mark, entry_side, boundary_exits=()):
    """非岔路的运动解析: 已知从 entry_side 进入 → 唯一另一开口.
    DEAD → 原路返回; WAY → 另一开口; BRANCH → None (岔路由 DFSExplorer 决策).
    boundary_exits: 场外开口方向集合 (出口/入口), 不作为普通出口候选."""
    if not mark or mark['kind'] == BRANCH:
        return None
    outs = [d for d in mark['opens']
            if d != entry_side and d not in boundary_exits]
    return outs[0] if len(outs) == 1 else None
