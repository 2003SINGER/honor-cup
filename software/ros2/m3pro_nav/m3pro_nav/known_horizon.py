#!/usr/bin/env python3
"""KnownHorizonPlanner —— 前方已知范围 (规范 §7).

沿 MoveIntent 链展开: way 用 transition, COMPLETE branch 用 DFS peek_choice
(纯函数零副作用), 直到第一个 INCOMPLETE/DEAD/不可预览.
输出 cells 序列 (含 STRAIGHT/LEFT/RIGHT/BACK 类型) 与 d_remain."""

DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
C = 0.4


def horizon(edges, traversal, cell, entry_side, classify_fn, dfs_peek, max_cells=12):
    """dfs_peek(branch_cell, mark, entry_side, walked_fn) -> (d2, mode)|None 纯函数.
    返回 {'cells': [(cell, type, exit)], 'stop': ..., 'd_remain': 米}"""
    cells = []
    cur, came = cell, entry_side
    for _ in range(max_cells):
        mark = classify_fn(cur)
        if mark is None:
            return {'cells': cells, 'stop': 'INCOMPLETE', 'd_remain': len(cells) * C}
        if mark['kind'] == 'BRANCH':
            pk = dfs_peek(cur, mark, came) if dfs_peek else None
            if pk is None:
                return {'cells': cells, 'stop': 'BRANCH_NO_PEEK', 'd_remain': len(cells) * C}
            d2, mode = pk
            if mode == 'backtrack':
                from .cell_classifier import turn_type
                cells.append((cur, 'BACK', d2))
                return {'cells': cells, 'stop': 'BACKTRACK', 'd_remain': len(cells) * C}
            from .cell_classifier import turn_type
            cells.append((cur, turn_type(came, d2), d2))
            cur, came = (cur[0] + DIRV[d2][0], cur[1] + DIRV[d2][1]), OPP[d2]
            continue
        if mark['kind'] == 'DEAD':
            from .cell_classifier import turn_type
            cells.append((cur, 'DEAD', came))
            return {'cells': cells, 'stop': 'DEAD', 'd_remain': len(cells) * C}
        # WAY
        outs = [d for d in mark['opens'] if d != came]
        if len(outs) != 1:
            return {'cells': cells, 'stop': 'AMBIGUOUS', 'd_remain': len(cells) * C}
        d2 = outs[0]
        from .cell_classifier import turn_type
        cells.append((cur, turn_type(came, d2), d2))
        nxt = (cur[0] + DIRV[d2][0], cur[1] + DIRV[d2][1])
        came, cur = OPP[d2], nxt
    return {'cells': cells, 'stop': 'MAX', 'd_remain': len(cells) * C}
