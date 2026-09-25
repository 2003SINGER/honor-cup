#!/usr/bin/env python3
"""KnownHorizonPlanner —— 前方已知范围 (规范 §7).

从当前位置沿 CellMark 链展开, 直到第一个 INCOMPLETE 或 BRANCH (需决策) /
DEAD (运动终点). Known Horizon 内 = 已知赛道, 速度按最坏刹车距离反推.
速度规划: v_max ≤ √(2·a_dec·d_remain) —— 调用方用 d_remain 控制."""

DIRV = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
C = 0.4


def horizon(edges, cell, entry_side, classify_fn, max_cells=12):
    """沿 way/直行链展开 (branch/dead/incomplete 为边界).
    返回 {'cells': [(cell,kind)], 'stop': 'INCOMPLETE'|'BRANCH'|'DEAD',
          'd_remain': 到停止决策点的格数×C}"""
    cells = []
    cur, came = cell, entry_side
    for _ in range(max_cells):
        mark = classify_fn(cur)
        if mark is None:
            return {'cells': cells, 'stop': 'INCOMPLETE',
                    'd_remain': len(cells) * C}
        if mark['kind'] == 'BRANCH':
            return {'cells': cells, 'stop': 'BRANCH', 'd_remain': len(cells) * C}
        if mark['kind'] == 'DEAD':
            cells.append((cur, 'DEAD'))
            return {'cells': cells, 'stop': 'DEAD', 'd_remain': (len(cells)) * C}
        outs = [d for d in mark['opens'] if d != came]
        if len(outs) != 1:
            return {'cells': cells, 'stop': 'AMBIGUOUS', 'd_remain': len(cells) * C}
        d2 = outs[0]
        cells.append((cur, f'STRAIGHT/{d2}' if d2 == _heading_of(came, d2) else f'TURN/{d2}'))
        nxt = (cur[0] + DIRV[d2][0], cur[1] + DIRV[d2][1])
        came, cur = OPP[d2], nxt
    return {'cells': cells, 'stop': 'MAX', 'd_remain': len(cells) * C}


def _heading_of(came, d2):
    """辅助: 进入方向 came 的反向 = 行进 heading → 转向类型由调用方细化"""
    return OPP[came] if d2 == OPP[came] else d2
