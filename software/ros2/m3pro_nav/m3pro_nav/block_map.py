#!/usr/bin/env python3
"""BlockMap —— 方块任务地图 (与物理地图彻底分离).

四态: UNKNOWN / EMPTY(已确认无方块, 负信息) / BLOCK(有未收方块) / COLLECTED.
EMPTY 是一等公民: 没有它就做不了"确认死路且无方块才剪枝".
物理地图 (EdgeMap/TraversalMap) 绝不被本模块写入."""

UNKNOWN = 'UNKNOWN'
EMPTY = 'EMPTY'
BLOCK = 'BLOCK'
COLLECTED = 'COLLECTED'


class BlockMap:
    def __init__(self):
        self.cells = {}          # cell -> EMPTY | BLOCK (未观测 = UNKNOWN)
        self.collected = set()   # 已收取方块格

    def observe(self, obs):
        """相机批量观测: obs = {cell: 'EMPTY' | 'BLOCK'}. 返回新 BLOCK 格列表."""
        newly = []
        for c, st in obs.items():
            if c in self.collected:
                continue
            cur = self.cells.get(c, UNKNOWN)
            if st == 'BLOCK' and cur != BLOCK:
                newly.append(c)
            self.cells[c] = st
        return newly

    def mark_collected(self, cell):
        self.collected.add(cell)

    def state(self, cell):
        if cell in self.collected:
            return COLLECTED
        return self.cells.get(cell, UNKNOWN)

    def has_uncollected_block(self, cell):
        return self.cells.get(cell) == BLOCK and cell not in self.collected

    def is_confirmed_empty(self, cell):
        """剪枝用: EMPTY 确认无方块; COLLECTED 也视作任务已了结 (GPT 定稿)."""
        return self.state(cell) in (EMPTY, COLLECTED)
