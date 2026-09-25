#!/usr/bin/env python3
"""TaskPruning —— 任务层空死枝证明 (GPT 定稿: 隔离于 TreeInference 与运动层).

TaskPruning 推"物理路存在但任务上不用走";
TreeInference 推物理墙 (成环必墙). 二者绝对不混:
  - 本模块绝不写 EdgeMap / TraversalMap / BlockMap;
  - 不产生任何轨迹, 不碰 MotionPlanner;
  - 剪枝状态动态派生 (每次选择时重新证明), 方块新观测自动使证明失效 => 自动恢复.

证明规则 (GPT 门 P1-P4):
  CellMark 未 COMPLETE            -> 不可剪
  走廊任一格 BLOCK_UNKNOWN/BLOCK  -> 不可剪
  全程 EMPTY_CONFIRMED/COLLECTED
  且走廊终点 DEAD (无前向 OPEN)    -> 可剪
  途中遇岔路                       -> 不可剪 (v1 不做递归子树证明)

返回值 = 被证明走廊的格列表 (空列表 = 不可剪), 供审计 (false_prune 对账)."""


def prove_empty_dead_branch(nav, parent, child):
    """从 parent 看向 child: 该支路是否为已确认无方块的死走廊.
    返回走廊格列表 (含 child, 不含 parent); 空列表 = 不可剪."""
    corridor = []
    prev, cur = parent, child
    while True:
        if not nav.cell_classified(cur):
            return []                      # P1: 标记未完成
        if not nav.block_map.is_confirmed_empty(cur):
            return []                      # P2/P3: BLOCK_PRESENT / UNKNOWN
        corridor.append(cur)
        nbrs = nav.open_neighbors(cur)
        nbrs.discard(prev)
        if not nbrs:
            return corridor                # P4: 终点 DEAD, 全程已确认空
        if len(nbrs) > 1:
            return []                      # v1: 遇岔路, 无法确认更深
        prev, cur = cur, next(iter(nbrs))
