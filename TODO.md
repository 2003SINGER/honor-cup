# TODO —— 真相板（按 GPT 三审令重排, 2026-09-25）

## ✅ P0-R2.5.1 Semantic Closure (2026-09-25, GPT 四审复核 9 项)

- [x] sim 真实进格事件接 commit_cell (grab/straight/far_cut 三点; far_cut 只 commit farc 不预登记)
- [x] _route_to_frontier 探索 fallback 删除 (测试断言复活即 FAIL)
- [x] exit 旁路真相删除: exit_cells() 从 boundary 边 effective OPEN 派生, 误 OPEN 翻 WALL 自动消失
- [x] TreeInference 纯 base 重算 (base_state/base_is_open/base_resolved; derived 垃圾预塞回归测试)
- [x] assume_tree 虚假参数删除 (树公理恒成立)
- [x] MoveIntent dataclass (move_intent.py); 旧 plan dict 降级 deprecated 适配层 (R3 删)
- [x] 35/35 G0a+G0b 全绿

接线过程修的连锁回归 (全部有 spy 现场定位):
- far_cut 启动后决策块重规划冻结 cut → 决策/速度调度加 cut is None 门控
- _backtrack_step None 泄漏 → 'home'/'wait'/d2 三值语义
- commit_enter 增加 arrived_side (parent_side 冻结 ≠ 本次来向标记 explored)
- 迟分类 branch: _pending[cell]=首访真实 parent_side, 栈空回访补 commit (parent 不从重访来向推)

### 🔴 已定位未修 (下一个一刀)
plan_edge 适配层 cell_classified(far) fallback 的 d3=_choose(f_far) 没过 is_boundary(far,d3) 检查
→ 出口格 far_cut 选边界方向 → walk_edge 越界 → cell=(3,-1) 永久 wait (seed3 实测根因)。
修复 = fallback 加 boundary guard。修完重跑 10 种子冒烟再谈 30 种子。

### 🔴 R3 范围残留 (数字继续不作数)
far_cut 双 walk (离散超前物理一格) / cut 完成位姿接缝跳变 / seed5 162k violation 碰撞风暴 /
三套位姿账本收敛为 Pose 唯一 owner。

## 本轮范围（GPT 指令：第一轮只做 R0+R1+R2, 停在 R3 前）

### ✅ P0-R0 文档与契约冻结 (本轮)
- [x] docs/design/算法规范.md —— 现行算法唯一规范 (15 章 + ownership 表 + Gates)
- [x] 旧方案移入 docs/decisions/
- [x] README 重写 (高层 + 一句话算法 + 数据流 + Gate)
- [x] TODO 重排为本结构; 明确 state owner 唯一答案

### ✅ P0-R1 Core deterministic model (本轮)
- [x] edge_map.py: EdgeMap (WALL/OPEN/UNKNOWN + provenance) + TraversalMap (walked) 分离
- [x] tree_inference.py: 规则 A 成环必墙 (规则 B/C/D 挂 assume_tree flag)
- [x] cell_classifier.py: CellMark (complete/degree/kind/transition)
- [x] dfs_explorer.py: 只管 BRANCH 调度 (stack), nearest_frontier 探索策略废除
- [x] known_horizon.py: 沿意图链展开到第一个 INCOMPLETE
- [ ] StreamNav 收敛为薄协调层 (委托上述模块) —— 部分完成: marks/beliefs 已委托

### ✅ P0-R2 EdgeBelief (本轮)
- [x] hysteresis 真语义: WALL 只在 score≤−T_FLIP 翻 OPEN, 反之亦然 (修复通用分支吃掉 T_FLIP)
- [x] canonical key 序列化修复 (普通边 k[2] 越界 bug)
- [x] provenance 字段
- [x] deterministic tests (tests/test_core.py, 21 例)

### 🔴 P0-R3 Motion execution layer (下一轮, 未经 R0-R2 审查不得开工)
- [ ] Pose2D 唯一权威位姿 (SensorSim/碰撞/可视化只读它)
- [ ] MotionPrimitive 契约: STRAIGHT/CUT90/SPIN90/TURN180/CREEP_OBSERVE/STOP
      (preconditions/commit_point/cancel_deadline/progress/done/events)
- [ ] crossed-edge 事件驱动 TraversalMap (废除 sim 手工 walk_edge 预登记)
- [ ] cut 完成只产生 A→B 一个 crossed_edge (修复超前一整格的双 walk)
- [ ] 迟到 CellMark: 已过 commit point → 禁止 cut → 制动 STOP/SPIN90

### 🔴 P0-R4 CREEP_OBSERVE / Known Horizon 接入速度规划
### 🔴 P0-R5 失败 seed 固化 regression fixture → 30 → 100 → 1000 seeds
      (watchdog/越界 = FAILED, 禁止 +50 计费与 finish() 兜底)

## P1 — Sim→Real 中间层 (R3 后)
bringup+health_check / TF / base estimator / GridLocalizer(墙吸附, 防自证循环) /
EdgeObserver(scan→edge association, ABSTAIN 语义) / MotionPlanner+Tracker /
SafetySupervisor(任何 stale→停车) / block detector+CollectorAdapter(统一接口)
完整链 = launch→传感器健康→TF→pose→墙修正→EdgeObservation→StreamNav→MoveIntent→
MotionPlanner→Executor→Tracker→SafetySupervisor→cmd_vel→方块→Collector

## P2 — 速度/cut/剪枝收益/参数优化 (G5 之前禁止投入主要精力)

## 历史已完成
- ✅ 09-25 一审修复: degree 语义/入射角置信/arc 计时作废/单一认知体
- ✅ 09-25 二审修复: P0-A 单一真相(删双副本)/P0-C 部分可撤销/P0-D 碰撞检查引入
- ✅ 09-25 三审 R0+R1+R2 (本轮, 见上)
