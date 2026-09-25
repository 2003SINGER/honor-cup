# TODO —— 真相板（按 GPT 三审令重排, 2026-09-25）

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
