# 英才杯 A 题 —— 7×7 树形迷宫探索与方块收集

## 一句话算法

> 系统通过雷达、已有墙地图和树结构约束，持续把车辆前方的 UNKNOWN cells 转化为
> COMPLETE CellMarks；COMPLETE 区域立即按已知路径执行；DFS 只负责 BRANCH 的分支
> 顺序；车辆速度由 Known Horizon 与运动 commit deadline 动态决定。
> （规范全文：[docs/design/算法规范.md](docs/design/算法规范.md)）

## 硬件与比赛约束

- Yahboom M3 Pro（麦克纳姆轮）+ Jetson Orin NX + 2×YDLIDAR T-mini Plus（360°，±20mm）+
  DCW2 双目结构光相机（装于机械臂 arm4）+ 机械臂夹爪
- 迷宫 7×7 树形，格距 0.4m，通道 0.4m，车宽约束 21.5cm
- 报名截止 2026-09-30；作品提交 2026-11-09
- 官方资料 134 份 PDF 已全量通读：`docs/官方资料总览.md`

## 系统数据流（目标架构）

```
odom+IMU → Pose 预测 → 用已确认墙做 GridLocalizer 修正 (防自证循环)
LaserScan → EdgeObserver → EdgeObservation
       → EdgeMap(WALL/OPEN/UNKNOWN + provenance)
       → TreeInference (成环必墙等公理闭包)
       → CellClassifier → CellMark (COMPLETE 即"怎么走"已知)
       → DFSExplorer (仅 BRANCH 分支调度) + KnownHorizonPlanner (速度)
       → MoveIntent → MotionPlanner → MotionExecutor(唯一权威 Pose)
       → Tracker → SafetySupervisor → /cmd_vel
Camera → BlockDetector → CollectorAdapter → 机械臂
```

## 当前阶段（2026-09-25）

- **R3 事件驱动运行时已落地**：Pose2D 唯一物理 owner + 几何跨越事件 +
  MotionPlanner/Executor + CellVisit 迟分类；10/10 seeds 0 碰撞 0 未确认 0 中断
- 下一步：KnownHorizon 调速 (P1-A) → 1000 seeds (P1-B) → Sim→Real 中间层 (P1-C)
- 验收 Gate：G0 core tests → G1 SemanticSim 1000 seeds 0 fail → G2 SensorSim →
  G3 replay → G4 台架 → G5 一格 → G6 小迷宫 → G7 全程。**G5 之前不优化速度。**

## 仓库结构

```
software/ros2/m3pro_nav/m3pro_nav/   核心实现 (单一真相源)
  ├─ edge_map.py        EdgeMap (hard/soft/derived 三层) + TraversalMap 分离
  ├─ tree_inference.py  树公理闭包 (成环必墙, 纯 base 重算可撤销)
  ├─ cell_classifier.py CellMark / CellClassifier / turn_type
  ├─ dfs_explorer.py    BRANCH 调度 (BranchState parent 冻结 + peek/commit 两阶段)
  ├─ known_horizon.py   KnownHorizonPlanner
  ├─ move_intent.py     MoveIntent (StreamNav 唯一输出契约)
  ├─ pose.py            Pose2D/Twist2D (唯一物理真相的数据类型)
  ├─ events.py          CrossedEdge / EnteredCell 事件
  ├─ visits.py          CellVisit (entered_from 只来自真实进入事件)
  ├─ event_detector.py  连续线段 → 跨格事件 (0漏/0重/0幻)
  ├─ motion_primitive.py / motion_planner.py / motion_executor.py
  │                     STRAIGHT/CUT90/SPIN/CREEP/STOP (只改 Pose, 不碰认知)
  ├─ stream_nav.py      薄 coordinator: 事件→认知, plan_intent→MoveIntent
  ├─ mazemap.py         path_between (RoutePlanner)
  └─ tracker.py         全向轨迹跟踪 (上车件, 未闭环)
software/sim/runtime_v2.py  R3 事件驱动 SemanticSim (Pose 唯一 owner)
software/sim/maze_sim.py    gen_maze/CLI/定位对照 (旧运行时已删)
software/tests/             G0a+G0b+R3 Gates 41 例
docs/design/算法规范.md      现行算法唯一规范
docs/decisions/             历史/被否定方案 (arc -58% 已作废等)
docs/官方资料总览.md         134 份官方 PDF 摘要
TODO.md                     唯一进度真相板 (P0-R0~R5 / P1 / P2)
```

## 阅读顺序

算法规范 → TODO（当前断点）→ tests（语义的可执行定义）→ stream_nav → maze_sim。

## 状态标注约定

"SemanticSim 已验证" ≠ "实车已验证"。性能数字必须带 Tier 级别与 failures 计数，
failures>0 即 INVALID。
