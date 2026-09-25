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

- **G0 核心确定性测试**进行中：EdgeMap/TraversalMap/TreeInference/CellClassifier/
  DFSExplorer/KnownHorizon 已实现，deterministic pytest 已建（`software/tests/`）
- **未解决的最大问题**：R3 运动执行层（Pose 唯一 owner + crossed-edge 事件）未建，
  此前仿真的 far==cell/位姿漂移/148k violation 均源于此（详见 TODO 断点记录）
- 验收 Gate：G0 core tests → G1 SemanticSim 1000 seeds 0 fail → G2 SensorSim →
  G3 replay → G4 台架 → G5 一格 → G6 小迷宫 → G7 全程。**G5 之前不优化速度。**

## 仓库结构

```
software/ros2/m3pro_nav/m3pro_nav/   唯一核心实现 (单一真相源)
  ├─ edge_map.py        EdgeMap + TraversalMap (几何事实 ⊥ 行驶历史)
  ├─ tree_inference.py  树结构公理闭包 (成环必墙; B/C/D 挂 flag)
  ├─ cell_classifier.py CellMark / CellClassifier
  ├─ dfs_explorer.py    BRANCH 调度 (nearest_frontier 探索策略已废除)
  ├─ known_horizon.py   KnownHorizonPlanner
  ├─ stream_nav.py      决策协调层 (beliefs/mark/plan_edge)
  ├─ mazemap.py         拓扑 + path_between (RoutePlanner)
  └─ tracker.py         全向轨迹跟踪 (上车件, 未闭环)
software/sim/maze_sim.py    Tier A SemanticSim (World 出 EdgeObservation)
software/tests/             deterministic regression suite (G0)
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
