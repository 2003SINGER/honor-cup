# TODO —— 真相板 (2026-09-25, R3 Clean-Room 完成)

> 唯一进度真相. 完成项打 ✅ 留证据; 数字必须带 Gate 级别, failures>0 即 INVALID.

## 当前状态一句话

R3 事件驱动运行时 (runtime_v2) Clean-Room 落地: Pose2D 唯一物理真相 +
CrossedEdge/EnteredCell 几何事件 + MotionPlanner/Executor + CellVisit 迟分类。
10/10 seeds 全信息模式: 0 exception / 0 越界 / 0 watchdog / **0 collision** /
0 unresolved / 认知地图与真值一致。G0-G3 (结构/事件/连续/CUT90) deterministic 全绿。

## 本轮完成 (R2.5.2 + R3)

- ✅ GPT 五审两刀: _pending 提前 pop bug (由 CellVisit.entered_from 结构性取代);
  deprecated plan_edge 自行发明 continuation 的 fallback 整体删除 (适配层已死)
- ✅ Pose2D/Twist2D 唯一物理 owner (MotionExecutor); World 只剩静态环境
- ✅ GridEventDetector: 连续线段真实穿格线 → CrossedEdge (0 漏/0 重/0 幻;
  大 dt 多跨按序全出) —— o>=0.4 簿记判定废除
- ✅ CellVisit: entered_from 只来自 EnteredCellEvent; 迟分类 branch parent 永真
- ✅ MotionPrimitive 契约 (STRAIGHT/CUT90/SPIN/CREEP_OBSERVE/STOP + preconditions/
  commit_point/cancel_deadline); primitive 只改 Pose, 不碰认知
- ✅ CUT90 重定义: 直行跨 A/B 恰一次 → cut 段零事件 → 后续直行跨 B/C 恰一次
  (双 walk/离散超前/接缝跳变一次消灭); 切点 0.10 (弦距内角 0.212 > 车对角 0.1855)
- ✅ 旧运行时死亡: explore_stream / walk_edge / plan_edge 适配层 / mark_walked /
  三套位姿账本全部删除 (结构测试断言复活即 FAIL)
- ✅ tests: G0a+G0b+R3 Gates = 41/41 全绿
- ✅ 出口语义修正: 角格 side 优先级 (E>S) 曾致返航冲墙; home_route 派生出口方向

## 下一步 (按序, 禁跳)

### P1-A KnownHorizon 接入速度规划 (R4)
远格未分类即停车重规划 (当前行为, 安全但慢 ~97s) → 沿意图链展开已知段全速。
Creep/commit-line 取消语义已有, 补 mid-primitive 重规划条件。

### P1-B SemanticSim 100 → 1000 seeds
10 seeds 冒烟已过; 1000/1000 全绿前不恢复性能统计。含块模式 (8 块) 全流程
(相机视野模型已在 sensors; 抓取取放策略未接)。

### P1-C Sim→Real 中间层 (G4-G7)
bringup+health_check / TF / base estimator / GridLocalizer (已确认墙吸附, 防自证) /
EdgeObserver (scan→edge, ABSTAIN 语义) / decision_node 接 runtime_v2 同链 /
SafetySupervisor (stale→停车) / CollectorAdapter。
Gate: G3 rosbag replay → G4 台架 → G5 一格 → G6 小迷宫 → G7 全程。

### P2 — 速度/cut/剪枝收益 (G7 前禁止投入)

## 已知小账
- st['mark_hit'] 统计未接 runtime_v2 (CLI 显示 0%), 仅观测指标, 不影响控制
- 方块收集策略 = has_block(far) 停车即抓, 无路径优化 (P1-B 一并)

## 历史已完成
- ✅ 09-25 一审: degree 语义/入射角置信/arc 计时作废/单一认知体
- ✅ 09-25 二审: 单一真相(删双副本)/碰撞检查引入
- ✅ 09-25 三审 R0+R1+R2: 算法规范/五纯逻辑模块/真迟滞
- ✅ 09-25 四审 R2.5: StreamNav 薄化/peek-commit 两阶段/derived 重算
- ✅ 09-25 五审 R2.5.1+R3: 事件驱动运行时 clean-room (本轮)
