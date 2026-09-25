# 英才杯 A 题 —— 7×7 树形迷宫探索与方块收集

## 项目是什么

2026 英才杯 A 题迷宫机器人：Yahboom M3 Pro 麦轮底盘 + Jetson Orin NX +
2×YDLIDAR T-mini Plus + 机械臂夹爪。迷宫 7×7 树形、格距 0.4m、通道 0.4m、
车宽约束 21.5cm，现场公布、不可预输入，任务为遍历迷宫收集方块并从出口离场。

一句话算法：

> **滚动可信地图 + 格级动作状态机 + 树迷宫局部 DFS + 固定麦轮轨迹模板。**

## 核心思想（最高设计 invariant）

> 雷达可置信范围随车连续移动，把眼前未知地图不断转化为可直接执行的已知路径；
> 未来格信息一旦足以确定下一出口，立即按固定模板通过。
> 长期确认地图只服务于回溯、已知路径高速运行、树结构剪枝和定位校正，
> **不是正常前进的许可条件**。

两类"已知"必须严格区分（详见[算法规范 §1](docs/design/算法规范.md)）：

- **Rolling Trusted Knowledge** —— 当前连续位姿下雷达几何允许可信的信息，
  临时、连续、各向异性（正前可信远、侧向掠射可信近），立即可用于运动
- **Persistent Confirmed Map** —— 多帧确认 + 真实走过 + 校准后的长期拓扑，
  用于回头高速跑、树剪枝（成环必墙）、墙吸附定位

## 系统数据流

```
odom+IMU → Pose 预测 → 已确认墙校正位姿（防自证循环：新墙不得本帧自证）
LaserScan → 连续可信遮罩 → 边离散归属(ABSTAIN 兜底) → EdgeMap
        → TreeInference(成环必墙, 可撤销) → CellMark(DEAD/WAY/BRANCH)
        → CellAction(entry_side → exit_side)   ← 格子状态直接包含怎么走
        → Action Horizon(向前逐格读动作, 遇 UNKNOWN 停)
        → 固定模板 STRAIGHT / 左右1/4圆弧(R=0.2) / REVERSE / STOP
        → world-frame (vx,vy) + wz(yaw hold) → 麦轮逆解 → 四轮 PID
```

关键运动学：**车身朝向全程固定**（姿态保持环负责），麦轮转弯是平移速度
矢量沿 R=C/2=0.2m 四分之一圆弧平滑过渡——车头不转，速度矢量转。

速度没有"探索/已知"两个模式：`v_max = min(v_car, √(2·a_dec·(d_unknown−margin)))`，
前方能可信看到多远，就按已知赛道跑多快。

## 本项目不是

- 不是走到格中心再 sense-decide-move 的教学 Micromouse
- 不是必须完整地图确认后才允许运动
- 不是 BFS nearest-frontier 探索（DFS 只管岔路支路次序，存在格子局部状态里）
- 不是车头跟着路线旋转的差速车模型（0 SPIN / 0 TURN180）
- 不是不同弯道在线求复杂轨迹（只有一套固定模板）
- 不存在 CREEP/蠕动探路（STOP 原地等扫描是异常边界情况，不是每格流程）

## 当前阶段（2026-09-25）

- 设计真相已按用户原始方案重新定稿（`docs/design/算法规范.md`），旧架构文档
  已归档至 `docs/decisions/`（Rejected after requirement re-alignment）
- 代码按新架构重构进行中（`386a067` WIP）：运动层已换（STRAIGHT/ARC/REVERSE/STOP，
  事件检测支持格线停靠语义），存在一个已知 P0 死锁（STOP 落点在格线中点），
  详见算法规范 §15
- 验收路线：Gate A–L（确定性）→ Gate M（100 seeds → 1000 seeds）

## 仓库结构

```
software/ros2/m3pro_nav/m3pro_nav/   核心实现 (单一真相源)
  ├─ edge_map.py        EdgeMap (hard/soft/derived) + TraversalMap 分离
  ├─ tree_inference.py  树公理闭包 (成环必墙, 纯 base 重算可撤销)
  ├─ cell_classifier.py CellMark / resolve_exit 查表 / turn_type
  ├─ pose.py            Pose2D (唯一物理真相的数据类型)
  ├─ events.py / visits.py / event_detector.py   几何事件链
  ├─ motion_primitive.py / motion_planner.py / motion_executor.py
  │                     固定模板 STRAIGHT/ARC/REVERSE/STOP (只改位置, 不改 yaw)
  ├─ stream_nav.py      薄 coordinator: 事件→认知, BRANCH 局部状态机
  ├─ mazemap.py / tracker.py   路径 BFS / 全向跟踪 (上车件)
software/sim/           runtime_v2 事件驱动 SemanticSim + gate harness
software/tests/         Gate 测试 (重构中, 待按新规范重写)
docs/design/            现行设计文档 (算法规范 = 唯一设计真相)
docs/decisions/         历史/已否决方案存档 (Rejected after re-alignment)
docs/官方资料总览.md     134 份官方 PDF 摘要
```

## 阅读顺序

算法规范（设计真相）→ 本 README（总纲）→ tests（语义的可执行定义）→
stream_nav / runtime_v2（当前实现断点）。

## 约定与关键节点

- 比赛关键节点：报名截止 2026-09-30，作品提交 2026-11-09
- 算法规范是唯一设计真相；改代码前先对规范，规范改动须可追溯到用户原始方案
- "SemanticSim 已验证" ≠ "实车已验证"。性能数字必须带 Gate 级别与 failures
  计数，failures>0 即 INVALID
- 每完成一刀提交推送；Gate 未全绿不进入下一级
