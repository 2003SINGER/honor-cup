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

## 目标数据流（尚未端到端实车验收）

```
odom+IMU → Pose 预测 → 已确认墙校正位姿（防自证循环：新墙不得本帧自证）
LaserScan → 连续可信遮罩 → 边离散归属(ABSTAIN 兜底) → EdgeMap
        → TreeInference(成环必墙, 可撤销) → CellMark(DEAD/WAY/BRANCH)
        → resolve_next(prev_cell, cell)   ← 局部图状态直接决定下一格
        → Action Horizon(虚拟推进局部 DFS, 遇 UNKNOWN 停)
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

## 当前状态（2026-10-03）

| 部分 | 已证实的范围 | 尚未验收 |
|---|---|---|
| 拓扑、任务与固定运动模板 | 确定性测试及历史 Tier A 模拟已通过；车头保持朝向，ARC 是麦轮平移轨迹。 | 实车完整迷宫自主探索。 |
| 位置闭环 | 软件链和宿舍地面往返试验已有记录；当前默认参数仍需复测。 | 固件更新后的地面过冲、回正、制动、yaw 和外部位姿误差。 |
| 雷达墙吸附与 WALL | 旧 17:47 完整手柄包的离线候选达到 62/62 真墙、0 假墙；另一短包的严格门槛仅保留 11/18 原可观测真墙。 | 新自动行驶包、独立位姿真值、跨场地阈值和生产晋升。 |
| 在线定位 shadow | 重计算已隔离到子进程，只读、默认关闭、仅 dry-run 可启用；软件测试通过。 | Orin 上的耗时、丢帧、主控制 timer 抖动与物理墙误差。 |
| OPEN 与自主导航 | `nav_runtime` 有计算及安全门槛。 | 可靠 OPEN 来源和 trust 标定；实跑仍被 preflight 阻止。 |
| 底盘连接 | 10 月 2 日固件试验记录了 odom 30 Hz、IMU 30 Hz、雷达发布 140 ms 的配置。 | 最近记录的 CP2104/USB 故障之后，实时 `/odom_raw` 尚待重新核验。 |

**下一步**先恢复并只读核对底盘通信，再做限定轨迹位置环、同步录包和只读 shadow 对照。具体顺序及停止条件见[当前任务](TODO.md)；代码入口、实验版本与数据归属见[现行链路索引](docs/architecture/CURRENT_PIPELINE.md)。完整测量及限制见[位置环报告](docs/reports/2026-10-01-dorm-position-loop.md)、[WALL 质量报告](docs/reports/2026-10-02-wall-evidence-quality.md)、[shadow 检查点](docs/reports/2026-10-02-temporal-wall-runtime-checkpoint.md)和[固件记录](docs/reports/2026-10-02-firmware-probe.md)。

## 仓库入口

- `software/ros2/m3pro_nav/m3pro_nav/`：现行 ROS/导航包，包含 runtime、控制、墙证据和 shadow；具体模块身份见[现行链路索引](docs/architecture/CURRENT_PIPELINE.md)。
- `software/sim/`、`software/tests/`：仿真门槛和可执行契约；测试通过不能代替实车验收。
- `software/scripts/`：车端部署、数据采集及限定运动试验入口。会发布 `/cmd_vel` 的脚本须以实际使用说明和显式 `--run` 为准。
- `software/tools/`、`experiments/`：离线分析工具及带版本的实验索引；失败原型保留作可复核证据，不代表生产链。
- `field_data/`：Git 忽略的原始车端/场地数据和固件档案；复制后检查完整性。数据保留规则见[现行链路索引](docs/architecture/CURRENT_PIPELINE.md#data-ownership)。
- `docs/design/`：设计规范；`docs/architecture/`：实现边界与现行入口；`docs/reports/`：实验结果；`docs/decisions/`：项目决策记录。

## 阅读顺序

本 README → [现行链路索引](docs/architecture/CURRENT_PIPELINE.md) → [当前任务](TODO.md)；修改算法时再读[算法规范](docs/design/算法规范.md)，判断实验结论时回到对应报告和原始数据。AI 与开发者修改导航或定位前，先读现行链路索引确认模块身份。

## 约定与关键节点

- 比赛关键节点：报名截止 2026-09-30（已过），作品提交 2026-11-09
- 算法规范是唯一设计真相；改代码前先对规范，规范改动须可追溯到用户原始方案
- "SemanticSim 已验证" ≠ "实车已验证"。性能数字必须带 Gate 级别与 failures
  计数，failures>0 即 INVALID
- 每完成一刀提交推送；Gate 未全绿不进入下一级
