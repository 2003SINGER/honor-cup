# FIELD_DEBUG —— 实车感知调试现场手册

> 目标: 到场地后**只做实验, 不做软件开发**。摆车 → 跑命令 → 看 RViz →
> 录数据 → 换位置。所有代码/TF/rosbag/网格关联已就位。

## 0. 前置 (每台车首次)

```bash
# Jetson 上: 编译工作区 (一次)
cd ~/honor-cup/software/ros2
colcon build --packages-select m3pro_nav
source install/setup.bash

# 确认驱动在跑
ros2 topic list | grep -E "scan|odom|imu"
```

`colcon build` 必须零错误通过; 若报缺 `visualization_msgs`/`tf2_ros`,
先 `rosdep install --from-paths . --ignore-src -r -y`。

## 1. 启动一次采集 session

人工把车摆进格子 (车中心对准格中心, 车头对准 N/E/S/W), 然后:

```bash
cd ~/honor-cup
./software/scripts/field_scan_session.sh --cell 3 2 --heading N --label dead_end_2cells
```

脚本自动完成: topic/类型/帧名核对 → maze 锚定 → scan_debug → RViz →
rosbag → 元数据 (git SHA / ROS distro / 参数) → Ctrl-C 后自动生成 summary。

带实验模板与自定义 topic:

```bash
./software/scripts/field_scan_session.sh \
    --config configs/scan_experiments/dead_end_2cells.yaml \
    --cell 3 2 --heading N --label dead_end_2cells_run1

SCAN_TOPIC=/my_scan ODOM_TOPIC=/my_odom ./software/scripts/field_scan_session.sh ...
```

无 TF 时给显式外参 (UNCALIBRATED 值仅当占位):

```bash
EXTRINSIC_YAML=configs/laser_extrinsic_uncalibrated.yaml \
    ./software/scripts/field_scan_session.sh --cell 3 2 --heading N --label x
```

**RViz 检查**: Fixed Frame = `maze`, 应看到 —— 原始 /scan 点、灰色网格、
绿色 UNIQUE hit、黄色 AMBIGUOUS hit、红色候选墙段、蓝色 OPEN 证据短线。
全黄/全灰 = 外参或锚定有问题, 先看终端告警。

## 2. 产物

```
field_data/<时间戳>_<label>/
├── bag/                 # rosbag2 (/scan /odom_raw /imu /tf* debug topics)
├── session.yaml         # cell/heading/topic/帧名/git SHA/ROS distro
├── topics.txt
├── frames.jsonl         # 每帧诊断聚合 (在线/离线同一格式)
├── summary.json / .csv  # unique rate · 残差分位数 · 按距离桶 · 拒绝原因
└── experiment.yaml      # 若用了模板
```

## 3. 离线重放 (同一算法链, 零第二套代码)

```bash
./software/scripts/replay_scan_session.sh field_data/<session>
```

结束时自动 diff 在线 summary vs 重放 summary, 时间无关统计必须一致。

只重算 summary:

```bash
python3 software/tools/scan_session_summary.py field_data/<session>
```

## 4. 第一批建议 session (按序)

1. `boundary_straight` —— 最干净的正对残差基线
2. `front_wall_1cell` / `front_wall_2cells` —— 正对距离 vs 残差/unique rate
3. `corridor_center` / `corridor_left_offset` —— 侧墙可信度对照
4. `dead_end_2cells` —— **关键实验**: 结构能否提前 2 格确定
5. `t_junction` / `boundary_corner` —— 开口边缘与角点的 AMBIGUOUS 行为

每姿势录 ≥30s; 同一姿势可微移 5cm 再录一档作对照。

## 5. 看 summary 怎么判断

`summary.csv` 按 `range_bin` 拆: `unique_rate` 高且 `residual_p50/p90`
小 = 该距离可靠; `ambiguous_rate`/`NEAR_CORNER` 抬升的距离 = 不可信区。
**程序不会也不会替你宣布"可信距离"** —— 看完数据人工把阈值填进
TrustPolicy, 那是下一轮的事。

## 6. UNCALIBRATED 清单 (现场实测前禁止当真值)

| 参数 | 状态 | 实测方法 |
|---|---|---|
| laser→base 外参 (x/y/yaw) | UNCALIBRATED (TF 优先) | 卷尺 + 正对墙残差 |
| TrustThresholds 全部 (残差/入射/距离/唯一性/角距/票数) | UNCALIBRATED | 本工作台数据 |
| 雷达实际量程与噪声特性 | UNCALIBRATED | `range_bin` 统计 |
| odom 漂移率 | UNCALIBRATED | 静止 session 的 odom 时间序列 |

## 7. 安全边界

- `scan_debug` **只读**: 无 `/cmd_vel` publisher, 不 arm 运动层
  (结构测试断言, 见 `test_scan_debug.py`);
- `diagnostic_only=true` 恒开: 一条观测都不会写 EdgeMap;
- 无外参 → 全体 NO_TRANSFORM ABSTAIN, 绝不猜雷达在车中心;
- 网格关联 UNIQUE/AMBIGUOUS/NONE, 宁可弃权绝不吸错格子。

## 8. 正式导航 runtime（nav_runtime, 2026-09-26）

整链已接通（事件驱动，非 while 串行）：

```
/scan_multi ─→ on_scan ─→ RealObservationAdapter ─→ EdgeMap
/odom_raw   ─→ on_odom ─→ 事件桥(GridEventDetector) + 反馈
(50Hz timer)  ─→ MotionRuntimeCore ─→ /cmd_vel (仅实跑模式)
(10Hz timer)  ─→ ActionHorizon 编链/延长/返航
```

**离线闭环已验证**：合成 ray-cast 雷达 + 完美 plant 的契约测试
（`test_nav_runtime.py` 3 seeds）全程跑通探索→返航，wrong_edges=0、
拓扑零失配。实车语义尚未验证。

### dry-run（第一步，安全）

```bash
# 车摆进格 (格中心, 车头对 N), source 工作区后:
ros2 run m3pro_nav nav_runtime --ros-args \
    -p config_path:=<安装路径>/config/nav_runtime.yaml \
    -p dry_run:=true \
    -p run_label:=first_dryrun
```

dry-run 完整跑传感器→地图→规划→轨迹→控制计算，但**不创建
/cmd_vel publisher**。看 evidence 目录（`field_data/run_*_dryrun/`）：
`events.jsonl`（anchor/plan/extend/crossed/mismatch）、`runtime.csv`
（每控制周期的指令与误差）、`summary.json`。

### 实跑（配置齐全后）

config 里所有 `__MEASURE__`/`__CALIBRATE__` 填实测值（帧名先用
driver_probe 核对），然后 `dry_run:=false`。preflight 任一不符即拒绝：
帧名、外参可用、配置占位、/cmd_vel 归属。

### evidence 喂 LLM

一次 run 一个目录（bag 可另录），把 `events.jsonl` + `runtime.csv` +
`config.yaml` + `git.txt` + 现象描述直接交给强模型分析
（"为什么第 63s 出现 mismatch"这类问题）。
