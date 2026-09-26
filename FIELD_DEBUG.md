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
