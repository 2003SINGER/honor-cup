# 2026-10-02 逐帧格线吸附离线回放

## 目标和边界

用 10 月 1 日 17:47 的手柄动态包验证用户提出的闭环：人工起点，上一帧校正位姿加本帧里程计增量预测，再用本帧雷达墙的**共同**横向、纵向、偏航偏差校正车位姿。这里只处理 `/scan_multi` 的有限命中端点；旧包缺原始双雷达逐束来源和时间，不能据它生成可靠 OPEN，也不能做真实逐束去畸变。本实验不改车端 runtime、导航许可或控制参数。

实现：[`software/tools/replay_frame_grid_snap.py`](../../software/tools/replay_frame_grid_snap.py)。它先从一帧命中点拟合局部墙段，依据强先验（0.4 m、7×7、只允许横竖墙）把墙段关联到附近格线；沿同一无限格线的碎片只算一票，求这帧统一的 `Δx, Δy, Δyaw`。只有一个方向的墙时只修可观测的法向坐标。距离、角度、共识、独立墙数和最大改正幅度均设门槛；歧义则沿用里程计预测。长墙切格时，每格必须有该格内原始支持点，不靠直线外推补墙。第一帧只用人工锚点；之后因果地使用上次校正位姿与当前 odom 增量。真值文件直到整段前向回放结束才读取。

仓库已有 [`GridAssociation`](../../software/ros2/m3pro_nav/m3pro_nav/grid_association.py) 和 [`EdgeMap`](../../software/ros2/m3pro_nav/m3pro_nav/edge_map.py) 的规范共享格边 ID，无须再造四方向墙 bitmask。旧 `physical-wall-tracks` 仍保留供诊断，本实验的校正**不依赖**其历史 track、独立视角晋升或 OPEN 票。

## 实测结果

输入 1,769 帧扫描，其中 1,763 帧有可用 odom 包围；拟合 13,374 个墙段。逐帧接受校正 989 次（`XY_YAW` 631、`X_ONLY` 187、`Y_ONLY` 171），拒绝 774 次。没有发现单帧大于 0.30 m 的跳格。输出为本目录 `frame-grid-snap-summary.json`、`frame-grid-snap.jsonl.zst`、`frame-grid-snap-timeseries.csv` 和 `frame-grid-snap-wall-scatter.csv`。

| 相同原始拟合墙段的后验评分 | 有效评分/总数 | 命中/有效评分 | 错墙/有效评分 |
| --- | ---: | ---: | ---: |
| 人工锚点 + 原始 odom，直接按最近格边量化 | 5,348/13,374（40.0%） | 2,797/5,348（52.3%） | 2,551/5,348 |
| 逐帧共同偏差校正，再按最近格边量化 | 11,996/13,374（89.7%） | 11,887/11,996（99.1%） | 109/11,996 |

同一原始墙段且两边都有有效评分时，2,517 段由原始 odom 的错墙变为校正后的真墙，22 段反向变差。切格后另有逐格评分，但两种位姿导致切格数不同，不能逐行配对；详见汇总 JSON。保留帧的格线残差中位数：原始 odom 96.4 mm、逐帧预测 24.60 mm、当帧校正后 24.59 mm。后两者相近，说明收益主要是**前序校正持续传递**，并非每帧都大幅拉回。与人工锚点加原始 odom 的轨迹差，末帧 0.758 m、最大 0.933 m；这是两种估计的差，**不是真实定位误差**。

## 对开源建议的判断

[micro_mouse_final](https://github.com/aramachandran7/micro_mouse_final) 的里程计与 LiDAR 到离散格墙流程、[lbxa/micromouse](https://github.com/lbxa/micromouse/blob/main/docs/algorithms.md) 的墙/已知状态分离，适合参考最终拓扑表达；仓库已有这个 owner。[ROSE²](https://github.com/aislabunimi/ROSE2) 从既有 OccupancyGrid 提取结构，输入条件与本包不同。另一种方案是“可靠 SLAM pose → 一次性对齐格网 → 每帧直接量化”。它的关键前提是可靠 SLAM pose；本包提供的是轮式 odom 和合并扫描，没有一条已验收的 SLAM `/map` 位姿。因此这份直接量化的对照只是**用户给定起点 + 原始 odom**，不是“优化过的一次性全局配准”实验。后者如要验证，应单独固定同一个变换、在前段拟合、后段盲评；不能用整段真值拟合变换再反评同段。

这里的“候选”只限附近格线和少量歧义拒绝，不形成长期物理墙对象。墙号最终仍是确定的规范 ID。按用户提供的 7×7 迷宫墙表和 `(3,0), N` 起点，当前实验表明逐帧共同吸附明显改善墙号判定。这个墙号评分有效，但它没有直接测量车身相对地面的厘米级位姿误差；车端 runtime 也尚未接入这项离线原型。

## 复现与下一关

在仓库根目录执行：

```bash
python3 software/tools/replay_frame_grid_snap.py \
  --session field_data/20261001_174719_joystick_full_maze \
  --truth-score field/maze_truth_7x7.json \
  --log /tmp/honor-cup-frame-grid-snap.jsonl \
  --summary /tmp/honor-cup-frame-grid-snap-summary.json
python3 software/tests/test_replay_frame_grid_snap.py
```

下一关是在车端接入前，用外部位姿或现场路标测量车身的厘米级定位误差，再看动态吸附能否驱动导航。真值文件里的 `physical_image_transform_resolved=false` 只表示两种**图片视角**之间尚未自动配准；本次评分使用用户直接提供的格坐标、方向和墙表，不依赖图片配准。墙体实际搭建偏离理想 0.4 m 时，单纯格线残差也会包含场地误差，须另看原始线坐标分布。
