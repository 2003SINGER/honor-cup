# 连续物理墙 track 离线回放

## 目标与边界

在不改现有 pointcloud prototype 的前提下，验证以连续有限墙段作为主地图实体时，能否减少理想 0.4 m 格线身份先验带来的动态退化。输入为 `field_data/20261001_174719_joystick_full_maze` 的 `/scan_multi` 有限命中点与 odom/静态 TF。回放只读 bag；不涉及 ROS runtime、车辆或控制。

新工具为 `software/tools/replay_physical_wall_tracks.py`，合成检查为 `software/tests/test_replay_physical_wall_tracks.py`。候选墙依照连续方向、法向距离、有限切向重叠关联；理想网格不参与 track 创建、关联、晋升或修正，只在回放结束后提供粗拓扑 hint。每帧先用此前稳定 track 提议 pose correction，随后才用当前观测更新历史。首批墙以人工起点和里程计短期增量投影；晋升至少需要 3 个互相独立视角（位置差 0.15 m 或朝向差 15°，且相隔至少 1 s）、方向跨度不超过 4°、法向偏差不超过 35 mm、重复观测有限跨度支持至少 10 cm。重复同一 pose 的帧只算一个视角。

## 合成检查

4 项标准库检查通过：

- 同一地点 40 帧重复观测不晋升墙；
- 物理墙位于理想格线偏移 20 mm 时，track 仍保留实测坐标；
- 长拟合墙只在各自有实测 hit 的格段输出支持，中间无点格不补墙；
- 无向线端点反转且角度跨 0/π 时，normal 与有限 overlap 保持一致。

## 动态包结果

最终修正版完整处理 1,763 帧，6 帧因 odom bracket 超过限制跳过；提取 13,374 条有限线段。corrected map 与独立 raw-odom baseline 分别产生 4,257 / 4,591 个 track，其中曾晋升且无后续冲突的稳定可用 track 为 62 / 68。已晋升 track 后续发生冲突的数量为 42/104 和 37/105。至少一对方向正交的稳定可用 track 最早在 7.28 s 出现。

后半程在相同 882 帧评估之前是否能唯一关联到既有物理 track：

| 指标 | WALL 修正轨迹 | anchor + raw odom |
|---|---:|---:|
| 可关联线段 / 全部拟合线段 | 64.89% | 61.56% |
| 每帧平均既有 track 关联数 | 5.246 | 4.977 |
| 事后可映射稳定 track 的真墙样本 | 47 | 11 |
| 事后可映射稳定 track 的假墙样本 | 34 | 93 |
| 条件 precision（仅计可判真/假的稳定 track 样本） | 58.0% | 10.6% |

跨整个回放，事后可给粗拓扑 hint 的已晋升 track：corrected 为 32 true / 13 false / 59 unknown，raw 为 32 / 21 / 52；其中包含后续冲突 track。无冲突 track 的后验样本是上表条件 precision 的来源。网格 hint 仅依据起点 anchor 与理想网格的后验最近邻关系，故这些 truth 数字不是独立测量精度，也不能排除场地轴、anchor 或施工误差造成的误标。

共接受 170 次修正。修正绝对量 p50/p90/max：`dx` 11.0/41.5/80.8 mm，`dy` 9.0/34.8/97.4 mm，`dyaw` 0.0132/0.0516/0.0868 rad；最大平移约 97.4 mm（接近 100 mm 门限），最大 yaw 修正约 4.97°（接近 5° 门限）。

本轮还修正了有向切线的规范化：多数水平线一律沿 `+x`，多数竖直线一律沿 `+y`。否则小负斜率水平线会落到接近 π 的角度，导致有限跨度中点变成负值，后验拓扑 hint 丢掉正场地内的线段。回归测试现用正场地中的小负斜率横墙，并验证反向端点的 normal 与 overlap 相同、hint 为 `('H', 3, 1)`。

## 结论

这是**有限正面证据，但整体验收失败**：相同后半程里，连续物理 track 的关联段比例比 raw-odom 高 3.32 个百分点，稳定 track 的后验条件身份 precision 为 58.0% 对 10.6%。不过 42/104 与 37/105 条晋升 track 后续冲突，track fragmentation 仍严重；接受修正也出现接近平移和 yaw 限幅的值。viewpoint 独立性由轮式 odom 判定，可能受漂移污染；目前没有外部实测 pose 或测量墙位，无法证明 pose 真正变准。本原型只给物理 track 事后粗拓扑 hint，还没有把稳定物理墙在有自身点支持的格段写成在线离散 EdgeMap。因此只保留为离线研究原型，不接 runtime，也不能声称已验证在线 WALL→pose correction。

可复现命令：

```bash
python3 software/tests/test_replay_physical_wall_tracks.py
python3 software/tools/replay_physical_wall_tracks.py
```

默认 summary 和逐帧 log 写入 `/tmp/honor-cup-physical-wall-tracks-summary.json` 与 `/tmp/honor-cup-physical-wall-tracks.jsonl`。
