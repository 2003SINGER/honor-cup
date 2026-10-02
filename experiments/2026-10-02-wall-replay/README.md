# 2026-10-02 动态墙体定位离线实验输出

本目录保存同一份 [17:47 手柄原始包](../2026-10-01-joystick-full-maze/README.md) 的分析输出。JSONL 为逐帧日志，使用 `zstd -d -c 文件名.zst` 查看；JSON 为汇总和后验评分。`frame-grid-snap-fp-by-edge.csv` 按格边列出错墙次数，`frame-grid-snap-fp-samples.csv` 保存每次错墙的帧、墙段、位姿和残差。`frame-grid-snap-edge-level.csv/json` 保存 112 条边的累计状态、真值和视角簇。脚本位于 `software/tools/`，报告位于 `docs/reports/`。运行时完整迷宫真值只用于结束后的评分，不能参与前向估计。

| 输出前缀 | 脚本 | 当前结论 |
| --- | --- | --- |
| `dynamic-wall-snap` | `replay_dynamic_wall_snap.py` | 理想格线关联随里程计漂移失效；不能上线。 |
| `continuous-wall-geometry` | `replay_continuous_wall_geometry.py` | 线坐标可连续，但以格边为历史主键仍有冲突；不能上线。 |
| `pointcloud-wall-match` | `replay_pointcloud_wall_match.py` | 从融合端点拟合实测墙段并按本格真实点切长墙；独立视角 bootstrap 未成功，校正数为零。 |
| `physical-wall-tracks` | `replay_physical_wall_tracks.py` | 连续物理墙作历史实体；后半程关联率和条件身份评分改善，但晋升后冲突多，尚无定位精度验收。 |
| `frame-grid-snap` | `replay_frame_grid_snap.py` + `replay_edge_level_acceptance.py` | 观测级条件命中 99.1%；112 边 wall-only 累计后，62/62 真墙确认，同时出现 12 个有效假墙，地图验收未通过。详见[报告](../../docs/reports/2026-10-02-frame-grid-snap-replay.md)。 |
| `wall-quality-*` | `sweep_wall_quality.py` | 逐格端点连续覆盖 + 地图确认门槛的离线扫描。完整包有 62/62、0 假墙的候选，17:41 短包暴露一条漏墙；参数尚不能上线。详见[报告](../../docs/reports/2026-10-02-wall-evidence-quality.md)。 |
| `honor-cup-conservative-open.json` | `analyze_conservative_open.py` | 旧包的保守 OPEN 候选有 543/1908 张逐帧票落在真墙上，仅作诊断。 |

`frame-grid-snap` 按用户提供的 `(3,0), N` 起点和 7×7 墙表完成离线 WALL → pose correction 原型；墙号评分已有数据支持。车身相对地面的位姿误差仍需单独测量，原型尚未接入在线导航。

新质量回放保存 `frame-grid-snap-quality.jsonl.zst` 和 `frame-grid-snap-174144-quality.jsonl.zst`，两者含每条边 2/3/4/5 cm 矩形内的原始端点与最长连续簇。`wall-quality-sweep.csv` / `wall-quality-174144.csv` 汇总扫描参数；同名 `.json.zst` 保存 112 边首次确认时刻、状态历史和连续物理偏移。17:41 原始短包另存于 [`experiments/2026-10-01-joystick-174144/`](../2026-10-01-joystick-174144/README.md)。完整包的位姿摘要和原报告不变。
