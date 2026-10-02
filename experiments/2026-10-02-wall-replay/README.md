# 2026-10-02 动态墙体定位离线实验输出

本目录保存同一份 [17:47 手柄原始包](../2026-10-01-joystick-full-maze/README.md) 的分析输出。JSONL 为逐帧日志，使用 `zstd -d -c 文件名.zst` 查看；JSON 为汇总和后验评分。脚本位于 `software/tools/`，报告位于 `docs/reports/`。运行时完整迷宫真值只用于结束后的评分，不能参与前向估计。

| 输出前缀 | 脚本 | 当前结论 |
| --- | --- | --- |
| `dynamic-wall-snap` | `replay_dynamic_wall_snap.py` | 理想格线关联随里程计漂移失效；不能上线。 |
| `continuous-wall-geometry` | `replay_continuous_wall_geometry.py` | 线坐标可连续，但以格边为历史主键仍有冲突；不能上线。 |
| `pointcloud-wall-match` | `replay_pointcloud_wall_match.py` | 从融合端点拟合实测墙段并按本格真实点切长墙；独立视角 bootstrap 未成功，校正数为零。 |
| `honor-cup-conservative-open.json` | `analyze_conservative_open.py` | 旧包的保守 OPEN 候选有 543/1908 张逐帧票落在真墙上，仅作诊断。 |

报告中的条件性真值评分依赖人工 `(3,0), N` 锚点与物理网格方向正确；该轴向尚无独立测量。动态 `WALL → pose correction` 仍是待验收目标，不能把这些离线输出当作在线导航许可。
