# 2026-10-01 17:47 全迷宫手柄动态采集

原始采集的压缩副本来自本机 `field_data/20261001_174719_joystick_full_maze/`。本目录用于复现 2026-10-02 的离线墙体定位实验；未改写原始采集。

- `record_0.db3.zst` 解压后是 ROS 2 SQLite bag，SHA-256：`7464a3da2cc81e47eef6e1b00c14577b9aa9fe8e3020117679d10c92fe252390`。
- `frames.jsonl.zst` 解压后是采集器逐帧日志，SHA-256：`7a24c16d938c9afd8a997d403b95363930ba4b5d89bc826ee67c2e45e9b7d5bf`。
- `metadata.yaml` 是 bag 元数据；`session.yaml`、`summary.json`、`summary.csv`、`topics.txt` 是同次采集的上下文。

从仓库根目录复原当前回放脚本默认读取的目录：

```bash
mkdir -p field_data/20261001_174719_joystick_full_maze/bag/record
zstd -d -c experiments/2026-10-01-joystick-full-maze/record_0.db3.zst > field_data/20261001_174719_joystick_full_maze/bag/record/record_0.db3
zstd -d -c experiments/2026-10-01-joystick-full-maze/frames.jsonl.zst > field_data/20261001_174719_joystick_full_maze/frames.jsonl
cp experiments/2026-10-01-joystick-full-maze/metadata.yaml field_data/20261001_174719_joystick_full_maze/bag/record/metadata.yaml
cp experiments/2026-10-01-joystick-full-maze/session.yaml field_data/20261001_174719_joystick_full_maze/session.yaml
```

bag 含 `/scan_multi`、`/odom_raw`、IMU、TF、手柄和速度命令；**没有**保存两路原始 `/scan0`、`/scan1` 消息。`/scan_multi` 是已投影到 `base_link` 的融合端点，不能把其角度 bin 当成从车体中心发射的真实射线。采集起点由现场人工记录为 `(3,0)` 朝北；完整墙图只用于离线事后评分，不能馈入定位器。

本目录是数据归档，不表示该次行驶已经完成雷达定位或位置环验收。算法、指标和失败边界见 `docs/reports/2026-10-02-*`。
