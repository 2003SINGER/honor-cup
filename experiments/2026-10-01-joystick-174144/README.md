# 2026-10-01 17:41 手柄短包

原始采集的压缩副本来自本机 `field_data/20261001_174144_joystick_full_maze/`，用作 17:47 完整包质量门槛的第二段复核。起点记录为 `(3,0)` 朝北；短包 848 个可用扫描帧，只覆盖部分场地。原始 bag 未改写，解压后 SQLite 文件 SHA-256 为 `5b91a4fe768f188a5bd19feb3897bc32fa54ca5e96518404d628cf997582d8e6`。
原采集的 `summary.csv` 和 `topics.txt` 也按原字节压缩留存。

从仓库根目录恢复回放目录：

```bash
mkdir -p field_data/20261001_174144_joystick_full_maze/bag/record
zstd -d -c experiments/2026-10-01-joystick-174144/record_0.db3.zst > field_data/20261001_174144_joystick_full_maze/bag/record/record_0.db3
cp experiments/2026-10-01-joystick-174144/metadata.yaml field_data/20261001_174144_joystick_full_maze/bag/record/metadata.yaml
cp experiments/2026-10-01-joystick-174144/session.yaml field_data/20261001_174144_joystick_full_maze/session.yaml
```

对应的逐帧质量日志与 112 格边累计结果在 `experiments/2026-10-02-wall-replay/`，评价见 `docs/reports/2026-10-02-wall-evidence-quality.md`。这段包不能代替完整地图或真实车身位姿验收。
