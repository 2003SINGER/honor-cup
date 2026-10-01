# 2026-10-01 手柄动态采集验收

车端 Applications 的 `Joystick Maze Data Capture` 已修复并由操作者自行启动、关闭。关闭采集终端后，独立清理进程生成 `cleanup.done`、`summary.json` 和 rosbag `metadata.yaml`；两份 bag 的 SQLite `integrity_check` 均为 `ok`。车端原件与 Mac 副本分别在 `/home/jetson/honor-cup/field_data/` 和本仓库 `field_data/`。

| Mac session | `/scan_multi` | `/odom_raw` | `/imu/data_raw` | `/joy` | `/cmd_vel` |
| --- | ---: | ---: | ---: | ---: | ---: |
| `field_data/20261001_173742_joystick_full_maze/` | 199 | 315 | 708 | 473 | 476 |
| `field_data/20261001_174144_joystick_full_maze/` | 848 | 1320 | 2968 | 1904 | 1904 |

第二份是操作者所说的短距离前进再后退测试。`/cmd_vel` 中前进命令 77 条、后退命令 33 条（阈值 `|linear.x|>0.03 m/s`）；轮式里程计累计路径约 0.959 m，起终点近似 `(2.152, 0.733)` 和 `(2.151, 0.722)` m。这验证了动态采集链、手柄命令与里程计数据同时存在，不构成独立的物理距离标定。

启动项将迷宫锚点固定为入口 `(3,0)`、车头 N。短测实际起点在采集时未由操作者重新确认，因此 `frames.jsonl` 的绝对迷宫坐标暂不作为真值；原始 bag 的传感器时间序列仍可用于相对运动、墙体拟合和传感器对齐。正式全图采集应从该入口锚点开始，或另行记录实际起点与车头方向。
