# 2026-10-02 M3 PRO 固件与车端探测记录

本记录区分离线 HEX 候选与实车状态。2026-10-02 已依次刷入 20 Hz 和 50 Hz 候选并完成静态读回及话题验收；尚未发送运动命令。

## 实车只读结果

- SSH `car` 可连接，主机 `yahboom`；控制板通信口 `/dev/myserial -> /dev/ttyUSB0` 实测为 Silicon Labs **CP2104**（USB ID `10c4:ea60`）。另一个 CH340 `/dev/ttyUSB1` 映射 `/dev/mic`，不是底板通信口。
- 初次探测时 ROS 节点为 `/YB_Node`，`/odom_raw` 约 11.1 Hz。`~/config_robot.py` 的 `read_version()` 在停用代理、串口无人占用后返回 `None`；调试收到 `0x7E...` micro-ROS XRCE 帧，没有收到该脚本预期的 `FF FB...` 配置响应。该脚本自身的 `Version: V1.1.5` 不是板上版本；后续通过 Flash 读回确认板上原固件为 V1.1.3。
- 版本探测后，重新启动 micro-ROS 代理只能反复建立 session，`/YB_Node` 和 `/odom_raw` 未恢复。等待代理断开 20 秒后再启动、按车端 `config_robot.py` 注释的 DTR 时序重置，均未恢复；代理日志仍是 `session re-established` 循环。没有再重复查询版本。
- 在 Jetson `/tmp/m3pro_stm32flash` 临时解包 Ubuntu `stm32flash`，按用户提供的 `rts,-dtr,dtr:-rts,-dtr,dtr` 时序做**只读芯片握手**，工具返回 `Failed to init device`。没有擦除、写入或读取 Flash；不能据此证明该板能自动进入 ROM bootloader。
- 用户引用的 DTR/RTS 烧录页属于旧 **Rosmaster 扩展板 CH340**；本机底板实测为 CP2104，不能把旧板接线推断用于 M3 PRO。M3 PRO 官方控制板资料确认有独立 RESET、BOOT0 和 SWD 接口，并说明串口烧录可用 BOOT0→RESET 手动进入。

## 进入 ROM 后的备份与 20 Hz 刷写

- 用户按 BOOT0、RESET 使控制板进入下载模式。`stm32flash` 只读握手识别为 STM32H74xxx/75xxx（ID `0x0450`），无读保护障碍。板上前 384 KiB 的原厂 HEX 映射字节与 V1.1.3 的 353,336 字节全部一致；与 V1.1.2 有 327,041 字节不同。
- 刷写前完整读出 2 MiB Flash，车端与 Mac 备份 SHA-256 均为 `3b0ea573bccd4540b9400c045d34e00dc9fc425ebc17d0ed5b5dfd5fe2cd7eef`。程序占前 3 个 128 KiB 扇区；`0x08160000` 扇区有 96 个非 `FF` 字节，其余后续扇区为空。
- 用 `stm32flash -e 3 -w ...20Hz.hex -v` 仅擦写前 3 个程序扇区并逐块校验。随后独立读回前 384 KiB，与 20 Hz HEX 的全部 353,336 个映射字节一致，未见额外非 `FF` 字节。读回文件保存在 Mac `field_data/firmware_archive/2026-10-02/installed_20Hz_prefix_384KiB.bin`。
- `stm32flash -g 0x0` 成功从 Flash 启动，`m3.sh up` 后 `/YB_Node` 恢复。`ros2 topic hz /odom_raw` 连续约 10 秒最终平均 **19.997 Hz**；`/cmd_vel`、`/imu/data_raw`、`/battery`、`/arm6_joints`、`/scan0`、`/scan1` 都存在。后续测得两路原始雷达与 `/scan_multi` 均约 7.14 Hz。相机彩色图像仍无有效数据，属于另一个待查问题。

## 50 Hz 候选上板结果

- 用户再次以 BOOT0、RESET 进入 ROM。停止占用 `/dev/myserial` 的 micro-ROS 代理后，ROM 只读握手成功。`stm32flash -e 3 -w ...50Hz.hex -v` 仅重写前 3 个程序扇区并逐块校验。
- 独立读回前 384 KiB，与 50 Hz HEX 的 353,336 个映射字节完全一致，额外非 `FF` 字节为零；另读回 `0x08160000` 的 128 KiB 配置扇区，与刷写前 2 MiB 备份逐字节相同（96 个非 `FF` 字节）。两个读回文件位于 Mac `field_data/firmware_archive/2026-10-02/`。
- `stm32flash -g 0x0` 后 `/YB_Node` 再次上线。50 Hz 定时配置的 `/odom_raw` **实际约 42.8 Hz**，不是标称 50 Hz。300 条消息的 header 时间戳平均频率 42.8 Hz，中位间隔 20 ms，但 299 个间隔中有 100 个超过 30 ms；因此不是单纯 `ros2 topic hz` 接收端计算偏差。实际跳过/延迟的层级尚未定位，不能断言串口或 executor 已饱和。
- 修改 odom 后，`/imu/data_raw` 仍约 25.0 Hz；`/scan0`、`/scan1`、`/scan_multi` 仍约 7.1 Hz。官方 [IMU 发布例程](https://www.yahboom.net/public/upload/upload-html/1755256575/8.Publish%20IMU%20data%20topic.html) 用 40 ms publisher timer，官方 [雷达发布例程](https://www.yahboom.net/public/upload/upload-html/1770371947/9.Publish%20radar%20data%20topic.html) 用 140 ms publisher timer。两者是教学例程，不能直接证明实车集成固件的全部配置，更不能把 ROS 发布率当作物理采样率。官方 [T-mini Plus 说明](https://www.yahboom.net/public/upload/upload-html/1770202281/01.Lidar%20introduction%20and%20use.html) 给出转镜 6–12 Hz 可调、测距 4000 Hz。

## 离线准备

- Mac `field_data/firmware_archive/2026-10-02/` 存有经哈希校验的 V1.1.2、V1.1.3 原厂 HEX 及对应 20/50 Hz 候选。V1.1.3 补丁只修改唯一 odom 定时字（`0x0800B818`），保留地址映射并重算 Intel HEX 校验；具体 SHA 见归档 README。
- `software/tools/patch_v113_odom_hex.py` 的 5 个离线测试通过。此前导航/观测测试 331 项通过。
- Mac 当前 `ground_loop_trial.py` 已同步到车端并由 `colcon build --packages-select m3pro_nav` 成功编译。50 Hz 定时版本当前在板上运行；地面轨迹等待车辆装回并摆入净空。

## 下一步接口

用户决定跳过 20 Hz 地面试验，当前保持 50 Hz 定时版。车辆装回并摆好后，以实测约 43 Hz 的里程计反馈跑同一位置环轨迹并保存 CSV；随后根据跟踪误差、消息间隔和现场净空决定是否调整定时器、控制器或运动参数。IMU 与雷达先保持官方例程对应的 25 Hz、约 7 Hz 发布率：要提高它们，须分别确认新的 IMU/DMP 样本率和雷达真实扫描周期，并评估增加的 micro-ROS 传输负载。当前 CP2104 的 DTR/RTS 自动进 ROM 尚未证实，不应以旧 Rosmaster CH340 的方法代替。

参考： [M3 PRO 控制板接口](https://www.yahboom.net/public/upload/upload-html/1755253726/1.Introduction%20to%20the%20Control%20Board.html)、[M3 PRO BOOT0/RESET 烧录步骤](https://www.yahboom.net/public/upload/upload-html/1755254244/13.Flash%20access%20data.html)、[旧 Rosmaster CH340 教程](https://www.yahboom.net/public/upload/upload-html/1758600901/1.%20Update%20the%20expansion%20board%20firmware.html)、[stm32flash 参数说明](https://github.com/stm32duino/stm32flash/blob/main/stm32flash.1)。
