# 2026-10-02 M3 PRO 固件与车端探测记录

本记录区分离线 HEX 候选与实车状态。2026-10-02 已依次测过 20 Hz、50 Hz、odom 30 Hz + IMU 60 Hz、雷达发布 70 ms，并恢复到 odom 30 Hz + IMU 30 Hz + 雷达发布 140 ms。所有版本均做过静态读回和话题测频；尚未发送运动命令。

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

## odom 30 Hz + IMU 60 Hz 定时版上板结果（已替换）

- 用户再次进入 ROM，`stm32flash -e 3 -w ...odom_30Hz_imu_60Hz.hex -v` 只擦写前 3 个程序扇区。独立读回前 384 KiB，与候选全部 353,336 个映射字节一致，额外非 `FF` 字节为零。配置扇区与最初 2 MiB 备份逐字节相同。读回文件已复制到 Mac 固件归档。
- 板上两个定时字分别为 `33,333,333 ns`（odom）和 `16,666,666 ns`（推定 IMU）。`/YB_Node` 正常恢复，12 秒双话题订阅得到 `/odom_raw` 360 帧，header 时间戳约 **30.027 Hz**；`/imu/data_raw` 499 帧，header 时间戳约 **41.923 Hz**，并未达到标称 60 Hz。IMU 相邻 498 帧中有 8 对加速度/角速度载荷完全相同；具体限制位于传感器采样、MCU 调度还是通信链仍未定位。改变该常量后 IMU 频率从约 25 Hz 升到约 42 Hz，支持它与 IMU 发布定时有关，但不证明 IMU 物理采样率为 60 Hz。
- `/scan0`、`/scan1`、`/scan_multi` 仍分别约 7.14、7.18、7.14 Hz。车载 Ubuntu 上独立空载 ROS 60 Hz timer 连续 12 秒运行 720 tick，平均 **60.0 Hz**（中位间隔 16.66 ms，最大 22.33 ms）。这证明上位机具备 60 Hz 定时节拍；实际地面试验程序尚未运行。
- 30/60 固件 HEX SHA-256 为 `9a484bf6026a418ab16e38a53f3b51aec140fcec64b1bf0388e613bada2e2625`。25/50 是尚未上板的备用候选。雷达 10 Hz 需要改变物理转速并验证每圈完整数据；只改 ROS 的 140 ms timer 不构成真实 10 Hz 扫描。

## 雷达 70 ms 对照与当前固件

- 先录制静止基线 `field_data/scan_rate_baseline_140ms_20261002/`，再刷入 odom 30 Hz、IMU 30 Hz、雷达发布 70 ms 诊断版，读回与目标 HEX 的 353,336 个映射字节一致，配置扇区未变。诊断数据保存于 `field_data/scan_rate_diagnostic_70ms_20261002/`，两组数据均已复制到 Mac。
- 70 ms 时 `/scan0`、`/scan1` 发布约 14.3 Hz，相邻帧 `ranges` 数组无完全重复；但相邻有效波束的变化比例从 140 ms 基线的 0.731/0.698 降至 0.364/0.352，隔一帧比较则为 0.735/0.723。新增消息没有带来相同比例的新测量信息；ROS 数据还不足以直接断定物理转速。
- 70 ms 版本还使标称 30 Hz 的 odom/IMU 实际降到约 21.1/20.3 Hz。已重新刷入 **odom 30 Hz、IMU 30 Hz、雷达 140 ms** 版本，SHA-256 `b421bea1a402392e1ca58ddd75c24f93d5a674a1cdd979e06115e0fa2a4af9da`。独立读回程序区与 HEX 完全一致，原配置扇区与备份完全相同；`/YB_Node` 已恢复。
- 当前 12 秒静态订阅实测：`/odom_raw` header 约 **29.972 Hz**、`/imu/data_raw` header 约 **26.224 Hz**、`/scan0` 约 **7.142 Hz**、`/scan1` 约 **7.126 Hz**、`/scan_multi` 约 **7.159 Hz**。上位机独立 60 Hz timer 12 秒运行 720 次，平均 60.0 Hz。IMU 30 Hz 是定时器目标，实际约 26 Hz。

## 离线准备

- Mac `field_data/firmware_archive/2026-10-02/` 存有经哈希校验的 V1.1.2、V1.1.3 原厂 HEX 及对应 20/50 Hz 候选。V1.1.3 补丁只修改唯一 odom 定时字（`0x0800B818`），保留地址映射并重算 Intel HEX 校验；具体 SHA 见归档 README。
- `software/tools/patch_v113_odom_hex.py` 限定原厂映像哈希、唯一待改字与 Intel HEX 校验；各候选已做映射与读回核对。此前导航/观测测试 331 项通过；本次新增测试尚未在缺少 pytest 的 Mac 环境运行。
- Mac 当前 `ground_loop_trial.py` 已同步到车端并由 `colcon build --packages-select m3pro_nav` 成功编译。地面试验入口支持 `--control-hz 50|60`，请求和实际循环、hold/new-odom tick 数写入 CSV；当前 odom30/IMU30/雷达140 ms 固件在板上运行，地面轨迹等待车辆装回并摆入净空。

## 下一步接口

当前建议维持底板 odom 30 Hz、IMU 定时 30 Hz（实测约 26 Hz）、雷达发布 140 ms（实测约 7.1 Hz）、上位机控制 60 Hz。`2:1` 是调度选择，不代表每个控制 tick 都有新里程计；程序按新 odom 样本更新反馈，其间保持上一命令。车辆装回并摆好后，以 `ground_loop_trial.sh --run --control-hz 60` 采集同一轨迹 CSV，再根据新样本数、hold tick、实际控制间隔及轨迹误差决定是否优化或回退。在找到两路 T-mini Plus 的物理转速命令接口并验证完整新扫描之前，不再缩短雷达发布定时。当前 CP2104 的 DTR/RTS 自动进 ROM 尚未证实，不应以旧 Rosmaster CH340 的方法代替。

## 宿舍复跑前的 USB 断连（同日稍后）

- 用户已将车摆好并授权宿舍往返试跑。预检发现 `/cmd_vel` 无其他发布者，但 `/odom_raw` 无实时帧；`/dev/myserial` 和 USB ID `10c4:ea60` 均消失，代理进程仍在。内核日志显示 CP2104 在 10:43:30 从 `1-2.3` 断开；这晚于前述成功测频，不能把当时的帧率结果当成当前链路状态。
- 用户确认接线牢固且未改动。整车重启后，CP2104 未重新枚举；针对其原 USB hub 端口 3 的 `uhubctl` 断电重上电由用户在车端执行成功，但端口状态为 `0100 power`（连接位未置位），Jetson 仍只识别到映射 `/dev/mic` 的 CH340。该 CH340 不是底板通信口；两路雷达扫描也经缺席的底板 ROS 链路发布。现有证据不足以区分线缆、端口、CP2104 或板级电源/复位问题；没有证据指向 ROS 进程或固件仍在下载模式。
- **本次没有发运动命令，也没有生成宿舍试跑 CSV。** 试跑脚本已支持 `--site dorm`，新的 CSV 会存到 `field_data/dorm/`；CSV 只含里程计、IMU 和控制数据，不含雷达扫描。恢复 CP2104 枚举和实时 `/odom_raw` 后才能继续闭环试跑。
- 随后用户确认控制板唯一 USB 数据线未改动，并已在 Jetson 另一 USB 口试过；同一线连接手机和 Windows 可以传数据。用户再把**控制板 USB Connect** 接至 Mac，Mac `SPUSBHostDataType` 只显示两条内建 USB 总线，没有任何新外设；Jetson 两口也均未检测到 CP2104。进一步读取 Mac `IOAccessory` 当前状态：Type-C 端口显示 `ConnectionActive=Yes`、`IOAccessoryDetect=Yes`，但 `TransportsActive=("CC")`，没有激活 USB2 数据通道；因此不是“完全没有 Type-C 插入事件”，当前停在 USB 设备枚举之前。实时日志在首次插入后才开启，不能排除插入瞬间曾有短暂 descriptor 错误；现有日志没有捕获到这种错误。跨主机结果把故障范围收窄到控制板端 USB 插座、板上桥接芯片供电/复位/数据线或 CP2104 本体，但不能证明 STM32 主芯片或应用固件损坏。此前下载模式刷写经 CP2104 成功，亦不支持把当前 USB 不枚举归因于仍在下载模式。

参考： [M3 PRO 控制板接口](https://www.yahboom.net/public/upload/upload-html/1755253726/1.Introduction%20to%20the%20Control%20Board.html)、[M3 PRO BOOT0/RESET 烧录步骤](https://www.yahboom.net/public/upload/upload-html/1755254244/13.Flash%20access%20data.html)、[旧 Rosmaster CH340 教程](https://www.yahboom.net/public/upload/upload-html/1758600901/1.%20Update%20the%20expansion%20board%20firmware.html)、[stm32flash 参数说明](https://github.com/stm32duino/stm32flash/blob/main/stm32flash.1)。
