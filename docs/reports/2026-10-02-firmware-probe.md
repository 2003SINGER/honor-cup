# 2026-10-02 M3 PRO 固件与车端探测记录

本记录区分离线 HEX 候选与实车状态。到记录时为止，没有刷写固件，也没有发送运动命令。

## 实车只读结果

- SSH `car` 可连接，主机 `yahboom`；控制板通信口 `/dev/myserial -> /dev/ttyUSB0` 实测为 Silicon Labs **CP2104**（USB ID `10c4:ea60`）。另一个 CH340 `/dev/ttyUSB1` 映射 `/dev/mic`，不是底板通信口。
- 初次探测时 ROS 节点为 `/YB_Node`，`/odom_raw` 约 11.1 Hz。`~/config_robot.py` 的 `read_version()` 在停用代理、串口无人占用后返回 `None`；调试收到 `0x7E...` micro-ROS XRCE 帧，没有收到该脚本预期的 `FF FB...` 配置响应。因此**安装版本未知**，不能把脚本自身的 `Version: V1.1.5` 当作板上版本。
- 版本探测后，重新启动 micro-ROS 代理只能反复建立 session，`/YB_Node` 和 `/odom_raw` 未恢复。等待代理断开 20 秒后再启动、按车端 `config_robot.py` 注释的 DTR 时序重置，均未恢复；代理日志仍是 `session re-established` 循环。没有再重复查询版本。
- 在 Jetson `/tmp/m3pro_stm32flash` 临时解包 Ubuntu `stm32flash`，按用户提供的 `rts,-dtr,dtr:-rts,-dtr,dtr` 时序做**只读芯片握手**，工具返回 `Failed to init device`。没有擦除、写入或读取 Flash；不能据此证明该板能自动进入 ROM bootloader。
- 用户引用的 DTR/RTS 烧录页属于旧 **Rosmaster 扩展板 CH340**；本机底板实测为 CP2104，不能把旧板接线推断用于 M3 PRO。M3 PRO 官方控制板资料确认有独立 RESET、BOOT0 和 SWD 接口，并说明串口烧录可用 BOOT0→RESET 手动进入。

## 已准备但未上板

- Mac `field_data/firmware_archive/2026-10-02/` 存有经哈希校验的 V1.1.2、V1.1.3 原厂 HEX 及对应 20/50 Hz 候选。V1.1.3 补丁只修改唯一 odom 定时字（`0x0800B818`），保留地址映射并重算 Intel HEX 校验；具体 SHA 见归档 README。
- `software/tools/patch_v113_odom_hex.py` 的 5 个离线测试通过。此前导航/观测测试 331 项通过。
- Mac 当前 `ground_loop_trial.py` 已同步到车端并由 `colcon build --packages-select m3pro_nav` 成功编译。它需要 `/odom_raw` 才能开始地面轨迹，当前不能试跑。

## 下一步接口

先恢复控制板运行态并确认 `/YB_Node`、`/odom_raw`。若要不拆车自动刷写，必须先通过这块 M3 PRO 的 USB 链路成功读取 ROM bootloader 芯片 ID；当前给定 DTR/RTS 时序没有成功。若需要读回板上原固件，须接可用的 ST-Link/SWD 或进入有效的 ROM bootloader，并先确认读保护状态；不要执行解保护，因为 RDP1 降级可能擦除原固件。版本或可读备份确认后再选择对应版本的 20 Hz 候选，上板读回验证、测 `/odom_raw` 频率及其他话题，跑同一位置环轨迹；通过后才重复 50 Hz。

参考： [M3 PRO 控制板接口](https://www.yahboom.net/public/upload/upload-html/1755253726/1.Introduction%20to%20the%20Control%20Board.html)、[M3 PRO BOOT0/RESET 烧录步骤](https://www.yahboom.net/public/upload/upload-html/1755254244/13.Flash%20access%20data.html)、[旧 Rosmaster CH340 教程](https://www.yahboom.net/public/upload/upload-html/1758600901/1.%20Update%20the%20expansion%20board%20firmware.html)、[stm32flash 参数说明](https://github.com/stm32duino/stm32flash/blob/main/stm32flash.1)。
