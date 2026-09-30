# 2026-10-01 摄像头输入检查

在宿舍架空状态，只启动官方 Orbbec DaBai DCW2 相机驱动，没有启动巡线、识别、导航或机械臂动作节点。车的 mDNS 名称一度无法解析，改用先前确认的 `10.24.200.6` 只读核对 ROS_DOMAIN_ID=30。

- 启动前只有 `/rgb [std_msgs/msg/ColorRGBA]`，没有相机图像。原 `m3.sh up` 错把 `/rgb` 当成相机就绪，因而跳过相机驱动。
- 单独运行 `ros2 launch orbbec_camera dabai_dcw2.launch.py` 后，`/camera/color/image_raw` 与 `/camera/depth/image_raw` 均为 `sensor_msgs/msg/Image`，且各收到一帧完整消息。
- 彩色图：640×480、`rgb8`、`camera_color_optical_frame`；短时 `ros2 topic hz` 约 15–17 Hz。深度图：640×480、`16UC1`，当前消息也标 `camera_color_optical_frame`。这是输入存在的证据，尚未检查图像内容和色深对齐。
- 官方巡线示例 `car/yahboomcar_ws/src/M3Pro_demo/M3Pro_demo/follow_line.py` 读取这两个图像话题，但还会发布 `/cmd_vel` 与机械臂指令，不能直接并入导航。其深度转换请求 `32FC1`，与本次实测的 `16UC1` 输入不一致，复用前需核查单位和转换行为。
- 当前 `nav_runtime_node` 只订阅雷达和里程计；视觉目标识别、格子绑定和目标收集尚未连到实车运行链。

相机驱动启动日志在车上 `~/honor-cup/field_data/20261001_camera_input/camera.log`。本阶段的验收仅是图像输入可读取，不等于视觉功能打通。
