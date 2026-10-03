# 项目当前任务（2026-10-03）

本页只记录下一步和验收边界。旧版任务记录可从 Git 历史查看（`git show ffd4aca:TODO.md`）；旧数字不代表当前实车状态。代码与实验的角色见[现行链路索引](docs/architecture/CURRENT_PIPELINE.md)，设计约束仍以[算法规范](docs/design/算法规范.md)为准。

## 当前阻塞

- 最后一次跨主机检查时，控制板 CP2104 未枚举，车端没有实时 `/odom_raw`；详见[固件与 USB 记录](docs/reports/2026-10-02-firmware-probe.md)。本轮 Mac 的 `car` 主机名 `yahboom.local` 也无法解析，尚不能证明硬件已恢复。
- 限定轨迹位置环可以独立试验；完整 `nav_runtime` 自主探索仍被未标定 trust、融合雷达不能提供可靠 OPEN 等 preflight 挡住。这是有意的运行边界，不应靠改开关绕过。

## 恢复实车后的顺序

1. [ ] **恢复通信并只读验收。** 确认 `/dev/myserial`/CP2104 枚举、唯一 micro-ROS 代理、`ROS_DOMAIN_ID=30`、实时 `/odom_raw`、`/imu/data_raw`、`/scan0`、`/scan1`、`/scan_multi` 的类型、帧名、源时间和实际频率。记录硬件状态及部署 SHA。
2. [ ] **复测位置环。** 在已确认净空和可立即停机的条件下，用 `software/scripts/ground_loop_trial.sh --run --control-hz 60` 跑已有固定直线/ARC/返程路线；保存 CSV，核对过冲、回正、车头方向、制动、控制 timer 抖动与新 odom 更新数。先验收当前参数，再决定是否调参。
3. [ ] **同步录自动动态包。** 同时启动 `field_scan_session.sh`（真实格坐标和朝向）与限定轨迹试验，保留原始双雷达、融合雷达、里程计、IMU、`/cmd_vel`、TF 和元数据；结束后检查 bag 完整性并同步到 Mac。采集与控制使用各自终端，避免两个 `/cmd_vel` 发布者。
4. [ ] **只读 shadow 验收。** `frame_grid_shadow` 默认关闭且只能在 `dry_run` 中运行。与限定轨迹并行时检查 Orin 上的 worker 耗时、队列丢帧、结果延迟、主节点控制 timer 抖动，以及每帧 Δx/Δy/Δyaw、墙残差和格边跳变；不让提案控制车辆。
5. [ ] **用新包验证 WALL。** 对 112 个 canonical edge 统计最终/曾经误墙、真墙召回、首次确认延迟，并按距离、侧墙位置、直线/ARC、独立视角分组。旧完整手柄包上的 62/62、0 假墙是离线候选成绩；17:41 短包召回下降，不能直接冻结生产阈值。

## 实车结果之后才决定

- [ ] 依据 shadow 延迟和定位误差，选定唯一生产定位器；旧 `pose_correction.py` 与新 `frame_grid_snap.py` 暂时保持基线/诊断关系，不同时接管导航锚。
- [ ] 给 OPEN 建立可追溯的原始雷达来源与保守验收，再考虑解除完整探索的 preflight。
- [ ] 做一次**纯结构**清理：缩小 `nav_runtime.py` 职责，类型化墙证据，抽取两个试验 CLI 的公共代码，归档明确失败的原型。保持算法、阈值、固定模板与现有实车试验语义不变。
- [ ] 若实车滚动感知确有性能收益，再实现已装入 follower 的未来轨迹在合法模板锚处安全取消；目前只支持取消未装入的排队后缀。

## 最近软件检查点

`ffd4aca` 将 frame-grid shadow 求解移至独立子进程，保持默认关闭和 dry-run 限制；本地 `software/tests` **426 项通过**。这是软件契约，不等于 Orin 或车辆运行验收。详见[检查点报告](docs/reports/2026-10-02-temporal-wall-runtime-checkpoint.md)。
