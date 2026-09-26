# TODO —— 进度真相板（2026-09-26）

> 数字只代表所列代码版本与 Gate。任一 seed 失败时，该轮 Gate 不算通过。
> 历史 R3 真相板见 `docs/decisions/2026-09-25-旧TODO-R3真相板.md`。

## 当前状态

`codex/template-topology-refactor` 已通过 Tier A 语义模拟重构验收；
`codex/control-chain-integration` 在其上构建独立运动控制链与 ROS 驱动探针；
`codex/rolling-horizon-continuation` 修复运行中动作链延长；
`codex/ros-control-adapter` 准备只读观测与受门槛限制的位置环车端试验；
以上已全部合入 `main`（5d8f91f, 122 tests）。
分支 `codex/feedback-follower`（233e5cd）补齐最后一层纯逻辑执行：
`FeedbackTrajectoryFollower` —— 外部实测 OdometryState 驱动（无 plant.step、
不自积分位姿）、odom↔planner 刚性帧锚定、显式 TRACKING/HOLDING/FINISHED
相位（STOP-only = HOLDING，非弧长轨迹例外）、STOP-only/ARC 采样/settle
完成语义测试覆盖。ROS 侧由 odometry_adapter + control_probe 承接
（odom→Twist 受控发布），rclpy 不进导航核心。129 tests 全绿。
`codex/feedback-follower`（d68156a）再补 ROS 运动执行骨架：
`MotionRuntimeCore`（IDLE/TRACKING/HOLDING/FINISHED/FAULT 状态机 +
DISARMED/ARMED/FAULT 安全分层 + 反馈新鲜度看门狗 + append_suffix 只改未来、
非法连续性立即拒绝）与 `motion_runtime` 薄 ROS 节点（/odom_raw→FeedbackSample
→core→/cmd_vel，默认 DISARMED，cmd_vel 发布者冲突拒绝 arm，关闭连发零指令）。
Gate A-O 确定性测试 140 项全绿。ActionHorizon 尚未真正接实时传感——
正式比赛仍需雷达/摄像头前端接入后经 load_plan/append_suffix 注入。
尚未完成（需实车/实感数据）：雷达 Trusted Mask、墙吸附定位校正、
d_unknown 信息视界速度上限、真实抓取链、下位参数标定。
`a1d43aa` / `tier-a-core-baseline` 是旧运动方案的 1000+1000 seed 回归参照；
`8893956` 是引入拓扑游标与方块任务层、但随机 Gate 尚未恢复的 WIP。
这些历史结果不能充当本分支验收。

以下 51 项确定性测试与 Gate H 数字属于
`codex/template-topology-refactor`，不直接充当当前分支的验收：

| 模式 | 100 seed | 1000 seed（0–999） | 1000 seed 平均模拟时间 | 平均剪枝格数 | 估算少走距离 |
|---|---:|---:|---:|---:|---:|
| fullinfo | 100/100 | 1000/1000 | 82.54 s | 0 | 0 |
| blocks（8 块） | 100/100 | 1000/1000 | 143.95 s | 3.101 | 2.481 m |

剪枝距离按每个未访问且已证实为空的格子 `2 × 0.4 m` 估算，是省略折返段的
几何估算，不是与另一实现做实测对照。1000 seed 按 0–249、250–499、
500–749、750–999 四段并行执行；每段 250/250、退出码 0，逐行校验
两种模式各覆盖 0–999 恰好一次。每 seed 均满足 Gate H 的硬断言。

## 本轮已完成

- 固定运动语言：`MotionPrimitive` 拒绝斜线、零长或长度错误的直线；
  ARC 只能是 R=0.2 m 的四分之一圆。`MotionPlanner` 只按
  `(prev, cell, next, pose)` 编译固定模板；错误锚点直接报
  `PlanGeometryMismatch`，不画任意连接线。返航保留 predecessor。
- 拓扑规划：`ActionHorizon` 持有有限动作链与虚拟局部 DFS；branch 的
  preview 不写真实状态，child 仅在虚拟返回时完成。入口根格与边界出口格
  分开处理；返航 BFS 可使用已确认 OPEN、尚未实际走过的边。
- 方块任务：相机观测只穿过直线 OPEN 走廊，记录正负信息；空死枝剪枝
  不改物理墙或走过记录，可在中途跳过已证实为空的后缀，新 BLOCK 可撤销证明。
  任务所需方块数由配置传入，不读取世界真值集合的大小作决策。
- 运动执行：直线和圆弧按加减速约束积分；到达 STOP 或 180° 反向接缝
  时，超过 1 微米制动距离对应的残速即报错，更小的数值残差归零。
  死路折返前预先降速，车身 yaw 不由模板修改。
- 验收：附件 §24 的 Geometry、局部图、边界等待、branch overlay、
  方块可见性、任务剪枝与结构禁令均有确定性契约测试；Gate H 每 seed
  断言 abort、collision、wrong_edges、topology_mismatch、false_prune
  及 8/8 方块收集。

## 下一步

1. 在实车 ROS Humble 环境只读核对 `/cmd_vel`、`/odom_raw`、`/imu/data_raw`
   的类型、频率、时间戳、已烧录固件版本与其他 `/cmd_vel` 发布者；
   `driver_probe` 和 `control_probe` 尚未在车端运行。
2. 在安全场地做小幅 `/cmd_vel` 阶跃并留 CSV，拟合轮速/里程计响应，
   量测固定模板的车体净空、侧移误差和定位误差；再替换控制模型中的
   `UNCALIBRATED` 参数。利用 `control_probe` 核验原始里程计帧/速度方向、
   位置环到 `/cmd_vel` 的闭环响应，然后接入正式动作链。
3. 接入真实 Observation Adapter，实装连续雷达可信遮罩、边离散归属与
   已确认墙定位校正，严格防止新墙本帧自证。实装规范 §10 的信息视界
   速度上限，并由真实可观测距离测定其参数；不凭 Tier A 假设定数值。

## 控制链分支新增验收（2026-09-26）

- 新增固定几何速度时间表、弧长参考、世界系前馈加位置反馈与独立 yaw
  保持环。输出车体系 `Twist2D`；旧的车头跟随运动方向的 `tracker.py`
  已从活动代码移除。
- 对照本地 `Subscriber_twist` 固件源码实现 10 ms 下位 tick、麦轮正逆解、
  编码器量化、四轮增量 PID 与 PWM 限幅/补偿；电机一阶响应参数明确
  `UNCALIBRATED`。源码与公式见 `docs/architecture/运动控制链与下位机审计.md`。
- 五项指定动作的下位模型确定性门槛：5/5 无离散碰撞；最小计算墙净空
  0.0143 m、最大横向误差 0.0442 m、最大轮速 165.7 RPM、最大终点误差
  0.0190 m。数字仅适用于当前未标定模拟参数，不能外推实车。
- 控制链分支在当时有 92 项确定性测试通过；该分支没有重跑 1000+1000
  随机种子。Tier A 核心运行时仍使用理想 `MotionExecutor`，地图算法不依赖
  未标定的下位模型。ROS 探针仅做静态与纯逻辑测试，尚无车端验证。

## 运行中延长动作链（2026-09-26）

- 地图更新后，可从队尾等待点的拓扑游标和位姿编 suffix；跨次编译传递虚拟
  DFS overlay。`MotionExecutor` 校验几何接缝，保留已启动动作。若不能安全
  移除尾端 STOP，就在 STOP 后继续，不重写正在执行的速度曲线。
- 修复方块模式的父子格来回弹跳：空死枝证明只能跳过尚未走过的未来支路，
  已走的返程边不能被当成可剪枝的前向支路。
- 当前分支 95 项确定性测试通过；随机 Gate 仅复查 fullinfo 0–9 和 blocks
  0–9，各 10/10 通过。blocks 平均模拟时间 104.8 s、平均路程未作为实车
  性能指标。尚未重跑 1000+1000，也未验证真实雷达与车端执行。

## 验收边界

可声称的是 Tier A 语义模拟的拓扑/任务/几何契约，以及未标定下位模型的
五项确定性接口门槛。真实雷达关联、闭环定位、实车车体净空、已烧录固件
一致性、真实轮速响应和车端 ROS 运行尚未验收；模拟通过不等于机器人
已能安全上场。

## ROS 位置环适配检查点（2026-09-26）

- 增加原始 `Odometry` 的有限值、四元数、时间戳和显式 frame 校验，并按
  ROS 消息约定把车体系 twist 转到里程计世界系。
- `control_probe` 默认只订阅并写 CSV；执行时必须显式给出实测 frame、
  轴和不超过 0.03 m 的距离。控制周期检查反馈接收新鲜度及其他
  `/cmd_vel` 发布者，约束输出幅值，试验结束后尝试连续发零命令 0.5 s。
- 本机 122 项测试通过；ROS Humble/colcon 和实车运行未验证。车端连接当前
  不可达，待同网后先进行只读核对，再决定小距离执行试验。
- 通用平面坐标变换已接入短直线试验，并用固定四分之一圆参考做纯逻辑检查；
  `REVERSE` 的构造也拒绝斜线。完整 Action Horizon 到 ROS 的运行时、
  实际迷宫/odom 对准及闭环实车结果仍未验收。

## 执行层审查修复（2026-09-26, GPT 审查单 P0×3 + P1×4 全部落实）

- P0 `motion_runtime_node._on_odom` 改传完整 `odometry_from_msg` 签名
  （此前第一帧 /odom_raw 即 TypeError）；坏帧丢弃告警不崩溃；新增
  rclpy-stub 注入的 callback 链测试（无需 ROS 环境）。
- P0 STOP 语义按 meta 分流：`meta['wait']` = WAIT STOP 永久 HOLDING
  （只有 suffix/cancel/fault 离开，绝不因 settle 变 FINISHED）；无 wait
  的终端 STOP 才 settle → FINISHED。
- P0 HOLDING 为位置/yaw 保持（v_ff=0 + 位置环，漂移拉回），不再是
  无条件全零；零误差零速度时 P+D 输出恰为零。
- P1 REVERSE 定稿为编译期宏：KINDS = STRAIGHT/ARC/STOP，死路折返由
  planner 编译为两段 STRAIGHT；validate/speed_profile/测试全部对齐。
- P1 修复两处假绿断言（曲率 `abs(abs(c)-5)`、REVERSE 去掉 `or True`
  改为真实去程/回程单调性检查）。
- P1 ARMED 期间每 ~0.5s 复查 /cmd_vel 归属，出现其他发布者 →
  FAULT + 零指令（新增 `MotionRuntimeCore.report_fault` 公共入口）。
- P1 append_suffix 双路径：巡航中 suffix 到达且终端 WAIT STOP 尚未
  影响活动段速度计划（未制动、活动段为 STRAIGHT、其后仅剩 WAIT STOP、
  切向连续、剖面可行）→ 保守 safe-extension 无缝拼接不停车
  （`follower.try_splice`：截断活动段 + 同帧变换 + elapsed 连续）；
  其余情形排队，刹停后续接（与仿真端语义一致）。排队中的新 suffix
  改为连续性校验后追加，不再静默覆盖。
- 连带修复：`_tick` 误用不存在的 `state.vx`（应为 vx_world）；
  `_build_follower`/`_apply_suffix` 把 planner_start.yaw 写死 0 导致
  整个计划绕锚点旋转实测 yaw 角 —— 改为规划系与 odom 对齐
  （yaw_ref = 实测底盘朝向），HOLD 续接沿用原帧变换不重锚。
- 验证：150 项测试全绿；仿真回归 10+10 seeds GATE PASS（avg 85.3s /
  104.8s），确认宏化 REVERSE 零影响。rolling horizon 的"跨越多个
  未开始段的无停车延长"仍属保守路径（排队刹停），未声称完整高速
  滚动优化。

## 实车感知调试工作台（2026-09-26, codex/scan-debug-workbench）

执行层冻结（0138a6b 不动），本轮补 Observation 前端调试设施：
- 纯 Python 管线: `scan_adapter`（RayObservation 保留 beam 语义与
  deskew 预留时间字段, NaN/Inf/越量程带因拒绝）→ `frame_projector`
  （laser→base→odom→maze 全链复用 RigidFrameTransform; ManualMazeAnchor
  一次性锚定后只跟随 odom 永不重吸附; 无 TF 无 YAML → 全体 NO_TRANSFORM
  ABSTAIN）→ `grid_association`（0.4m/7×7 canonical 网格, UNIQUE/
  AMBIGUOUS/NONE 三态, 角点歧义绝不强选边; 每候选带 residual/range/
  incidence/along/corner/uniqueness 全特征; free path 穿越生成 OPEN
  证据, hit 后不推理; edge_id 与 EdgeMap canonical 同构）
- `trust_policy`: diagnostic_only 默认恒开; TrustThresholds 全部
  UNCALIBRATED（None 即 ABSTAIN + 明确缺失清单）; 预留
  TrustedEdgeObservation → ObservationAdapter → EdgeMap 接口但不接导航
- `scan_diagnostics`: 在线/离线同一 FrameAccumulator + summarize_frames
  （1mm 直方图可加, 分位数精确可重放）; 绝不自动宣布可信距离
- `scan_debug` ROS 节点: 只读（无 /cmd_vel, 不 import 运动层, 结构测试
  断言）; topic 全可配置; 一次性 static TF maze←odom 仅供 RViz
- `scan_debug_markers`: 一帧 ≤6 个批量 Marker（POINTS/LINE_LIST）,
  绝不 per-beam; RViz 只观察不是数据通路
- `scan_debug.launch.py`（cell_x/cell_y/heading 参数化）+ 
  `config/scan_debug.rviz` + 10 个实验模板 + 外参样例（UNCALIBRATED）
- `software/scripts/field_scan_session.sh`（环境/topic/类型/帧名核对 →
  anchor → 节点+RViz+rosbag → git SHA/参数元数据 → 自动 summary）与
  `replay_scan_session.sh`（同参数重放 + 在线/离线 summary 自动 diff）
- `software/tools/scan_session_summary.py`（frames.jsonl → summary.json/
  csv, 按距离桶 unique rate/残差分位数/入射角/角距/拒绝原因直方图）
- FIELD_DEBUG.md: 现场全流程手册（0 前置 → 1 采集 → 2 产物 → 3 重放 →
  4 首批 session 建议 → 5 判读 → 6 UNCALIBRATED 清单 → 7 安全边界）
- 验证: 173 tests 全绿（新增 23 项感知契约: ray 语义/坐标链解析/锚定
  跟随/三态关联/角点弃权/OPEN 证据/ABSTAIN/批量 marker/重放逐字节一致/
  节点只读+可配置+端到端落盘）; summary CLI 合成 session 冒烟通过
- 未做（有意）: colcon build 需在车端/ROS 环境执行（本机无 ROS）;
  TrustPolicy 阈值标定、TrustedEdgeObservation 接 EdgeMap 属下一轮

## 真实闭环 runtime（2026-09-26, codex/scan-debug-workbench 续）

执行层与算法核心零语义改动，本轮把"眼睛→脑子→神经→腿"接成同一循环：
- `observation_adapter.py`（纯 Python）: WorldRay[] → StreamNav.observe
  的 hits/opens —— 与仿真 sense_from 语义对齐（dist=轴向垂距, alpha=
  真实入射角）; 同边同帧多 ray 保最小入射角（正对测量不被掠射覆盖）;
  逐边单票; 无回波射线只出 free-path OPEN 证据
- `nav_runtime.py`（纯 Python 协调器）: 事件驱动核心（on_odom 事件桥/
  on_scan 观测/planner_tick 编链-延长-返航/control_tick）; cursor+
  planned_cells 一致性校验; 返航链补终端 STOP 契约; 任务完成后才装
  返航（链到位再装, 延长分支加 done 守卫）; evidence 快照喂 LLM
- `nav_runtime_node.py`（薄 ROS）: subscriptions+timers 事件驱动（无
  while 串行）; dry_run 默认（完整计算但零 /cmd_vel publisher）;
  UNCALIBRATED 占位 + 实跑 → fail closed; /cmd_vel 归属 preflight;
  evidence 目录（events.jsonl/runtime.csv/config/git SHA）
- `config/nav_runtime.yaml`: 冻结硬件事实（/scan_multi 等已实测 topic）
  + __MEASURE__/__CALIBRATE__ 占位 + 感知参数（gate/dphi/rng/confirm_near
  = 仿真默认, 现场用 scan_debug 工作台数据人工修订）
- 连带修复三个真 bug: ① FrameProjector 全部 ray 存了同一方向 (c,s)
  → 逐 ray 方向; ② open_edges_along 端点浮点噪声把"打在墙上"当穿越
  → 给有墙的边投 OPEN 票（端点容差 1e-6）; ③ ARC 前衔接 v_end=v_arc
  无条件覆盖死路折返段 v_max=GRAB_V → 非法速度界（min(v_arc, v_max),
  motion_planner + action_horizon 两处; 仿真 executor 不校验所以从未暴露）
- **闭环契约测试**: ray-cast 合成雷达（几何真实: 360°/无语义捷径）+
  完美 plant + 帧变换（odom 平移+旋转 30°）→ 整链探索+返航 3 seeds
  全过（avg ~75s, wrong_edges=0, mismatches=0）
- 验证: 182 tests 全绿; 仿真 gate 回归 10+10 PASS（衔接修复零影响）
- 未做（按 GPT 路线图）: 方块视觉 Adapter（RGB→BLOCK/EMPTY）、机械臂
  collect transaction、真实标定（帧名/外参/感知参数——到场用工作台采数）
