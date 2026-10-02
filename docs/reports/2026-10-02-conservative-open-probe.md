# 旧 `/scan_multi` 的保守 OPEN 推断探针

## 实现边界

新增 `software/tools/analyze_conservative_open.py`，只读 2026-10-01 17:47 手柄 bag。每个有限 endpoint 分别以已核对的候选雷达原点构造真实候选线段；仅所有原点假设都在命中前穿过同一格边内部，且穿越位置远离角点、命中点时，才为该 edge 记一票。synthetic `base_link` 原点不在候选集。每帧每 edge 最多一票；来源、边界或时间条件不满足即 abstain。

坐标来源：merger 输入为 `/scan0 /scan1`，TF 将各自点云投到 `base_link`；输出 `/scan_multi` 再按 1° bin 保留最近点。因此输出 endpoint 是经过角度量化/同 bin 丢点的融合几何。脚本直接从该 bag 的 `/tf_static` 读取两颗原点：`laser0_frame=(-0.11617,+0.09156)` m、`laser1_frame=(+0.10766,-0.09078)` m；必须两种来源射线都支持同一格边。

时间边界需纠正附件中的一项表述：本地源码没有把输出 stamp 设成 `now()`。它将 `merged_cloud` 初始化为 `clouds[0]`，最后复制该 cloud header；`clouds[0]` 的时间戳来自输入点云，而第二路对应扫描的逐束来源时间不会随合并 LaserScan 保留。旧 bag 前 100 帧的 `bag timestamp - header timestamp` 中位数：scan 约 `-85.7 ms`，odom 约 `-103.2 ms`，两路相差约 `17.5 ms`。这显示 bag 与 ROS header 有近 0.1 s 的时钟偏置，不能据负 lag 判定 scan header 错误；也不能把单一 fused header 当作两路束时间。

工具将 150 ms 显式设为**经验时间包络假设**，再仅接纳里程计估算 `|v| ≤ 0.03 m/s`、`|ω| ≤ 0.05 rad/s` 的帧，并把该包络、1° bin 半宽、2 cm 外参/融合余量计入几何 margin。Merger 会缓存每路最近点云，源码本身没有提供第二路点云年龄上界；因此这个 150 ms 不是形式保证。以下结果均为条件性诊断，不能作为在线 OPEN 写图门槛。

## 旧包结果

```text
合格帧                         436
检查的有限融合 endpoint       143,906
候选来源/几何/边界 abstain    119,978
motion gate 拒绝帧             1,326
同帧 WALL 冲突撤回 OPEN vote  1,023
每帧去重 edge OPEN vote       1,908
覆盖 edge                      78
逐票真 OPEN / 假 OPEN          1,365 / 543（精度 71.5%）
覆盖 edge 中真 OPEN / 真 WALL  43 / 35（精度 55.1%）
```

真值仅在所有候选 vote 生成后用于评分。场地物理轴变换仍未解析，所以混淆矩阵只在 session 的 `(3,0), N` 锚点与理想 0.4 m 网格方向成立时有效。按当前条件假设，这套规则确实从旧包产生了 OPEN 候选，但假 OPEN 占逐票的 28.5%，不安全，不能接在线地图。

## 验证

`python3 software/tools/analyze_conservative_open.py --output /tmp/honor-cup-conservative-open.json` 完成只读 bag 回放；对原 bag 未写入。`py_compile` 和 5 项 pytest 检查通过，覆盖多 edge 穿越、hit/corner abstain 与本帧 WALL 冲突候选。
