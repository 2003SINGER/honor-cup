# TODO

> 真相板: 这里是唯一进度真相. 完成项不删, 打 ✅ 并留证据.

## 🔴 P0 — 流式探索边角鲁棒性专修 (下一步第一优先)

30 种子仿真 (v3 架构, 2026-09-25) 暴露: 全信息模式 4/30 中断 (看门狗兜底),
violation 均值 31 (大部分是出界兜底 +50 计费, 非真实擦碰). 根因群:
- frontier/BFS 收尾语义: seen 边的远端格未 visited 时 frontier 判定 (open_edge 不再 touch 后语义变化)
- way/branch 重访路径的 BFS 导航死角
- cut 完成接缝的位姿不连续 (viol_line 尾部 ~9 次)
- ✅ 已修: way_forward 弯后来向错位 / spin90 旧 plan 残留 / far==cell 切弯 / 出界兜底必终止
验收: 100 种子 0 中断, violations 全部来自真实擦碰模型 (无 +50 兜底计费)

## 🟠 P1 — Sim→Real 接线

- [ ] /scan + 定位 → (cell,dir,dist,α) 几何反算 → StreamNav.observe()  ← 唯一大 ⛔
- [ ] 实测 δφ/gate/σ_range 回填 (d_conf 曲线: 掠射误差 vs 距离/入射角)
- [ ] 相机方块检测 → set_block_seen() (视野模型已在仿真: 1.5m ±45°)
- [ ] decision_node 实例化 StreamNav (import 已加, 节点回调用 ⛔)
- [ ] tracker 上车闭环 (cmd_vel → 轮速)

## 🟡 P2 — 工程清理

- [ ] 双 mazemap/tracker 拷贝合并 (src/ 与 ros2 包内, 当前 SHA 相同但会漂移)
- [ ] tests/ pytest 套件: degree 语义五类格 / 证据冲突双向 / json roundtrip /
      tracker 变异路径 / 碰撞三场景 / 死路有块必进
- [ ] FP/FN 压测: 方块检测误报漏报 + 墙口误判率
- [ ] _dbg 临时文件清理纪律 (gitignore)

## ✅ 已完成 (摘要)

- ✅ 2026-09-25 GPT 审计修复: degree 语义错位(dead=1口,way=2口,branch≥3) /
  入射角置信模型(err=D·δα/cos²α, 替代距离阈值) / 多帧证据确认+冲突撤销 /
  arc 计时 bug(-58% 结论作废重测) / proc_ms 真实施加 / 连续位姿+足迹碰撞检查 /
  StreamNav 唯一认知体(仿真内联副本已删) / has_block 视野化 / from_json / tracker 缓存 /
  open_edge 不再污染 visited
- ✅ 2026-09-24/25 流式探索架构: World/Agent 分离 + 格子标记器 + 一致性守卫 + 剪枝
