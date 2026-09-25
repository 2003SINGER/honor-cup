# TODO —— 进度真相板 (2026-09-25 晚, Tier A 运行时修复后)

> 唯一进度真相. 完成项打 ✅ 留证据; 数字必须带 Gate 级别, failures>0 即 INVALID.
> 旧真相板（R3 运动层时代）已归档：`docs/decisions/2026-09-25-旧TODO-R3真相板.md`。

## 当前状态一句话

**Tier A clean baseline 达成（TAG `tier-a-core-baseline`）**：设计真相定稿 +
运行时四连根因修复 + 33 例 Gate 测试全绿 + **Gate M 1000/1000 fullinfo
(avg 158.8s) + 1000/1000 blocks (avg 166.7s, 8/8 全收) 双模式全绿**
（0 abort / 0 collision / 0 unresolved* / 0 wrong_edges）。

## ✅ 本轮完成

- ✅ 文档重写对齐（README / 算法规范 / TODO；旧文档归档 Rejected after re-alignment）
- ✅ 运动层重构：STRAIGHT/ARC/REVERSE/STOP，R=0.2 固定模板，body yaw 恒定
- ✅ P0 死锁修复：STOP 前向格内蹭 NUDGE=0.1m → 离线跨越事件触发 → visit 建立
- ✅ 折返死循环修复：refresh_branch 不再重复登记 visit（visit_count=709 污染根因）
- ✅ 出口格切角碰撞修复：compile_home 出口格走同一套模板，禁止弦线切角
- ✅ 边界守卫恢复：探索永不冲出场地（边界出口 → 折返；进出皆边界 → 停车）
- ✅ entry_side 只来自真实事件/已走拓扑，禁止几何猜测（审查定案落实）
- ✅ 漏收方块修复：链编译后才进入视野的方块 → 清队列重规划（ARC 中途禁清）
- ✅ 结构清理：mazemap 演示块删除；旧 decision_node ROS 骨架归档
- ✅ tests 按新架构重写 33 例（Gate A/B/D/E/G/H/J/K + 结构/依赖方向）全绿
- ✅ Gate M：100 fullinfo + 100 blocks 全绿 → **1000 fullinfo + 1000 blocks 全绿**

## 🔜 下一步（按序）

1. Tier B：定位前端实装（连续可信遮罩 + 已确认墙校正 + 防自证，规范 §8/§9）
   —— 只换 Observation Adapter，core 不准改（Adapter Replacement 测试守门）
2. 速度控制实装（v_max = √(2·a_dec·(d_unknown−margin))，规范 §10）
3. 实车适配层（RealObservationAdapter / RealMotionAdapter），官方 car/ 不污染

## 验收纪律

- Gate A–L（确定性）全绿前禁止 Gate M；Gate M 100 全绿前禁止 1000
- 运行时出现 SPIN/CUT90/CREEP/全局 DFS stack 即结构 FAIL（结构测试断言）
- "SemanticSim 已验证" ≠ "实车已验证"
- (*fullinfo 模式 unresolved=0；blocks 模式允许未探区域，已下结论边全对)
