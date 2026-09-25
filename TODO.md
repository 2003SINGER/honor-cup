# TODO —— 进度真相板 (2026-09-25, 原始设计对齐后)

> 唯一进度真相. 完成项打 ✅ 留证据; 数字必须带 Gate 级别, failures>0 即 INVALID.
> 旧真相板（R3 运动层时代）已归档：`docs/decisions/2026-09-25-旧TODO-R3真相板.md`。

## 当前状态一句话

设计真相已重写（`docs/design/算法规范.md`，依据用户原始方案 + 2026-09-25
GPT 对齐对话）；代码按新架构重构进行中（WIP `386a067`），存在一个已知
P0 死锁，Gate 未跑。

## ✅ 本轮完成

- ✅ 文档审计：README / 旧算法规范 / 旧 TODO 全部按新架构重写或归档
  （Rejected after requirement re-alignment），旧文本不再误导后续实现
- ✅ GPT 对话存档入库：`docs/decisions/2026-09-25-原始设计对齐-GPT对话存档.md`
- ✅ 运动层重构（WIP `386a067`）：STRAIGHT/ARC/REVERSE/STOP 四原语；
  R=0.2 固定模板；body yaw 恒定（executor 只积分位置）；REVERSE 倒穿父边；
  抓取停格中心；事件检测支持"格线停靠后离线=真实跨越"
- ✅ StreamNav 去 DFS：branch 局部状态机（parent/children/next），
  runtime 接 compile_chain / compile_home，旧 dfs_explorer / known_horizon /
  move_intent 删除

## 🔜 下一步（按序）

1. **P0 解死锁**：STOP 落点内移进邻格（符合规范 §11 WAIT 位置），
   或事件检测支持"到达即进入"语义 → seed0 起全 seed 冒烟
2. P1 tests 按 Gate A–L 重写（旧 DFS/MoveIntent 测试已失效）
3. P1 Gate M：100 seeds fullinfo + 100 blocks → 0 abort/0 collision/
   0 wrong_edges → 再 1000
4. P2 定位前端 Tier B 实装（连续可信遮罩 + 已确认墙校正 + 防自证，规范 §8/§9）
5. P2 速度控制实装（v_max = √(2·a_dec·(d_unknown−margin))，规范 §10）

## 验收纪律

- Gate A–L（确定性）全绿前禁止 Gate M；Gate M 100 全绿前禁止 1000
- 运行时出现 SPIN/CUT90/CREEP/全局 DFS stack 即结构 FAIL（结构测试断言）
- "SemanticSim 已验证" ≠ "实车已验证"
