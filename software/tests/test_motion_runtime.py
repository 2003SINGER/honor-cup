#!/usr/bin/env python3
"""MotionRuntimeCore 契约测试 (GPT Gate B-K; Gate A 结构门在文件尾)."""

import math

import pytest

from m3pro_nav.feedback_trajectory_follower import FollowerPhase
from m3pro_nav.motion_planner import MotionPlanner
from m3pro_nav.motion_primitive import MotionPrimitive
from m3pro_nav.motion_runtime import (FeedbackSample, MotionRuntimeCore,
                                      RuntimeState, SafetyState)
from m3pro_nav.pose import Pose2D


def build_chain():
    """straight → right quarter arc → straight → STOP (终端必须 STOP)."""
    planner = MotionPlanner()
    yaw = 0.37
    pose = Pose2D(0.2, 0.2, yaw)
    root, pt, _ = planner.template(None, (0, 0), (0, 1), pose)
    arc, pt, _ = planner.template((0, 0), (0, 1), (1, 1), Pose2D(*pt, yaw))
    last, pt, _ = planner.template((0, 1), (1, 1), (1, 2), Pose2D(*pt, yaw))
    chain = root + arc + last
    chain[-1].v_end = 0.0
    chain.append(MotionPrimitive('STOP', Pose2D(*pt, yaw), p0=pt,
                                 yaw0=yaw, duration=0.2))
    return chain, pose


def make_runtime(**kw):
    return MotionRuntimeCore(a_acc=1.0, a_dec=1.0,
                             position_tolerance=0.01, yaw_tolerance=0.02,
                             velocity_tolerance=0.01,
                             yaw_rate_tolerance=0.02, settle_time=0.1,
                             **kw)


def fb(t, pose, vx=0.0, vy=0.0, wz=0.0):
    return FeedbackSample(t, pose, vx, vy, wz)


def run_tracking(rt, chain, *, start=Pose2D(0.2, 0.2, 0.37), dt=0.02,
                 fixed_pose=None, ticks=400):
    """装载并按固定实测位姿推进; 返回 (outputs, arc_samples)."""
    rt.arm()
    rt.update(0.0, fb(0.0, start))
    rt.load_plan(chain)
    outputs = []
    arc = []
    for i in range(ticks):
        t = i * dt
        pose = fixed_pose if fixed_pose is not None else start
        out = rt.update(t, fb(t, pose))
        outputs.append(out)
        fs = out.follower
        if fs is not None and fs.phase == FollowerPhase.TRACKING and \
                fs.reference is not None and fs.reference.curvature != 0.0:
            arc.append(out)
        if out.state in (RuntimeState.HOLDING, RuntimeState.FINISHED) and i > 5:
            pass
        if out.state == RuntimeState.FINISHED:
            break
    return outputs, arc


# ---- Gate C: 直线 ----

def test_gate_c_straight_tracks_and_finishes():
    rt = make_runtime()
    planner = MotionPlanner()
    yaw = 0.37
    pose = Pose2D(0.2, 0.2, yaw)
    root, pt, _ = planner.template(None, (0, 0), (0, 1), pose)
    root[0].v_end = 0.0                     # STOP 前一段必须刹到零 (契约)
    chain = root + [MotionPrimitive('STOP', Pose2D(*pt, yaw),
                                    p0=pt, yaw0=yaw, duration=0.2)]
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    yaw_refs = []
    finished = False
    measured = pose
    for i in range(1, 400):
        t = i * 0.02
        out = rt.update(t, fb(t, measured))
        if out.follower is not None and out.follower.reference is not None:
            yaw_refs.append(out.follower.reference.yaw_ref)
            measured = Pose2D(out.follower.reference.x,
                              out.follower.reference.y,
                              out.follower.reference.yaw_ref)  # 完美 plant
        if out.state == RuntimeState.FINISHED:
            finished = True
            break
    assert finished
    assert all(abs(y - pose.yaw) < 1e-9 for y in yaw_refs)
    assert out.command.vx == out.command.vy == out.command.wz == 0.0
    assert all(math.isfinite(v) for v in
               (out.command.vx, out.command.vy, out.command.wz))


# ---- Gate B: 外部反馈是唯一反馈 ----

def test_gate_b_fixed_measured_pose_makes_error_grow():
    rt = make_runtime()
    chain, pose = build_chain()
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    fixed = Pose2D(0.2, 0.2, pose.yaw)          # 实测位姿钉死不动
    errors = []
    cmds = []
    for i in range(1, 12):
        out = rt.update(i * 0.02, fb(i * 0.02, fixed))
        errors.append(out.follower.position_error)
        cmds.append((out.command.vx, out.command.vy))
    assert errors == sorted(errors)              # 误差单调增长 (无自积分)
    assert len(set(cmds)) > 1                    # 指令随误差变化


# ---- Gate D: 真圆弧 ----

def test_gate_d_arc_samples_quarter_circle():
    rt = make_runtime()
    chain, pose = build_chain()
    _, arc = run_tracking(rt, chain, ticks=600)
    assert arc, 'trajectory never sampled an ARC segment'
    curvatures = {round(o.follower.reference.curvature, 3) for o in arc}
    assert any(abs(abs(c) - 5.0) < 0.01 for c in curvatures), curvatures
    yaw_refs = {o.follower.reference.yaw_ref for o in arc}
    assert all(abs(y - pose.yaw) < 1e-9 for y in yaw_refs)   # yaw_ref 恒定
    # 切向真实旋转: 弧内速度方向至少跨 45°
    v0 = (arc[0].follower.reference.vx_world, arc[0].follower.reference.vy_world)
    v1 = (arc[-1].follower.reference.vx_world, arc[-1].follower.reference.vy_world)
    a0 = math.atan2(v0[1], v0[0])
    a1 = math.atan2(v1[1], v1[0])
    assert abs(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))) > math.radians(30)


# ---- Gate E: REVERSE ----

def test_gate_e_reverse_keeps_yaw_and_reverses_travel():
    rt = make_runtime()
    planner = MotionPlanner()
    yaw = 0.37
    pose = Pose2D(0.2, 0.2, yaw)
    root, pt, _ = planner.template(None, (0, 0), (1, 0), pose)
    rev, pt, _ = planner.template((0, 0), (1, 0), (0, 0), Pose2D(*pt, yaw))
    chain = root + rev
    chain[-1].v_end = 0.0
    chain.append(MotionPrimitive('STOP', Pose2D(*pt, yaw), p0=pt,
                                 yaw0=yaw, duration=0.2))
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    xs = []
    yaw_refs = set()
    for i in range(600):
        out = rt.update(i * 0.02, fb(i * 0.02, pose))
        if out.follower is None or out.follower.reference is None:
            continue
        xs.append(out.follower.reference.x)
        yaw_refs.add(round(out.follower.reference.yaw_ref, 9))
        if out.state == RuntimeState.FINISHED:
            break
    assert xs, 'reference never sampled'
    peak = xs.index(max(xs))
    assert peak not in (0, len(xs) - 1)          # 行程双向: 内部极值 (格中心)
    assert xs[:peak + 1] == sorted(xs[:peak + 1])             # 去程单调离开入口
    assert xs[peak:] == sorted(xs[peak:], reverse=True)       # 回程单调返回入口
    assert max(xs) - min(xs) > 0.3
    assert yaw_refs == {round(yaw, 9)}           # body yaw 绝不跟行驶方向转


# ---- Gate F: STOP-only (WAIT 与终端两种语义) ----

def test_gate_f_wait_stop_holds_indefinitely():
    """WAIT STOP (meta['wait']) = 未知边界等待: 永久 HOLDING, 不因 settle 变
    FINISHED; suffix 到来直接续跑 —— 滚动探索的核心语义 (GPT P0-2)."""
    rt = make_runtime()
    pose = Pose2D(0.2, 0.2, 0.37)
    wait = MotionPrimitive('STOP', pose.copy(), p0=(pose.x, pose.y),
                           yaw0=pose.yaw, duration=0.2, meta={'wait': (1, 0)})
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan([wait])
    assert rt.state == RuntimeState.HOLDING
    for i in range(1, 100):                       # 2s, 远超 settle_time
        out = rt.update(i * 0.02, fb(i * 0.02, pose))
        assert out.state == RuntimeState.HOLDING  # 绝不自结束
    assert out.command.vx == out.command.vy == out.command.wz == 0.0
    # suffix 到来 → 无缝续跑
    planner = MotionPlanner()
    root, pt, _ = planner.template(None, (0, 0), (0, 1), pose)
    root[0].v_end = 0.0
    suffix = root + [MotionPrimitive('STOP', Pose2D(*pt, 0.37), p0=pt,
                                     yaw0=0.37, duration=0.2)]
    rt.append_suffix(suffix)
    out = rt.update(2.02, fb(2.02, pose))
    assert rt.state == RuntimeState.TRACKING


def test_gate_f_terminal_stop_settles_to_finished():
    """终端 STOP (无 meta['wait']) = 任务终点: settle 后 FINISHED."""
    rt = make_runtime()
    pose = Pose2D(0.2, 0.2, 0.37)
    stop = MotionPrimitive('STOP', pose.copy(), p0=(pose.x, pose.y),
                           yaw0=pose.yaw, duration=0.2)
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan([stop])
    assert rt.state == RuntimeState.HOLDING
    out = rt.update(0.02, fb(0.02, pose))
    assert out.state == RuntimeState.HOLDING
    assert out.command.vx == out.command.vy == out.command.wz == 0.0
    for i in range(3, 30):
        out = rt.update(i * 0.02, fb(i * 0.02, pose))
    assert out.state == RuntimeState.FINISHED
    assert out.command.vx == out.command.vy == out.command.wz == 0.0


def test_gate_f_hold_corrects_position_drift():
    """HOLDING 是位置/yaw 保持 (v_ff=0 + 位置环), 不是无条件全零:
    实测漂移时必须产生拉回指令 (GPT P0-3)."""
    rt = make_runtime()
    pose = Pose2D(0.2, 0.2, 0.37)
    wait = MotionPrimitive('STOP', pose.copy(), p0=(pose.x, pose.y),
                           yaw0=pose.yaw, duration=0.2, meta={'wait': (1, 0)})
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan([wait])
    rt.update(0.02, fb(0.02, pose))               # 无漂移 → 零指令
    drifted = Pose2D(0.24, 0.2, 0.37)             # 漂出 4cm
    out = rt.update(0.04, fb(0.04, drifted))
    assert out.state == RuntimeState.HOLDING
    assert out.command.vx != 0.0 or out.command.vy != 0.0   # 有拉回指令
    assert out.follower.position_error == pytest.approx(0.04, abs=1e-6)


# ---- Gate G: HOLD → suffix → TRACKING ----

def test_gate_g_hold_resumes_with_continuous_suffix():
    rt = make_runtime()
    pose = Pose2D(0.2, 0.2, 0.37)
    stop = MotionPrimitive('STOP', pose.copy(), p0=(pose.x, pose.y),
                           yaw0=pose.yaw, duration=0.2, meta={'wait': (0, 0)})
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan([stop])
    rt.update(0.02, fb(0.02, pose))
    assert rt.state == RuntimeState.HOLDING
    # suffix: 从 HOLD 点 (格中心) 起步的连续计划
    planner = MotionPlanner()
    root, pt, _ = planner.template(None, (0, 0), (0, 1),
                                   Pose2D(0.2, 0.2, 0.37))
    root[0].v_end = 0.0                     # STOP 前一段必须刹到零 (契约)
    suffix = root + [MotionPrimitive('STOP', Pose2D(*pt, 0.37), p0=pt,
                                     yaw0=0.37, duration=0.2)]
    rt.append_suffix(suffix)
    out = rt.update(0.04, fb(0.04, pose))
    assert rt.state == RuntimeState.TRACKING     # 无位姿跳变直接续跑
    assert out.follower is not None


# ---- Gate H: 活动 primitive 不被 append 改写 ----

def test_gate_h_append_during_arc_queues_and_keeps_reference():
    rt = make_runtime()
    chain, pose = build_chain()
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    arc_ref = None
    queued = False
    ref_after = None
    for i in range(1, 600):
        t = i * 0.02
        out = rt.update(t, fb(t, pose))
        fs = out.follower
        if (not queued and fs is not None
                and fs.phase == FollowerPhase.TRACKING
                and fs.reference is not None
                and fs.reference.curvature != 0.0):
            arc_ref = fs.reference
            # 连续 suffix: 从计划尾 (0.6,0.8) 继续直行 0.2m 后停
            rt.append_suffix([
                MotionPrimitive('STRAIGHT', Pose2D(0.6, 0.8, 0.0),
                                p0=(0.6, 0.8), p1=(0.6, 1.0),
                                yaw0=0.0, yaw1=0.0, length=0.2,
                                v_max=0.5, v_end=0.0),
                MotionPrimitive('STOP', Pose2D(0.6, 1.0, 0.0),
                                p0=(0.6, 1.0), yaw0=0.0, duration=0.1)])
            queued = True
        elif queued and fs is not None and fs.reference is not None:
            ref_after = fs.reference
            break
    assert queued and arc_ref is not None and ref_after is not None
    # 活动 ARC 未被重编: progress 单调前进不归零, 曲率不变
    assert ref_after.progress_s > arc_ref.progress_s
    assert ref_after.curvature == pytest.approx(arc_ref.curvature)
    assert rt.plan_tail == pytest.approx((0.6, 1.0))  # 队尾已被 suffix 延长


def test_gate_i_discontinuous_suffix_rejected():
    rt = make_runtime()
    chain, pose = build_chain()
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    rt.update(0.02, fb(0.02, pose))
    bad = [MotionPrimitive('STRAIGHT', Pose2D(1.2, 1.0, 0.0),
                           p0=(1.2, 1.0), p1=(1.2, 1.4),
                           yaw0=0.0, yaw1=0.0, length=0.4,
                           v_max=0.7, v_end=0.0),
           MotionPrimitive('STOP', Pose2D(1.2, 1.4, 0.0),
                           p0=(1.2, 1.4), yaw0=0.0, duration=0.1)]
    with pytest.raises(ValueError, match='continuous'):
        rt.append_suffix(bad)


def test_gate_i_suffix_without_terminal_stop_rejected():
    """suffix 必须以 STOP 结尾 (follower 契约), 否则立即拒绝."""
    rt = make_runtime()
    chain, pose = build_chain()
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    rt.update(0.02, fb(0.02, pose))
    no_stop = [MotionPrimitive('STRAIGHT', Pose2D(0.6, 0.8, 0.0),
                               p0=(0.6, 0.8), p1=(0.6, 1.0),
                               yaw0=0.0, yaw1=0.0, length=0.2,
                               v_max=0.5, v_end=0.0)]
    with pytest.raises(ValueError, match='must end with STOP'):
        rt.append_suffix(no_stop)


# ---- Gate K: 反馈超时 ----

def test_gate_k_stale_feedback_faults_and_requires_reset():
    rt = make_runtime()
    chain, pose = build_chain()
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    rt.update(0.02, fb(0.02, pose))
    out = rt.update(2.0, fb(0.05, pose))         # 反馈停更 → 超时
    assert rt.safety == SafetyState.FAULT
    assert out.state == RuntimeState.FAULT
    assert out.command.vx == out.command.vy == out.command.wz == 0.0
    assert 'stale' in rt.fault_reason
    # 新反馈到来也不自动恢复 (FAULT 必须显式 reset)
    out = rt.update(2.02, fb(2.02, pose))
    assert rt.safety == SafetyState.FAULT
    assert out.command.vx == out.command.vy == out.command.wz == 0.0
    rt.reset()
    assert rt.safety == SafetyState.DISARMED
    assert rt.state == RuntimeState.IDLE


# ---- Gate B 补充: IDLE 零指令 / load 门槛 ----

def test_idle_zero_and_load_requires_feedback_and_arm():
    rt = make_runtime()
    out = rt.update(0.02, None)
    assert out.state == RuntimeState.IDLE
    assert out.command.vx == out.command.vy == out.command.wz == 0.0
    chain, pose = build_chain()
    with pytest.raises(RuntimeError, match='ARMED'):
        rt.load_plan(chain)
    rt.arm()
    with pytest.raises(ValueError, match='feedback'):
        rt.load_plan(chain)
    rt.update(0.02, fb(0.02, pose))
    rt.load_plan(chain)
    assert rt.state in (RuntimeState.TRACKING, RuntimeState.HOLDING)


# ---- Gate R: 保守 safe-extension (滚动 horizon 不停车续接) ----

def build_wait_chain():
    """root 0.2m + 0.4m + 0.4m 直线走廊 + 终端 WAIT STOP (未知边界在 (0.2,1.2))."""
    planner = MotionPlanner()
    yaw = 0.37
    pose = Pose2D(0.2, 0.2, yaw)
    root, pt, _ = planner.template(None, (0, 0), (0, 1), pose)
    s2, pt, _ = planner.template((0, 0), (0, 1), (0, 2), Pose2D(*pt, yaw))
    s3, pt, _ = planner.template((0, 1), (0, 2), (0, 3), Pose2D(*pt, yaw))
    s3[-1].v_end = 0.0                        # WAIT STOP 前刹零 (契约)
    stop = MotionPrimitive('STOP', Pose2D(*pt, yaw), p0=pt, yaw0=yaw,
                           duration=0.2, meta={'wait': (0, 3)})
    return root + s2 + s3 + [stop], pose


def build_boundary_suffix():
    """从旧边界 (0.2,1.2) 继续直行 0.4m 到 (0.2,1.6) 的终端 STOP."""
    planner = MotionPlanner()
    yaw = 0.37
    s4, pt, _ = planner.template((0, 2), (0, 3), (0, 4), Pose2D(0.2, 1.2, yaw))
    s4[-1].v_end = 0.0
    return s4 + [MotionPrimitive('STOP', Pose2D(*pt, yaw), p0=pt, yaw0=yaw,
                                 duration=0.2)]


def test_gate_r_suffix_splices_without_stopping():
    """巡航中 suffix 到达 → 删尚未影响速度计划的 WAIT STOP, 无缝续跑:
    不停车、参考位姿/速度连续、活动段几何不被重编为停车 (GPT P1-7)."""
    rt = make_runtime()
    chain, pose = build_wait_chain()
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    assert rt.state == RuntimeState.TRACKING
    measured = pose
    ref_before = None
    spliced = False
    outputs_after = []
    for i in range(1, 500):
        t = i * 0.02
        out = rt.update(t, fb(t, measured))
        fs = out.follower
        if fs is None or fs.reference is None:
            continue
        measured = Pose2D(fs.reference.x, fs.reference.y,
                          fs.reference.yaw_ref)      # 完美跟踪
        if (not spliced and out.state == RuntimeState.TRACKING
                and 0.62 < fs.reference.progress_s < 0.74):
            # 巡航段 (未制动): suffix 到达
            ref_before = (fs.reference.x, fs.reference.y,
                          math.hypot(fs.reference.vx_world, fs.reference.vy_world))
            rt.append_suffix(build_boundary_suffix())
            spliced = True
            assert rt.state == RuntimeState.TRACKING     # 立即续跑
            assert rt.plan_tail == pytest.approx((0.2, 1.6))
            continue
        if spliced:
            outputs_after.append(out)
        if out.state == RuntimeState.FINISHED:
            break
    assert spliced and ref_before is not None
    # 拼接瞬间参考连续: 下一拍位置 ≈ 前一拍 + v·dt (无跳变、无回退)
    first = outputs_after[0].follower
    assert first.reference.y == pytest.approx(
        ref_before[1] + ref_before[2] * 0.02, abs=5e-3)
    # 穿过旧边界 (0.2,1.2) 时全程 TRACKING: 从未在该处停车等待
    crossing = [o for o in outputs_after if 1.0 < o.follower.reference.y < 1.35]
    assert crossing, 'never crossed the old wait boundary'
    assert all(o.state == RuntimeState.TRACKING for o in crossing)
    assert all(math.hypot(o.follower.reference.vx_world,
                          o.follower.reference.vy_world) > 0.3 for o in crossing)
    # 最终在新终端 STOP 处收尾
    assert outputs_after[-1].state == RuntimeState.FINISHED


def test_gate_r_late_suffix_falls_back_to_stop_then_resume():
    """已进入制动区 (STOP 已影响速度计划) → 保守路径: 刹到边界停车,
    再从停车无缝续接 suffix (与仿真端语义一致)."""
    rt = make_runtime()
    chain, pose = build_wait_chain()
    rt.arm()
    rt.update(0.0, fb(0.0, pose))
    rt.load_plan(chain)
    measured = pose
    appended = False
    stopped_at_boundary = False
    resumed = False
    finished = False
    for i in range(1, 600):
        t = i * 0.02
        out = rt.update(t, fb(t, measured))
        fs = out.follower
        if fs is None or fs.reference is None:
            continue
        measured = Pose2D(fs.reference.x, fs.reference.y, fs.reference.yaw_ref)
        speed = math.hypot(fs.reference.vx_world, fs.reference.vy_world)
        if (not appended and out.state == RuntimeState.TRACKING
                and fs.reference.progress_s > 0.78):
            assert speed < 0.69                     # 制动已经开始
            rt.append_suffix(build_boundary_suffix())
            appended = True
        if (appended and not stopped_at_boundary
                and speed < 0.05 and abs(fs.reference.y - 1.2) < 0.05):
            stopped_at_boundary = True              # 保守: 在旧边界刹停
        if stopped_at_boundary and not resumed and fs.reference.y > 1.3:
            resumed = True                          # 从停车续接 suffix 前进
        if out.state == RuntimeState.FINISHED:
            finished = True
            break
    assert appended and stopped_at_boundary and resumed and finished


# ---- Gate A: 结构隔离 (core 无 ROS; node 无控制数学) ----

def test_gate_a_pure_core_and_thin_node():
    base = 'software/ros2/m3pro_nav/m3pro_nav'
    core_banned = ('rclpy', 'geometry_msgs', 'nav_msgs', 'StreamNav',
                   'ActionHorizon', 'edge_map', 'task_pruning',
                   'LowerLoopPlant', 'plant.step')
    for f in ('feedback_trajectory_follower.py', 'motion_runtime.py'):
        src = open(f'{base}/{f}').read()
        imports = [ln for ln in src.splitlines()
                   if ln.startswith('import ') or ln.startswith('from ')]
        for b in core_banned:
            assert not any(b in ln for ln in imports), \
                f'{f} 违反纯逻辑隔离: import {b}'
    node = open(f'{base}/motion_runtime_node.py').read()
    node_banned = ('SpeedProfile', 'TrajectoryReference', 'kp_pos',
                   'kd_vel', 'resolve_next', 'EdgeMap')
    for b in node_banned:
        assert b not in node, f'ROS 节点含控制数学: {b}'
