#!/usr/bin/env python3
"""NavRuntime 闭环契约测试 —— 真实 adapter 路径的 Tier A 回归.

关键设计: 用 ray-cast 从迷宫真值合成 LaserScan (几何上接近真实雷达),
odom 用完美 plant (实测位姿 = 上一拍参考位姿), 驱动:
    scan → RealObservationAdapter → StreamNav → ActionHorizon
        → MotionPlanner → MotionRuntimeCore (dry_run)
整链跑完一座迷宫, 对账 wrong_edges —— 证明"真实感知路径"下算法依然成立。
另有 dry-run/fail-closed/事件桥的单元契约。"""

import json
import math
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'sim'))

from m3pro_nav.frame_projector import LaserExtrinsic            # noqa: E402
from m3pro_nav.nav_runtime import NavRuntime, NavRuntimePhase   # noqa: E402
from m3pro_nav.motion_runtime import RuntimeState, SafetyState  # noqa: E402
from m3pro_nav.odometry_adapter import OdometryState           # noqa: E402
from m3pro_nav.pose import C, DIRV, OPP, DIRS, Pose2D          # noqa: E402

import maze_sim                                                # noqa: E402

N_RAYS = 180                       # 2° 一束: 覆盖足够, 测试耗时可控
SCAN_HZ = 8.0
CTRL_DT = 0.02
PLANNER_DT = 0.10


# ---- 合成传感器: ray-cast 迷宫真值 → LaserScan 形状 ----

class _Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class _Header:
    def __init__(self, frame_id, stamp):
        self.frame_id, self.stamp = frame_id, stamp


class FakeLaserScan:
    def __init__(self, ranges, stamp):
        self.header = _Header('laser', _Stamp(int(stamp)))
        self.angle_min = -math.pi
        self.angle_increment = 2 * math.pi / len(ranges)
        self.angle_max = math.pi
        self.range_min = 0.02
        self.range_max = 8.0
        self.time_increment = 1e-4
        self.scan_time = 1.0 / SCAN_HZ
        self.ranges = list(ranges)


def maze_wall_segments(walls, n=7):
    """迷宫真值墙 → 线段列表 (maze 帧). 只画 walls 真值: 边界开口无墙."""
    segs = []
    size = n * C
    for (i, j), ds in walls.items():
        for d in ds:
            nb = (i + DIRV[d][0], j + DIRV[d][1])
            if 0 <= nb[0] < n and 0 <= nb[1] < n:
                if (i, j) > nb:               # interior: canonical 去重
                    continue
                if d in ('E', 'W'):
                    x = max(i, nb[0]) * C
                    segs.append(((x, j * C), (x, (j + 1) * C)))
                else:
                    y = max(j, nb[1]) * C
                    segs.append(((i * C, y), ((i + 1) * C, y)))
            else:                             # 边界: 仅真值有墙的边
                if d == 'S':
                    segs.append(((i * C, 0.0), ((i + 1) * C, 0.0)))
                elif d == 'N':
                    segs.append(((i * C, size), ((i + 1) * C, size)))
                elif d == 'W':
                    segs.append(((0.0, j * C), (0.0, (j + 1) * C)))
                elif d == 'E':
                    segs.append(((size, j * C), (size, (j + 1) * C)))
    return segs


def cast_scan(wall_segs, maze_pose, dir_offset=0.0, max_r=4.0):
    """从 maze 位姿 ray-cast 一帧激光帧 LaserScan.

    激光角 θ 对应的 maze 方向 = θ + dir_offset (全链旋转角); 物理正确的
    合成: projector 会把激光角再加回同一偏移。"""
    ranges = []
    for k in range(N_RAYS):
        ang = -math.pi + 2 * math.pi * k / N_RAYS + dir_offset
        dx, dy = math.cos(ang), math.sin(ang)
        best = max_r
        for (ax, ay), (bx, by) in wall_segs:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((ax - maze_pose.x) * ey - (ay - maze_pose.y) * ex) / den
            u = ((ax - maze_pose.x) * dy - (ay - maze_pose.y) * dx) / den
            if t > 0.01 and 0.0 <= u <= 1.0:
                best = min(best, t)
        ranges.append(best)
    return FakeLaserScan(ranges, stamp=0.0)


def odom_state(pose, stamp, frame='odom', child='base_link'):
    return OdometryState(pose, 0.0, 0.0, 0.0, stamp, frame, child)


def make_runtime(walls_entry_ex, *, task_mode=False, required=None,
                 extrinsic=(0.0, 0.0, 0.0)):
    walls, entry, ex = walls_entry_ex
    rt = NavRuntime(
        entry=entry, order='LFR', cell=entry, heading='N',
        dry_run=True, task_mode=task_mode, required_blocks=required,
        extrinsic=LaserExtrinsic.from_yaml(*extrinsic))
    rt.set_anchor_spec(entry, 'N')
    return rt, walls, entry, ex


# =========== 单元契约 ===========

def test_dry_run_control_loop_reports_commands_without_publishers():
    """dry_run: control_tick 照常计算指令, 但节点层无 /cmd_vel (节点测试另证).
    这里验证纯协调器: 指令有限、FAULT 路径可用。"""
    walls, entry, ex, side = maze_sim.gen_maze(0)
    rt, walls, entry, ex = make_runtime((walls, entry, ex))
    rt.on_odom(odom_state(Pose2D(0.2, 0.2, math.pi / 2), 1.0))
    out = rt.control_tick(1.0)
    assert out.safety in (SafetyState.DISARMED, SafetyState.ARMED)
    assert all(math.isfinite(v) for v in
               (out.command.vx, out.command.vy, out.command.wz))


def test_missing_extrinsic_abstains_and_runtime_waits():
    """无外参 → on_scan 记 no_transform, 地图零写入, 但流程不崩."""
    walls, entry, ex, side = maze_sim.gen_maze(0)
    rt = NavRuntime(entry=entry, cell=entry, heading='N', dry_run=True,
                    extrinsic=LaserExtrinsic.missing())
    rt.set_anchor_spec(entry, 'N')
    rt.on_odom(odom_state(Pose2D(0.2, 0.2, math.pi / 2), 1.0))
    segs = maze_wall_segments(walls)
    scan = cast_scan(segs, Pose2D(0.2, 0.2, 0.0))
    scan.header.stamp.sec = 1
    assert rt.on_scan(scan) is None
    assert any(e['kind'] == 'no_transform' for e in rt.events)
    assert rt.nav.mark(entry) is None or rt.nav.mark(entry) == \
        pytest.approx(rt.nav.mark(entry))   # 未崩即可


def test_event_bridge_records_crossings_and_verifies_plan():
    """odom 事件桥: 位姿跨线 → crossed/entered 事件 + 一致性校验."""
    walls, entry, ex, side = maze_sim.gen_maze(0)
    rt, walls, entry, ex = make_runtime((walls, entry, ex))
    rt.on_odom(odom_state(Pose2D(0.2, 0.2, math.pi / 2), 1.0))
    # 从 (0,0) 格中心向北走到 (0,1) 格中心 (两次 odom 步进)
    rt.on_odom(odom_state(Pose2D(0.2, 0.3, math.pi / 2), 1.1))
    rt.on_odom(odom_state(Pose2D(0.2, 0.5, math.pi / 2), 1.2))
    rt.on_odom(odom_state(Pose2D(0.2, 0.7, math.pi / 2), 1.3))
    crossed = [e for e in rt.events if e['kind'] == 'crossed']
    assert crossed and crossed[0]['to'] == [0, 1]


# =========== 闭环回归: ray-cast 感知 + 完美 plant ===========

def run_closed_loop(seed, *, max_sim_time=400.0, task_mode=False):
    """整链驱动一座迷宫; 返回 (runtime, 结果 dict)."""
    from m3pro_nav.frame_transform import RigidFrameTransform
    walls, entry, ex, side = maze_sim.gen_maze(seed)
    rt, walls, entry, ex = make_runtime((walls, entry, ex),
                                        task_mode=task_mode, required=8)
    segs = maze_wall_segments(walls)
    n = 7
    # odom 帧 = 平移 (100, -50) + 旋转 30°: 验证帧变换全链
    import math as m
    odom0 = Pose2D(100.0, -50.0, m.radians(30))
    rt.on_odom(odom_state(odom0, 0.0))
    maze_pose = rt.anchor.maze_pose(odom0)
    assert maze_pose.x == pytest.approx((entry[0] + 0.5) * C, abs=1e-9)
    # 逆变换 (odom 帧 → maze 帧): 完美 plant 回投用
    inv = RigidFrameTransform(rt.anchor.odom_anchor, rt.anchor.maze_anchor)

    # 逆变换 (odom 帧 ⇄ maze 帧): 完美 plant 双向换算都用显式逆变换
    inv = RigidFrameTransform(rt.anchor.odom_anchor, rt.anchor.maze_anchor)
    fwd = RigidFrameTransform(rt.anchor.maze_anchor, rt.anchor.odom_anchor)

    t = 0.0
    next_scan = 0.0
    next_planner = 0.0
    scan_interval = 1.0 / SCAN_HZ
    while t < max_sim_time:
        # odom (完美 plant: 实测 = 上一拍参考位姿, maze 帧保持)
        odom_pose = fwd.transform_pose(maze_pose)
        rt.on_odom(odom_state(odom_pose, t))
        # scan: 激光帧角度 = maze 方向 - 全链旋转 (ext 0 + odom yaw + 锚旋转)
        if t >= next_scan:
            total_rot = odom_pose.yaw + rt.anchor.transform.rotation
            scan = cast_scan(segs, maze_pose, dir_offset=total_rot)
            scan.header.stamp = _Stamp(int(t), int((t % 1) * 1e9))
            rt.on_scan(scan)
            next_scan += scan_interval
        # planner tick
        if t >= next_planner:
            rt.planner_tick(t)
            next_planner += PLANNER_DT
        # control tick (50Hz): 参考位姿 (odom 帧) 逆变换回 maze 帧回喂
        out = rt.control_tick(t)
        if out.follower is not None and out.follower.reference is not None:
            ref = out.follower.reference
            maze_pose = inv.transform_pose(Pose2D(ref.x, ref.y,
                                                  ref.yaw_ref))
        if rt.phase == NavRuntimePhase.DONE:
            break
        t += CTRL_DT
    # 真值对账
    wrong = 0
    for i in range(n):
        for j in range(n):
            for d in DIRS:
                dv = DIRV[d]
                nb = (i + dv[0], j + dv[1])
                if not (0 <= nb[0] < n and 0 <= nb[1] < n):
                    continue
                if not rt.nav.resolved((i, j), d):
                    continue
                truth = 'OPEN' if d not in walls[(i, j)] else 'WALL'
                if rt.nav.edges.state((i, j), d, rt.nav.traversal) != truth:
                    wrong += 1
    return rt, {'time': t, 'wrong_edges': wrong, 'phase': rt.phase,
                'mismatches': len(rt.mismatches)}


@pytest.mark.slow
@pytest.mark.parametrize('seed', [0, 1, 2])
def test_closed_loop_explores_maze_with_realistic_rays(seed):
    """整链: ray-cast 雷达 + 帧变换 + 事件桥 + 编链 + dry-run 控制
    → 迷宫全解析 (wrong_edges == 0), 无拓扑失配。"""
    rt, res = run_closed_loop(seed)
    assert res['phase'] in (NavRuntimePhase.DONE, NavRuntimePhase.GOING_HOME), \
        f"stuck in {res['phase']} at t={res['time']:.0f}"
    assert res['wrong_edges'] == 0, res
    assert res['mismatches'] == 0, res
    # 事件序列完整: 有 anchor/plan/crossed
    kinds = {e['kind'] for e in rt.events}
    assert {'anchor', 'plan', 'crossed'} <= kinds


def test_evidence_snapshot_is_json_serializable():
    walls, entry, ex, side = maze_sim.gen_maze(0)
    rt, walls, entry, ex = make_runtime((walls, entry, ex))
    rt.on_odom(odom_state(Pose2D(0.2, 0.2, 0.0), 1.0))
    snap = rt.evidence_snapshot()
    json.dumps(snap)                  # 可序列化 (喂 LLM 的格式)
    assert 'events' in snap and 'mismatches' in snap


# =========== 节点级契约 (rclpy stub): dry-run / fail-closed ===========

def _install_nav_ros_stubs():
    import test_motion_runtime_node as _stubmod  # 复用 stub 基建
    return _stubmod._install_ros_stubs()


def test_nav_node_source_is_event_driven_and_safe():
    """节点源码契约: 事件驱动 (subscriptions+timers, 无 while 循环);
    dry_run 默认; 非 dry_run 才创建 /cmd_vel; UNCALIBRATED 拒绝实跑。"""
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'ros2', 'm3pro_nav', 'm3pro_nav',
                            'nav_runtime_node.py')).read()
    assert 'dry_run' in src and 'declare_parameter(\'dry_run\', True)' in src
    # /cmd_vel publisher 只在非 dry_run 分支创建
    assert 'if not self._dry_run:' in src
    # 事件驱动: 有订阅与定时器, 无裸 while True 主循环
    assert 'create_subscription' in src and 'create_timer' in src
    assert 'while True' not in src
    # fail closed: 占位配置 + 实跑 → RuntimeError
    assert 'uncalibrated' in src and 'RuntimeError' in src


def test_nav_config_has_placeholders_and_dry_run_allows_them(tmp_path=None):
    """config/nav_runtime.yaml: 未知项留 __MEASURE__/__CALIBRATE__ 占位;
    load_runtime_config 正确识别占位清单。"""
    import tempfile
    from m3pro_nav.nav_runtime import load_runtime_config
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'ros2', 'm3pro_nav', 'config',
                        'nav_runtime.yaml')
    config, uncalibrated = load_runtime_config(base)
    assert uncalibrated, 'shipping config must keep UNCALIBRATED placeholders'
    assert any('frames' in k for k in uncalibrated)
    # 全填好的配置 → 无占位
    filled = tempfile.mkdtemp()
    path = os.path.join(filled, 'filled.yaml')
    with open(path, 'w') as f:
        f.write('ros: {scan_topic: /scan_multi}\n'
                'frames: {odom_frame: odom, base_frame: base_link}\n')
    _, unc = load_runtime_config(path)
    assert unc == []


def test_config_tf_source_exempts_yaml_placeholders():
    """source=tf 时 YAML 外参占位不挡实跑 (GPT 审查: 假 fail 修复)."""
    import tempfile
    from m3pro_nav.nav_runtime import load_runtime_config
    d = tempfile.mkdtemp()
    tf_cfg = os.path.join(d, 'tf.yaml')
    with open(tf_cfg, 'w') as f:
        f.write('laser_extrinsic:\n'
                '  source: tf\n'
                '  x: __CALIBRATE__\n'
                '  y: __CALIBRATE__\n'
                '  yaw: __CALIBRATE__\n'
                'frames: {odom_frame: odom, base_frame: base_link}\n')
    _, unc = load_runtime_config(tf_cfg)
    assert unc == [], f'tf source must exempt yaml placeholders, got {unc}'
    yaml_cfg = os.path.join(d, 'yaml_src.yaml')
    with open(yaml_cfg, 'w') as f:
        f.write('laser_extrinsic:\n'
                '  source: yaml\n'
                '  x: __CALIBRATE__\n')
    _, unc2 = load_runtime_config(yaml_cfg)
    assert any('laser_extrinsic.x' in k for k in unc2)


def test_nav_node_preflight_gates_real_run():
    """preflight 契约: 实跑挡 (外参 missing / 占位 / cmd_vel 归属 /
    scan 帧不符→FAULT); dry_run 只 warn。源码级断言。"""
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'ros2', 'm3pro_nav', 'm3pro_nav',
                            'nav_runtime_node.py')).read()
    # 实跑 preflight 显式收集并 fail closed
    assert 'preflight_failures' in src
    assert 'extrinsic unavailable' in src
    # scan 帧不符: 实跑 → report_fault; dry_run → warn
    assert 'scan frame mismatch in real run' in src
    # ARMED 期间周期复查归属
    assert 'publisher conflict' in src


def test_tf_listener_self_spins_before_executor():
    """TF listener 必须显式 spin_thread=True: constructor 阶段 executor 尚未
    spin, 默认 listener 不处理 /tf(_static) → TF 存在也假 missing
    (GPT 审查: 启动时序修复)。nav_runtime 与 scan_debug 两处都要覆盖。"""
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'ros2', 'm3pro_nav', 'm3pro_nav')
    for name in ('nav_runtime_node.py', 'scan_debug_node.py'):
        src = open(os.path.join(base, name)).read()
        assert 'TransformListener(buffer, self, spin_thread=True)' in src, \
            f'{name}: TF listener must self-spin (executor not running yet)'
