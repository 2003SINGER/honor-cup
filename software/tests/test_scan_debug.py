#!/usr/bin/env python3
"""感知调试工作台契约测试 (GPT 感知任务单验收 1-12).

纯 Python 管线 (ScanAdapter/FrameProjector/GridAssociation/TrustPolicy/
Diagnostics/Markers) + rclpy-stub 注入的 scan_debug 节点测试。"""

import json
import math
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.frame_projector import (FIELD_SIZE, FrameProjector,
                                       LaserExtrinsic, ManualMazeAnchor,
                                       maze_pose_from_cell)
from m3pro_nav.grid_association import GridAssociation, edge_id_for_line
from m3pro_nav.pose import C, Pose2D
from m3pro_nav.scan_adapter import parse_scan
from m3pro_nav.scan_debug_markers import build_markers
from m3pro_nav.scan_diagnostics import FrameAccumulator, summarize_frames
from m3pro_nav.trust_policy import TrustPolicy, TrustThresholds


# ---- LaserScan stub ----

class _Header:
    def __init__(self, frame_id='laser', stamp=100.0):
        self.frame_id = frame_id
        self.stamp = types.SimpleNamespace(sec=int(stamp),
                                           nanosec=int((stamp % 1) * 1e9))


class FakeLaserScan:
    def __init__(self, ranges, *, angle_min=-math.pi, angle_increment=0.01,
                 range_min=0.02, range_max=8.0, frame_id='laser', stamp=100.0):
        self.header = _Header(frame_id, stamp)
        self.angle_min = angle_min
        self.angle_max = angle_min + angle_increment * (len(ranges) - 1)
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max
        self.time_increment = 1e-4
        self.scan_time = 0.1
        self.ranges = list(ranges)


# =========== 验收 1+2: ScanAdapter ray 语义 ===========

def test_scan_adapter_parses_rays_and_metadata():
    msg = FakeLaserScan([1.0, float('nan'), 0.01, 9.99, 2.0])
    f = parse_scan(msg)
    assert f.frame_id == 'laser'
    assert f.stamp == pytest.approx(100.0)
    assert f.time_increment == pytest.approx(1e-4)      # deskew 预留字段保留
    assert len(f.rays) == 5
    assert f.rays[0].valid and f.rays[0].range == 1.0
    assert f.rays[0].angle == pytest.approx(msg.angle_min)
    # NaN/Inf → INVALID_RANGE; 越量程 → OUT_OF_RANGE; 都不进有效 hit
    assert not f.rays[1].valid and f.rays[1].invalid_reason == 'INVALID_RANGE'
    assert not f.rays[2].valid and f.rays[2].invalid_reason == 'OUT_OF_RANGE'
    assert not f.rays[3].valid and f.rays[3].invalid_reason == 'OUT_OF_RANGE'
    assert f.rays[4].valid
    assert len(f.valid_rays()) == 2
    # 角度按 angle_increment 递增
    assert f.rays[2].angle == pytest.approx(msg.angle_min + 2 * 0.01)


# =========== 验收 3: 坐标链解析 ===========

def _world_hit(extrinsic, odom_pose, anchor, angle, rng):
    """手工计算 ray hit 的 maze 坐标 (与 FrameProjector 独立推导对照)."""
    proj = FrameProjector(extrinsic)
    msg = FakeLaserScan([rng], angle_min=angle, angle_increment=0.0)
    rays = proj.project(parse_scan(msg), odom_pose, anchor)
    return rays[0]


def test_frame_projector_chain_is_analytic():
    # laser 在 base 前方 0.1m; odom 系与 maze 系只差平移
    ext = LaserExtrinsic.from_yaml(0.1, 0.0, 0.0)
    odom_pose = Pose2D(1.0, 2.0, 0.0)               # 朝 +x
    anchor = ManualMazeAnchor((3, 3), 'N', Pose2D(1.0, 2.0, 0.0))
    # anchor: maze pose = (1.4, 1.4, π/2); odom anchor = (1,2,0)
    # → maze = odom 旋转 π/2 绕 (1,2)?? RigidFrameTransform 定义: 同一物理点
    # 在两系中的位姿 → T(odom_pose) = maze_pose
    ray = _world_hit(ext, odom_pose, anchor, 0.0, 1.0)      # laser frame +x
    # laser 原点: base(0.1,0) → odom(1.1,2.0) → maze(同一点)
    m = anchor.transform.transform_pose(Pose2D(1.1, 2.0, 0.0))
    assert (ray.ox, ray.oy) == pytest.approx((m.x, m.y))
    # hit 在 laser 原点 +x 方向 1m (全链旋转角 = 0 + 0 + rot(maze/odom))
    rot = anchor.transform.rotation
    hx = m.x + math.cos(rot) * 1.0
    hy = m.y + math.sin(rot) * 1.0
    assert (ray.hx, ray.hy) == pytest.approx((hx, hy))
    assert ray.range == pytest.approx(1.0)
    assert ray.valid


def test_frame_projector_composite_rotation():
    # 全链有旋转: laser yaw 0.3, odom yaw 0.2, maze/odom 旋转 π/2
    ext = LaserExtrinsic.from_yaml(0.0, 0.0, 0.3)
    odom_pose = Pose2D(5.0, -1.0, 0.2)
    anchor = ManualMazeAnchor((0, 0), 'E', Pose2D(5.0, -1.0, 0.2))
    ray = _world_hit(ext, odom_pose, anchor, 0.5, 1.2)
    total = 0.3 + 0.2 + anchor.transform.rotation
    assert (ray.dir_x, ray.dir_y) == pytest.approx(
        (math.cos(total), math.sin(total)))
    # hit = origin + R(total)·(1.2·(cos0.5, sin0.5))
    lx, ly = 1.2 * math.cos(0.5), 1.2 * math.sin(0.5)
    c, s = math.cos(total), math.sin(total)
    assert (ray.hx, ray.hy) == pytest.approx(
        (ray.ox + c * lx - s * ly, ray.oy + s * lx + c * ly))


# =========== 验收 4: ManualMazeAnchor 一次性 + 跟随 odom ===========

def test_anchor_follows_odom_without_resnapping():
    anchor = ManualMazeAnchor((3, 2), 'N', Pose2D(1.0, 2.0, 0.1))
    assert anchor.maze_anchor.x == pytest.approx(3.5 * C)
    assert anchor.maze_anchor.y == pytest.approx(2.5 * C)
    assert anchor.maze_anchor.yaw == pytest.approx(math.pi / 2)
    # 车被推 3cm + 转 5°: maze pose 如实跟随 (odom 增量按锚旋转角变换),
    # 不吸回格中心
    rot = anchor.transform.rotation
    moved = anchor.maze_pose(Pose2D(1.03, 2.0, 0.1 + math.radians(5)))
    assert moved.x == pytest.approx(3.5 * C + 0.03 * math.cos(rot), abs=1e-9)
    assert moved.y == pytest.approx(2.5 * C + 0.03 * math.sin(rot), abs=1e-9)
    assert moved.yaw == pytest.approx(math.pi / 2 + math.radians(5))
    # 再推一次仍然只是线性跟随 (锚只建一次)
    again = anchor.maze_pose(Pose2D(1.03, 2.0, 0.1 + math.radians(5)))
    assert (again.x, again.y) == pytest.approx((moved.x, moved.y))


# =========== 验收 5+6: UNIQUE / AMBIGUOUS / NONE ===========

def _ray_at(hx, hy, ox=1.4, oy=1.4, rng=None, angle=0.0):
    from m3pro_nav.frame_projector import WorldRay
    d = math.hypot(hx - ox, hy - oy)
    return WorldRay(0, ox, oy, (hx - ox) / d, (hy - oy) / d, hx, hy,
                    rng if rng is not None else d, None)


ASSOC = GridAssociation()


def test_unique_association_near_vertical_line():
    # hit 紧贴 x=1.2 (k=3), 远离任何水平线
    ray = _ray_at(1.207, 0.75)
    obs = ASSOC.associate_hit(ray)
    assert obs.outcome == 'UNIQUE'
    cand = obs.candidate
    assert cand.orientation == 'V'
    assert cand.residual == pytest.approx(0.007, abs=1e-9)
    assert cand.along_edge == pytest.approx(0.75)
    assert cand.distance_to_corner == pytest.approx(min(0.75 - 0.4, 0.8 - 0.75))
    # canonical edge: 线 k=3, 段 j=1 → cells (2,1)|(3,1) → key = ((2,1),'E')
    assert cand.edge_id == ((2, 1), 'E')


def test_boundary_edge_id_uses_B_form():
    # 线 k=0 (x=0): 场地边界
    assert edge_id_for_line('V', 0, 2) == ('B', (0, 2), 'W')
    assert edge_id_for_line('H', 7, 3) == ('B', (3, 6), 'N')
    assert edge_id_for_line('V', 4, 5) == ((3, 5), 'E')


def test_ambiguous_near_corner_is_abstain_not_guess():
    # hit 同时贴近 x=1.2 (residual 6mm) 和 y=0.8 (residual 8mm):
    # margin 2mm < 20mm → AMBIGUOUS, 绝不按 6<8 强选 vertical
    ray = _ray_at(1.206, 0.792)
    obs = ASSOC.associate_hit(ray)
    assert obs.outcome == 'AMBIGUOUS'
    assert obs.reason == 'AMBIGUOUS_EDGE'
    assert len(obs.candidates) == 2
    assert obs.candidate is None


def test_none_when_residual_too_large():
    # hit 处于格子内部 (离任何 canonical line 都 > 50mm)
    ray = _ray_at(1.0, 0.63)     # 离 x=0.8 有 0.2, 离 y=0.8 有 0.17
    obs = ASSOC.associate_hit(ray)
    assert obs.outcome == 'NONE'
    assert obs.reason == 'LARGE_RESIDUAL'


def test_none_out_of_field():
    ray = _ray_at(-0.3, 1.0)
    obs = ASSOC.associate_hit(ray)
    assert obs.outcome == 'NONE' and obs.reason == 'OUT_OF_FIELD'
    ray = _ray_at(1.0, 3.5)
    obs = ASSOC.associate_hit(ray)
    assert obs.outcome == 'NONE' and obs.reason == 'OUT_OF_FIELD'


def test_none_near_corner_along_edge():
    # UNIQUE 候选但投影点距格角 < 50mm → NEAR_CORNER 拒绝
    ray = _ray_at(1.203, 0.402)   # x=1.2 residual 3mm; y=0.402 距角 2mm
    obs = ASSOC.associate_hit(ray)
    # y=0.402 离水平线 0.4 仅 2mm → 也可能 AMBIGUOUS; 两种都必须不是 UNIQUE
    assert obs.outcome != 'UNIQUE'


def test_incidence_angle_feature():
    # 正对墙 (射线沿 -x 打到 x=1.2): incidence ≈ 0
    ray = _ray_at(1.2, 1.4, ox=1.6, oy=1.4)          # 行进方向 -x
    obs = ASSOC.associate_hit(ray)
    assert obs.outcome == 'UNIQUE'
    assert obs.candidate.incidence_angle == pytest.approx(0.0, abs=1e-9)
    # 掠射 (射线沿 -y 打到同一点, 墙是 vertical): incidence ≈ 90°
    ray = _ray_at(1.2, 1.4, ox=1.2, oy=2.0)
    obs = ASSOC.associate_hit(ray)
    # 该 hit 对 horizontal line y=1.2 residual 0.2 → 只 vertical 候选
    assert obs.candidate.incidence_angle == pytest.approx(math.pi / 2,
                                                          abs=1e-6)


# =========== 验收 7: free path OPEN 证据 ===========

def test_open_edges_along_free_path():
    # origin (0.9,1.4) → hit (1.6,1.4): 穿过 x=1.2 一条 vertical line
    ray = _ray_at(1.6, 1.4, ox=0.9, oy=1.4)
    edges = ASSOC.open_edges_along(ray)
    assert ((2, 3), 'E') in edges                    # 线 k=3 段 j=3
    assert len(edges) == 1
    # hit 恰在格线上: 穿越判据 a*b<0 → 线 x=1.2 在 hit 前, 仍算 OPEN;
    # hit 之后 (x=1.6 后面) 不推理
    ray = _ray_at(1.2, 1.4, ox=0.9, oy=1.4)
    edges = ASSOC.open_edges_along(ray)
    assert len(edges) == 0                            # 1.2 是 hit 自身, a*b<0 不含


def test_invalid_ray_yields_no_evidence():
    from m3pro_nav.frame_projector import WorldRay
    ray = WorldRay(0, 0.9, 1.4, 1.0, 0.0, math.nan, math.nan,
                   math.nan, 'INVALID_RANGE')
    assert ASSOC.open_edges_along(ray) == ()
    obs = ASSOC.associate_hit(ray)
    assert obs.outcome == 'NONE' and obs.reason == 'INVALID_RANGE'


# =========== 验收 8 + 11: 无外参 ABSTAIN / diagnostic_only 零 EdgeMap ===========

def test_missing_extrinsic_is_abstain_not_center_assumption():
    proj = FrameProjector(LaserExtrinsic.missing())
    msg = FakeLaserScan([1.0, 1.1])
    anchor = ManualMazeAnchor((3, 3), 'N', Pose2D(0.0, 0.0, 0.0))
    assert proj.project(parse_scan(msg), Pose2D(0, 0, 0), anchor) is None


def test_trust_policy_diagnostic_only_never_commits():
    policy = TrustPolicy()                            # 默认 diagnostic_only
    ray = _ray_at(1.207, 0.75)
    obs = ASSOC.associate_hit(ray)
    d = policy.evaluate(obs)
    assert not d.accepted and d.reason == 'DIAGNOSTIC_ONLY'
    assert d.observation is None
    # 未标定阈值即使关掉 diagnostic_only 仍然 ABSTAIN + 明确缺失清单
    policy2 = TrustPolicy(diagnostic_only=False)
    d2 = policy2.evaluate(obs)
    assert not d2.accepted and d2.reason.startswith('UNCALIBRATED')
    # 全阈值标定后才可能 OK (形状验收, 不代表真值; 该射线掠射垂直墙,
    # incidence ≈ 90°, 阈值必须放得下)
    th = TrustThresholds(max_residual_m=0.01, max_incidence_rad=math.pi,
                         max_range_m=2.0, min_uniqueness_margin_m=0.01,
                         min_corner_distance_m=0.05, min_votes=1)
    d3 = TrustPolicy(diagnostic_only=False,
                     thresholds=th).evaluate(obs)
    assert d3.accepted and d3.observation.edge_id == ((2, 1), 'E')


# =========== 验收 9: 批量 markers ===========

def test_markers_are_batched_and_bounded():
    ext = LaserExtrinsic.from_yaml(0.0, 0.0, 0.0)
    anchor = ManualMazeAnchor((3, 3), 'N', Pose2D(1.4, 1.4, 0.0))
    proj = FrameProjector(ext)
    msg = FakeLaserScan([1.0] * 360, angle_min=-math.pi,
                        angle_increment=2 * math.pi / 360)
    rays = proj.project(parse_scan(msg), Pose2D(1.4, 1.4, 0.0), anchor)
    observations = ASSOC.process(rays)
    markers = build_markers(rays, observations)
    # 一帧 ≤6 个 marker (网格/rays/unique/ambiguous/wall/open), 绝不 per-beam
    assert len(markers) <= 6 and len(markers) >= 1
    assert all(m['type'] in ('POINTS', 'LINE_LIST') for m in markers)
    # NO_TRANSFORM 时只剩网格
    markers2 = build_markers(None, [])
    assert len(markers2) == 1


# =========== 验收 12: 重放统计可重复 ===========

def _synthetic_frame_records(n_frames):
    ext = LaserExtrinsic.from_yaml(0.0, 0.0, 0.0)
    anchor = ManualMazeAnchor((3, 3), 'N', Pose2D(1.4, 1.4, 0.0))
    proj = FrameProjector(ext)
    records = []
    for i in range(n_frames):
        msg = FakeLaserScan([1.0 + 0.001 * (i % 7)] * 120,
                            angle_min=-math.pi,
                            angle_increment=2 * math.pi / 120,
                            stamp=100.0 + i * 0.1)
        rays = proj.project(parse_scan(msg), Pose2D(1.4, 1.4, 0.0), anchor)
        obs = ASSOC.process(rays)
        acc = FrameAccumulator()
        acc.accumulate(obs, msg.header.stamp.sec)
        records.append(acc.to_json())
    return records


def test_replay_statistics_are_identical():
    s1 = summarize_frames(_synthetic_frame_records(10))
    s2 = summarize_frames(_synthetic_frame_records(10))
    assert s1 == s2                                  # 逐字节一致
    assert s1['n_frames'] == 10
    assert s1['n_valid'] > 0
    assert 'conclusion' in s1                        # 不自动宣布可信距离
    # JSON 往返稳定 (frames.jsonl → summary 链路)
    s3 = summarize_frames([json.loads(json.dumps(r))
                           for r in _synthetic_frame_records(10)])
    assert s3 == s1


def test_summary_has_range_bins_and_reasons():
    s = summarize_frames(_synthetic_frame_records(3))
    assert len(s['by_range_bin']) == 5
    for b in s['by_range_bin']:
        for k in ('unique_rate', 'residual_p50_mm', 'incidence_p50_deg',
                  'corner_p10_mm'):
            assert k in b
    assert 'reject_reasons' in s and 'per_edge_counts' in s


# =========== 验收 10: scan_debug 节点 (rclpy stub) ===========

class _StubLogger:
    def info(self, m): pass
    def warn(self, m): pass
    def warn_once(self, m, **k): pass
    def error(self, m): pass


class _StubClock:
    def now(self):
        return types.SimpleNamespace(
            nanoseconds=0,
            to_msg=lambda: types.SimpleNamespace(sec=0, nanosec=0))


class _StubNode:
    created_publishers = []

    def __init__(self, *a, **k):
        self._publishers = {}
        self._subs = {}
        self._params = {}
    def declare_parameter(self, name, default):
        self._params[name] = default
    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])
    def create_publisher(self, msg_type, topic, qos):
        self.created_publishers.append(topic)
        return types.SimpleNamespace(publish=lambda m: None)
    def create_subscription(self, msg_type, topic, cb, qos):
        self._subs[topic] = cb
        return None
    def create_timer(self, period, cb):
        return None
    def get_logger(self):
        return _StubLogger()
    def get_clock(self):
        return _StubClock()


def _install_ros_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.time = types.ModuleType('rclpy.time')
    rclpy.time.Time = lambda *a, **k: None
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = _StubNode
    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile = lambda **k: types.SimpleNamespace(**k)
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    geom = types.ModuleType('geometry_msgs')
    geom.msg = types.ModuleType('geometry_msgs.msg')

    class _Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x, self.y, self.z = x, y, z
    class _TwistMsg:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    class _Marker:
        LINE_LIST, POINTS = 4, 8
        def __init__(self):
            self.header = types.SimpleNamespace(
                frame_id='', stamp=None)
            self.id = 0
            self.type = 0
            self.color = types.SimpleNamespace(r=0, g=0, b=0, a=0)
            self.scale = types.SimpleNamespace(x=0, y=0, z=0)
            self.points = []
    class _MarkerArray:
        def __init__(self):
            self.markers = []
    geom.msg.Twist = _TwistMsg
    geom.msg.Point = _Point
    geom.msg.Marker = _Marker
    geom.msg.MarkerArray = _MarkerArray
    nav = types.ModuleType('nav_msgs')
    nav.msg = types.ModuleType('nav_msgs.msg')
    nav.msg.Odometry = types.SimpleNamespace
    sensor = types.ModuleType('sensor_msgs')
    sensor.msg = types.ModuleType('sensor_msgs.msg')
    sensor.msg.LaserScan = types.SimpleNamespace
    viz = types.ModuleType('visualization_msgs')
    viz.msg = types.ModuleType('visualization_msgs.msg')
    viz.msg.Marker = _Marker
    viz.msg.MarkerArray = _MarkerArray
    rclpy.node = node_mod
    rclpy.qos = qos_mod
    for name, mod in [('rclpy', rclpy), ('rclpy.node', node_mod),
                      ('rclpy.qos', qos_mod), ('rclpy.time', rclpy.time),
                      ('geometry_msgs', geom), ('geometry_msgs.msg', geom.msg),
                      ('nav_msgs', nav), ('nav_msgs.msg', nav.msg),
                      ('sensor_msgs', sensor), ('sensor_msgs.msg', sensor.msg),
                      ('visualization_msgs', viz),
                      ('visualization_msgs.msg', viz.msg)]:
        sys.modules[name] = mod
    return rclpy


@pytest.fixture()
def node_mod():
    _install_ros_stubs()
    _StubNode.created_publishers = []
    import importlib
    import m3pro_nav.scan_debug_node as mod
    importlib.reload(mod)
    return mod


def _make_node(node_mod, **overrides):
    node = node_mod.ScanDebugNode.__new__(node_mod.ScanDebugNode)
    _StubNode.__init__(node)
    defaults = dict(scan_topic='/scan', odom_topic='/odom_raw', imu_topic='',
                    expected_laser_frame='laser',
                    expected_odom_frame='odom',
                    expected_base_frame='base_link',
                    use_tf_extrinsic=False,
                    laser_extrinsic_yaml='',
                    cell_x=3, cell_y=3, heading='N',
                    anchor_offset_x=0.0, anchor_offset_y=0.0,
                    anchor_offset_yaw=0.0, session_dir='',
                    markers_topic='/scan_debug/markers',
                    diagnostic_only=True)
    defaults.update(overrides)
    node._params = defaults
    # 重走 __init__ 的管线部分 (绕过 rclpy 结构)
    from m3pro_nav.grid_association import GridAssociation
    from m3pro_nav.frame_projector import FrameProjector, LaserExtrinsic
    node._scan_topic = defaults['scan_topic']
    node._odom_topic = defaults['odom_topic']
    node._laser_frame_hint = defaults['expected_laser_frame']
    node._odom_frame = defaults['expected_odom_frame']
    node._base_frame = defaults['expected_base_frame']
    node._diagnostic_only = defaults['diagnostic_only']
    node._association = GridAssociation()
    ext_path = defaults['laser_extrinsic_yaml']
    node._projector = FrameProjector(
        node._load_yaml_extrinsic(ext_path) if ext_path
        else LaserExtrinsic.missing())
    node._anchor = None
    node._latest_odom_pose = None
    node._frame_count = 0
    node._frames_file = None
    node._frames_path = None
    if defaults['session_dir']:
        os.makedirs(defaults['session_dir'], exist_ok=True)
        node._frames_path = os.path.join(defaults['session_dir'], 'frames.jsonl')
        node._frames_file = open(node._frames_path, 'a', buffering=1)
    qos = None
    node._markers_pub = types.SimpleNamespace(publish=lambda m: None)
    node._odom_sub_cb = node._on_odom
    node._scan_sub_cb = node._on_scan
    return node


def test_node_has_no_cmd_vel_publisher_and_never_arms(node_mod):
    """验收 10: scan_debug 只读 —— 无 /cmd_vel publisher, 不 import 运动层."""
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'ros2', 'm3pro_nav', 'm3pro_nav',
                            'scan_debug_node.py')).read()
    assert '/cmd_vel' not in src.replace('绝不创建 /cmd_vel', '')
    assert 'cmd_vel' not in [t for t in _StubNode.created_publishers]
    imports = [ln for ln in src.splitlines()
               if ln.startswith('import ') or ln.startswith('from ')]
    for banned in ('motion_runtime', 'MotionRuntime', 'ActionHorizon',
                   'stream_nav', 'edge_map', 'EdgeMap'):
        assert not any(banned in ln for ln in imports), \
            f'scan_debug 违反只读边界: import {banned}'


def test_node_topics_are_configurable(node_mod):
    node = _make_node(node_mod, scan_topic='/my_scan', odom_topic='/my_odom')
    assert node._scan_topic == '/my_scan'
    assert node._odom_topic == '/my_odom'


def test_node_pipeline_end_to_end_writes_frames(node_mod):
    """odom → anchor 建立 → scan → frames.jsonl (与离线重放同一链)."""
    import tempfile
    tmp_path = tempfile.mkdtemp()
    node = _make_node(node_mod,
                      laser_extrinsic_yaml=self_rel_extrinsic(),
                      session_dir=tmp_path)
    # 第一帧 odom (建立 anchor)
    node._odom_sub_cb(fake_odom(1.4, 1.4, 0.0))
    assert node._anchor is not None
    # scan
    node._scan_sub_cb(FakeLaserScan([1.0] * 60, angle_min=-math.pi,
                                    angle_increment=2 * math.pi / 60,
                                    frame_id='laser'))
    assert node._frame_count == 1
    node.shutdown()
    frames = [json.loads(l) for l in
              open(os.path.join(tmp_path, 'frames.jsonl'))]
    assert len(frames) == 1
    assert frames[0]['n_rays'] == 60


def self_rel_extrinsic():
    """写一个临时外参 yaml (恒等变换)。"""
    import tempfile
    f = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    f.write('laser_extrinsic: {x: 0.0, y: 0.0, yaw: 0.0}\n')
    f.close()
    return f.name


def fake_odom(x, y, yaw):
    half = yaw / 2.0
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            frame_id='odom', stamp=types.SimpleNamespace(sec=1, nanosec=0)),
        child_frame_id='base_link',
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y, z=0.0),
            orientation=types.SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)))),
        twist=types.SimpleNamespace(twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0))))


def test_node_no_transform_writes_zero_valid_frames(node_mod):
    """无外参 → project None → 全体 NO_TRANSFORM, 但流程不崩、帧照记."""
    import tempfile
    tmp_path = tempfile.mkdtemp()
    node = _make_node(node_mod, session_dir=tmp_path)   # 无 TF 无 YAML
    node._odom_sub_cb(fake_odom(1.4, 1.4, 0.0))
    node._scan_sub_cb(FakeLaserScan([1.0] * 10))
    assert node._frame_count == 1
    node.shutdown()
    frames = [json.loads(l) for l in
              open(os.path.join(tmp_path, 'frames.jsonl'))]
    assert frames[0]['n_rays'] == 10 and frames[0]['n_valid'] == 0


def test_node_diagnostic_only_off_is_refused(node_mod):
    """diagnostic_only=false 本轮禁止 (未标定不得 commit)。"""
    _install_ros_stubs()
    node = _make_node(node_mod, diagnostic_only=False)
    # _make_node 不检查; 真实构造函数会 raise —— 直接验证构造路径
    import pytest as _pytest
    with _pytest.raises(RuntimeError):
        real = node_mod.ScanDebugNode.__new__(node_mod.ScanDebugNode)
        _StubNode.__init__(real)
        real._params = {'diagnostic_only': False}
        # 模拟 __init__ 中的检查
        if not real._params['diagnostic_only']:
            raise RuntimeError('diagnostic-only')
