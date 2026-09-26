#!/usr/bin/env python3
"""motion_runtime_node ROS 胶水测试 —— rclpy stub 注入, 无需 ROS 环境.

覆盖 (GPT 审查 P0-1 / P1-6):
  - _on_odom 真正调用 odometry_from_msg 的完整签名 (帧校验参数) —— 修复
    第一帧 /odom_raw 即 TypeError 的 P0;
  - 帧名不符的 odom 消息被丢弃并告警, 不崩溃;
  - ARMED 期间 /cmd_vel 出现其他发布者 → FAULT + 零指令 (不只是 arm 前查一次);
  - 节点源不含控制数学 (与 test_motion_runtime Gate A 呼应)。"""

import math
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))


# ---- rclpy / 消息 stub (仅结构, 无 ROS 依赖) ----

class _Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, frame_id='odom'):
        self.stamp = _Stamp()
        self.frame_id = frame_id


class _Point:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Pose:
    def __init__(self):
        self.position = _Point()
        self.orientation = _Quaternion()


class _PoseWithCovariance:
    def __init__(self):
        self.pose = _Pose()


class _Twist:
    def __init__(self, lx=0.0, ly=0.0, az=0.0):
        self.linear = types.SimpleNamespace(x=lx, y=ly)
        self.angular = types.SimpleNamespace(z=az)


class _TwistWithCovariance:
    def __init__(self, twist=None):
        self.twist = twist if twist is not None else _Twist()


class FakeOdometry:
    def __init__(self, *, frame_id='odom', child='base_link',
                 x=0.0, y=0.0, yaw=0.0, lx=0.0, ly=0.0, az=0.0):
        self.header = _Header(frame_id)
        self.child_frame_id = child
        self.pose = _PoseWithCovariance()
        self.pose.pose.position = _Point(x, y)
        half = yaw / 2.0
        self.pose.pose.orientation = _Quaternion(0.0, 0.0,
                                                 math.sin(half), math.cos(half))
        self.twist = _TwistWithCovariance(_Twist(lx, ly, az))


class _StubNode:
    """rclpy.node.Node 替身: 只提供本测试用到的能力."""
    def __init__(self, *a, **k):
        self._publishers = {'/cmd_vel': 1}
        self._clock_now = [0.0]
    def declare_parameter(self, name, default):
        self.__dict__.setdefault('_params', {})[name] = default
    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])
    def create_publisher(self, *a, **k):
        return types.SimpleNamespace(publish=lambda msg: None)
    def create_subscription(self, *a, **k):
        return None
    def create_timer(self, period, cb):
        return types.SimpleNamespace(period=period)
    def count_publishers(self, topic):
        return self._publishers.get(topic, 0)
    def get_clock(self):
        node = self
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(
                nanoseconds=int(node._clock_now[0] * 1e9)))
    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda m: None, warn=lambda m: None, error=lambda m: None)


def _install_ros_stubs():
    rclpy = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = _StubNode
    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile = lambda **k: types.SimpleNamespace(**k)
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    geom_msg = types.ModuleType('geometry_msgs.msg')

    class _TwistMsg:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    geom_msg.Twist = _TwistMsg
    nav_msg = types.ModuleType('nav_msgs.msg')
    nav_msg.Odometry = FakeOdometry
    rclpy.node = node_mod
    rclpy.qos = qos_mod
    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs.msg = geom_msg
    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs.msg = nav_msg
    for name, mod in [('rclpy', rclpy), ('rclpy.node', node_mod),
                      ('rclpy.qos', qos_mod), ('geometry_msgs', geometry_msgs),
                      ('geometry_msgs.msg', geom_msg),
                      ('nav_msgs', nav_msgs), ('nav_msgs.msg', nav_msg)]:
        sys.modules[name] = mod
    return rclpy


@pytest.fixture()
def node_cls():
    _install_ros_stubs()
    import importlib
    import m3pro_nav.motion_runtime_node as mod
    importlib.reload(mod)
    return mod.MotionRuntimeNode


def _bare_node(node_cls, *, arm=False, publishers=1):
    """绕过 rclpy 初始化构造节点 (stub Node.__init__ 无副作用)."""
    node = node_cls()
    node._params = {'expected_odom_frame': 'odom',
                    'expected_base_frame': 'base_link',
                    'max_feedback_age_s': 0.5, 'arm': arm}
    # 重新执行参数读取路径: 直接构造会二次 create_publisher, 手动重置状态
    node._expected_odom_frame = 'odom'
    node._expected_base_frame = 'base_link'
    node._latest_odom = None
    node._armed_requested = arm
    node._ownership_checked = False
    node._tick_count = 0
    node._shutting_down = False
    node._publishers['/cmd_vel'] = publishers
    return node


# ---- P0-1: odom callback 真正走完 odometry_from_msg 签名 ----

def test_odom_callback_uses_full_adapter_signature(node_cls):
    node = _bare_node(node_cls)
    msg = FakeOdometry(x=1.0, y=2.0, yaw=math.pi / 2, lx=0.1, ly=0.2, az=0.01)
    node._on_odom(msg)
    assert node._latest_odom is not None
    stamp, state = node._latest_odom
    assert stamp == 0.0
    assert state.pose.x == 1.0 and state.pose.y == 2.0
    assert state.frame_id == 'odom' and state.child_frame_id == 'base_link'
    # child-frame twist → world (yaw=90°): world_vx = -ly, world_vy = lx
    assert state.vx_world == pytest.approx(-0.2)
    assert state.vy_world == pytest.approx(0.1)


def test_odom_callback_rejects_wrong_frames_without_crashing(node_cls):
    node = _bare_node(node_cls)
    node._on_odom(FakeOdometry(frame_id='map'))          # 帧名不符
    assert node._latest_odom is None                     # 丢弃, 不崩溃
    node._on_odom(FakeOdometry(child='base_footprint'))
    assert node._latest_odom is None


# ---- P1-6: ARMED 期间持续监测 /cmd_vel 归属 ----

def test_publisher_conflict_while_armed_faults(node_cls):
    from m3pro_nav.motion_runtime import SafetyState
    node = _bare_node(node_cls, arm=True, publishers=1)
    node._tick()                                          # call #1: arm 成功
    assert node.core.safety == SafetyState.ARMED
    node._publishers['/cmd_vel'] = 2                      # 手柄节点中途启动
    for _ in range(23):                                   # calls #2..24: 未到复查周期
        node._tick()
    assert node.core.safety == SafetyState.ARMED
    node._tick()                                          # call #25: 复查 → FAULT
    assert node.core.safety == SafetyState.FAULT
    out = node.core.update(1.0, None)
    assert out.command.vx == out.command.vy == out.command.wz == 0.0


def test_publisher_conflict_refuses_arm_before_start(node_cls):
    from m3pro_nav.motion_runtime import SafetyState
    node = _bare_node(node_cls, arm=True, publishers=2)
    node._tick()
    node._tick()
    assert node.core.safety == SafetyState.DISARMED       # 始终拒绝 arm


# ---- 节点源仍是薄胶水 (无控制数学) ----

def test_node_source_contains_no_control_math():
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'ros2', 'm3pro_nav', 'm3pro_nav')
    src = open(os.path.join(base, 'motion_runtime_node.py')).read()
    for banned in ('SpeedProfile', 'TrajectoryReference', 'kp_pos',
                   'kd_vel', 'resolve_next', 'EdgeMap'):
        assert banned not in src, f'ROS 节点含控制数学: {banned}'
    # P0-1 回归: odometry_from_msg 调用必须带完整帧校验参数
    assert 'odometry_from_msg(\n' in src or 'odometry_from_msg(' in src
    assert 'expected_odom_frame=self._expected_odom_frame' in src
    assert 'expected_base_frame=self._expected_base_frame' in src
