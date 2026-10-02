"""Focused ROS-glue safety regression tests without a ROS installation."""

import importlib
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))


@pytest.fixture
def nav_node_module(monkeypatch):
    rclpy = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = type('Node', (), {})
    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    geom_msg = types.ModuleType('geometry_msgs.msg')

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    geom_msg.Twist = Twist
    nav_msg = types.ModuleType('nav_msgs.msg')
    nav_msg.Odometry = type('Odometry', (), {})
    sensor_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msg.LaserScan = type('LaserScan', (), {})
    sensor_msg.Imu = type('Imu', (), {})
    modules = {
        'rclpy': rclpy, 'rclpy.node': node_mod, 'rclpy.qos': qos_mod,
        'geometry_msgs': types.ModuleType('geometry_msgs'),
        'geometry_msgs.msg': geom_msg,
        'nav_msgs': types.ModuleType('nav_msgs'), 'nav_msgs.msg': nav_msg,
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': sensor_msg,
    }
    modules['rclpy'].node = node_mod
    modules['rclpy'].qos = qos_mod
    for parent_name, child_name in (
            ('geometry_msgs', 'geometry_msgs.msg'),
            ('nav_msgs', 'nav_msgs.msg'), ('sensor_msgs', 'sensor_msgs.msg')):
        modules[parent_name].msg = modules[child_name]
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    module_name = 'm3pro_nav.nav_runtime_node'
    sys.modules.pop(module_name, None)
    module = importlib.import_module(module_name)
    yield module
    sys.modules.pop(module_name, None)


def test_control_tick_exception_latches_fault_and_keeps_publishing_zero(
        nav_node_module):
    from m3pro_nav.motion_runtime import SafetyState

    class Core:
        fault_reason = None

        def report_fault(self, reason):
            self.fault_reason = reason

    class Runtime:
        phase = 'EXPLORING'

        def __init__(self):
            self.core = Core()
            self.calls = 0

        def control_tick(self, _now):
            self.calls += 1
            if self.calls == 1:
                raise ValueError('control failure')
            assert self.core.fault_reason is not None
            return types.SimpleNamespace(
                follower=None, state=types.SimpleNamespace(value='FAULT'),
                safety=SafetyState.FAULT,
                command=types.SimpleNamespace(vx=0.0, vy=0.0, wz=0.0))

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    class Csv:
        def writerow(self, _row):
            pass

    node = types.SimpleNamespace(
        get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            nanoseconds=1_000_000_000)),
        _ownership_tick=0, _dry_run=True,
        get_logger=lambda: types.SimpleNamespace(error=lambda _msg: None),
        runtime=Runtime(), _cmd_pub=Publisher(), _csv=Csv())

    nav_node_module.NavRuntimeNode._control_tick(node)
    assert node.runtime.core.fault_reason.startswith('control tick exception:')
    assert len(node._cmd_pub.messages) == 1
    first_zero = node._cmd_pub.messages[0]
    assert (first_zero.linear.x, first_zero.linear.y,
            first_zero.angular.z) == (0.0, 0.0, 0.0)

    # A later healthy calculation still returns FAULT/zero until explicit reset.
    nav_node_module.NavRuntimeNode._control_tick(node)
    assert len(node._cmd_pub.messages) == 2
    next_zero = node._cmd_pub.messages[1]
    assert (next_zero.linear.x, next_zero.linear.y,
            next_zero.angular.z) == (0.0, 0.0, 0.0)
