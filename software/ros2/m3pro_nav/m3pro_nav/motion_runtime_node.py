#!/usr/bin/env python3
"""motion_runtime —— 薄 ROS 执行层 (GPT 定稿: 只做消息适配/安全/生命周期).

数据流:
    /odom_raw ──→ OdometryAdapter ──→ FeedbackSample ──┐
                                                       ↓
                        MotionRuntimeCore (纯 Python, 无 ROS)
                                                       ↓
                                              Twist2D ──→ /cmd_vel

铁律:
  - 本节点内禁止出现 PID/轨迹插值/DFS/地图逻辑 —— 控制数学全部在
    FeedbackTrajectoryFollower / PositionController (纯 Python 核心);
  - 不创建新的 Primitive ROS message/topic —— 计划经同进程 Python API
    (core.load_plan / core.append_suffix) 注入;
  - 默认 DISARMED, 不 arm 不发任何运动指令; /cmd_vel 已有其他发布者时拒绝 arm;
  - FAULT / 关闭 / 异常: 持续发零指令;
  - 雷达/摄像头/比赛逻辑本轮不接, 硬件参数全部 UNCALIBRATED。"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from m3pro_nav.motion_runtime import FeedbackSample, MotionRuntimeCore
from m3pro_nav.odometry_adapter import odometry_from_msg
from m3pro_nav.pose import Pose2D
from m3pro_nav.position_controller import PositionController

CONTROL_PERIOD_S = 0.02
DEFAULT_MAX_FEEDBACK_AGE_S = 0.5
OWNERSHIP_CHECK_INTERVAL_TICKS = 25     # ARMED 后每 ~0.5s 复查 /cmd_vel 归属
OWNERSHIP_CHECK_INTERVAL_TICKS = 25     # ARMED 后每 ~0.5s 复查 /cmd_vel 归属


class MotionRuntimeNode(Node):
    """/odom_raw + 控制钟 → MotionRuntimeCore → /cmd_vel (受安全门控)."""

    def __init__(self):
        super().__init__('motion_runtime')
        self.declare_parameter('expected_odom_frame', 'odom')
        self.declare_parameter('expected_base_frame', 'base_link')
        self.declare_parameter('max_feedback_age_s', DEFAULT_MAX_FEEDBACK_AGE_S)
        self.declare_parameter('arm', False)          # 默认绝不 ARMED
        odom_frame = self.get_parameter('expected_odom_frame').value
        base_frame = self.get_parameter('expected_base_frame').value
        max_age = float(self.get_parameter('max_feedback_age_s').value)
        self._expected_odom_frame = odom_frame
        self._expected_base_frame = base_frame

        self.core = MotionRuntimeCore(
            a_acc=1.0, a_dec=1.0, controller=PositionController(),
            max_feedback_age_s=max_age,
            expected_odom_frame=odom_frame, expected_base_frame=base_frame)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self._odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self._on_odom, qos)
        self._timer = self.create_timer(CONTROL_PERIOD_S, self._tick)
        self._latest_odom = None          # (stamp_s, OdometryState)
        self._armed_requested = bool(self.get_parameter('arm').value)
        self._ownership_checked = False
        self._tick_count = 0
        self._shutting_down = False

    # ---- ROS 回调: 只做消息适配, 无控制数学 ----

    def _on_odom(self, msg):
        try:
            state = odometry_from_msg(
                msg,
                expected_odom_frame=self._expected_odom_frame,
                expected_base_frame=self._expected_base_frame)
        except ValueError as exc:
            # 帧名不符/字段非法: 丢弃本帧并告警, 不崩溃 (新鲜度由看门狗管)
            self.get_logger().warn(f'odometry rejected: {exc}')
            return
        stamp = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        self._latest_odom = (stamp, state)

    # ---- 安全门控 ----

    def _try_arm(self):
        """Gate M: /cmd_vel 已有其他发布者 → 拒绝 arm (不抢控制权)."""
        if self.core.safety.value == 'ARMED' or self._shutting_down:
            return
        if not self._armed_requested:
            return
        if not self._ownership_checked:
            publishers = self.count_publishers('/cmd_vel')
            if publishers > 1:            # 自己 + 他人
                self.get_logger().warn(
                    f'/cmd_vel has {publishers} publishers; refuse to arm')
                return
            self._ownership_checked = True
        self.core.arm()
        self.get_logger().info('motion runtime ARMED')

    def _check_ownership_while_armed(self):
        """ARMED 期间周期复查: 出现其他 /cmd_vel 发布者 → FAULT (零指令).
        (手柄/调试节点中途启动也必须被发现, 不只在 arm 前查一次)"""
        if self.core.safety.value != 'ARMED':
            return
        publishers = self.count_publishers('/cmd_vel')
        if publishers > 1:
            self.get_logger().error(
                f'/cmd_vel now has {publishers} publishers; FAULT')
            self.core.report_fault('/cmd_vel publisher conflict while ARMED')

    # ---- 控制周期 ----

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._try_arm()
        self._tick_count += 1
        if self._tick_count % OWNERSHIP_CHECK_INTERVAL_TICKS == 0:
            self._check_ownership_while_armed()
        feedback = None
        if self._latest_odom is not None:
            stamp, state = self._latest_odom
            feedback = FeedbackSample(stamp, state.pose, state.vx_world,
                                      state.vy_world, state.wz)
        try:
            out = self.core.update(now, feedback)
        except Exception as exc:                      # 任何异常 → 零指令
            self.get_logger().error(f'runtime error: {exc}')
            out = None
        self._publish(out)

    def _publish(self, out):
        msg = Twist()
        if out is not None and self.core.safety.value == 'ARMED':
            msg.linear.x = float(out.command.vx)
            msg.linear.y = float(out.command.vy)
            msg.angular.z = float(out.command.wz)
        # DISARMED / FAULT / 异常: 一律零指令
        self._cmd_pub.publish(msg)

    def shutdown_zero(self):
        """关闭序列: 短暂连发零指令, 之后再停发."""
        self._shutting_down = True
        zero = Twist()
        for _ in range(25):                           # ~0.5s @20ms
            self._cmd_pub.publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = MotionRuntimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
