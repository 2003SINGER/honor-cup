#!/usr/bin/env python3
"""scan_debug —— 实车静态 LaserScan 感知调试节点 (只读).

数据流 (与离线重放同一条链):
    /scan ─→ ScanAdapter ─→ FrameProjector(laser→base→odom→maze)
          ─→ GridAssociation ─→ DiagnosticObservation[]
          ├─→ MarkerArray (RViz, 只观察不进算法)
          └─→ FrameAccumulator ─→ frames.jsonl ─→ summary 工具

铁律:
  - 只读: 绝不创建 /cmd_vel publisher, 绝不 arm 任何运动层;
  - topic 名全部可配置 (车端实际 scan topic 未确认, 不做假设);
  - laser→base 优先 TF, 否则显式 YAML 外参, 都没有 → 全体 NO_TRANSFORM
    ABSTAIN, 绝不假设雷达在车中心;
  - maze anchor 启动时建一次, 之后跟随 odom, 永不把车吸回格中心;
  - diagnostic_only 默认 true: 诊断层绝不写 EdgeMap;
  - RViz 只是观察同一批数据, 不是算法数据通路。"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist  # noqa: F401  (仅类型存在性检查用)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from m3pro_nav.frame_projector import (FIELD_SIZE, FrameProjector,
                                       LaserExtrinsic, ManualMazeAnchor)
from m3pro_nav.grid_association import GridAssociation
from m3pro_nav.odometry_adapter import odometry_from_msg
from m3pro_nav.pose import Pose2D
from m3pro_nav.scan_adapter import parse_scan
from m3pro_nav.scan_debug_markers import build_markers
from m3pro_nav.scan_diagnostics import FrameAccumulator


class ScanDebugNode(Node):
    def __init__(self):
        super().__init__('scan_debug')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('imu_topic', '')            # 只检查存在性
        self.declare_parameter('expected_laser_frame', '')
        self.declare_parameter('expected_odom_frame', 'odom')
        self.declare_parameter('expected_base_frame', 'base_link')
        self.declare_parameter('use_tf_extrinsic', True)
        self.declare_parameter('laser_extrinsic_yaml', '')
        self.declare_parameter('cell_x', 0)
        self.declare_parameter('cell_y', 0)
        self.declare_parameter('heading', 'N')
        self.declare_parameter('anchor_offset_x', 0.0)
        self.declare_parameter('anchor_offset_y', 0.0)
        self.declare_parameter('anchor_offset_yaw', 0.0)
        self.declare_parameter('session_dir', '')
        self.declare_parameter('markers_topic', '/scan_debug/markers')
        self.declare_parameter('diagnostic_only', True)    # 本轮恒 true
        p = lambda name: self.get_parameter(name).value    # noqa: E731

        self._scan_topic = str(p('scan_topic'))
        self._odom_topic = str(p('odom_topic'))
        self._laser_frame_hint = str(p('expected_laser_frame'))
        self._odom_frame = str(p('expected_odom_frame'))
        self._base_frame = str(p('expected_base_frame'))
        self._diagnostic_only = bool(p('diagnostic_only'))
        if not self._diagnostic_only:
            raise RuntimeError('scan_debug is diagnostic-only in this '
                               'release; EdgeMap writes are not calibrated')

        # ---- 感知管线 (纯 Python, 与离线重放完全同一套) ----
        self._association = GridAssociation()
        self._projector = FrameProjector(self._load_extrinsic(p))
        self._anchor = None                 # 首帧 odom 到达时建立
        self._latest_odom_pose = None
        self._frame_count = 0
        self._frames_file = None
        session_dir = str(p('session_dir'))
        if session_dir:
            os.makedirs(session_dir, exist_ok=True)
            self._frames_path = os.path.join(session_dir, 'frames.jsonl')
            self._frames_file = open(self._frames_path, 'a', buffering=1)
        else:
            self._frames_path = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._markers_pub = self.create_publisher(
            MarkerArray, str(p('markers_topic')), qos)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, qos)
        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, qos)
        self.get_logger().info(
            f"scan_debug: scan={self._scan_topic} odom={self._odom_topic} "
            f"extrinsic={self._projector.extrinsic.source} "
            f"cell=({int(p('cell_x'))},{int(p('cell_y'))}) "
            f"heading={p('heading')}")

    # ---- 外参: TF 优先, 显式 YAML 次之, 缺失 → ABSTAIN ----

    def _load_extrinsic(self, p):
        if bool(p('use_tf_extrinsic')):
            ext = self._lookup_tf_extrinsic()
            if ext is not None:
                return ext
        yaml_path = str(p('laser_extrinsic_yaml'))
        if yaml_path:
            return self._load_yaml_extrinsic(yaml_path)
        self.get_logger().warn(
            'no laser extrinsic (no TF, no YAML): all observations '
            'will be NO_TRANSFORM / ABSTAIN')
        return LaserExtrinsic.missing()

    def _lookup_tf_extrinsic(self):
        try:
            from tf2_ros import Buffer, TransformListener
        except ImportError:
            return None
        try:
            buffer = Buffer()
            # spin_thread=True: constructor 阶段 executor 尚未 spin, listener
            # 必须自带线程收 /tf(_static), 否则 TF 存在也会假 missing。
            TransformListener(buffer, self, spin_thread=True)
            from rclpy.duration import Duration
            from time import sleep
            for _ in range(50):                     # 等 TF 就绪 (≤5s)
                if buffer.can_transform(self._base_frame,
                                        self._laser_frame_hint
                                        or 'laser', rclpy.time.Time()):
                    break
                sleep(0.1)
            tf = buffer.lookup_transform(
                self._base_frame, self._laser_frame_hint or 'laser',
                rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return LaserExtrinsic.from_tf(t.x, t.y, yaw)
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().warn(f'TF extrinsic unavailable: {exc}')
            return None

    def _load_yaml_extrinsic(self, path):
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f)
        ext = data.get('laser_extrinsic', data)
        self.get_logger().info(f'laser extrinsic from YAML ({path}): '
                               'UNCALIBRATED until field-measured')
        return LaserExtrinsic.from_yaml(ext['x'], ext['y'], ext['yaw'])

    # ---- ROS 回调: 只做消息适配, 无算法 ----

    def _on_odom(self, msg):
        try:
            state = odometry_from_msg(
                msg, expected_odom_frame=self._odom_frame,
                expected_base_frame=self._base_frame)
        except ValueError as exc:
            self.get_logger().warn(f'odom rejected: {exc}')
            return
        if self._anchor is None:
            self._anchor = ManualMazeAnchor(
                (self.get_parameter('cell_x').value,
                 self.get_parameter('cell_y').value),
                self.get_parameter('heading').value,
                state.pose,
                offset=(self.get_parameter('anchor_offset_x').value,
                        self.get_parameter('anchor_offset_y').value,
                        self.get_parameter('anchor_offset_yaw').value))
            self._broadcast_maze_tf(state.pose)
            self.get_logger().info(
                f'maze anchor fixed: odom=({state.pose.x:.3f},'
                f'{state.pose.y:.3f},{state.pose.yaw:.3f}) -> '
                f'maze=({self._anchor.maze_anchor.x:.3f},'
                f'{self._anchor.maze_anchor.y:.3f})')
        self._latest_odom_pose = state.pose

    def _broadcast_maze_tf(self, odom_pose):
        """发布一次性 static TF maze←odom, 仅为 RViz 显示 (非算法通路)."""
        try:
            from tf2_ros import StaticTransformBroadcaster
            from geometry_msgs.msg import TransformStamped
        except ImportError:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'maze'
        t.child_frame_id = self._odom_frame
        anchor = self._anchor.maze_anchor
        yaw = anchor.yaw - odom_pose.yaw
        t.transform.translation.x = float(
            anchor.x - (math.cos(yaw) * odom_pose.x
                        - math.sin(yaw) * odom_pose.y))
        t.transform.translation.y = float(
            anchor.y - (math.sin(yaw) * odom_pose.x
                        + math.cos(yaw) * odom_pose.y))
        t.transform.rotation.z = float(math.sin(yaw / 2.0))
        t.transform.rotation.w = float(math.cos(yaw / 2.0))
        StaticTransformBroadcaster(self).sendTransform(t)

    def _on_scan(self, msg):
        frame = parse_scan(msg)
        if self._laser_frame_hint and frame.frame_id != self._laser_frame_hint:
            self.get_logger().warn_once(
                f"scan frame_id {frame.frame_id!r} != expected "
                f"{self._laser_frame_hint!r}")
        odom_pose = self._latest_odom_pose
        if self._anchor is None or odom_pose is None:
            self.get_logger().warn_once('no odom yet: frame dropped',
                                        throttle_duration_sec=2.0)
            return
        # ---- 与离线重放完全同一管线 ----
        world_rays = self._projector.project(frame, odom_pose, self._anchor)
        if world_rays is None:
            # 无外参: 每条 ray 记 NO_TRANSFORM 诊断 (ABSTAIN, 绝不猜)
            from m3pro_nav.grid_association import (NONE,
                                                    DiagnosticObservation)
            observations = [DiagnosticObservation(
                r.index, NONE, 'NO_TRANSFORM', None, (), ())
                for r in frame.rays]
        else:
            observations = self._association.process(world_rays)
        acc = FrameAccumulator()
        acc.accumulate(observations, frame.stamp)
        self._frame_count += 1
        if self._frames_file is not None:
            self._frames_file.write(json.dumps(acc.to_json()) + '\n')
        self._publish_markers(world_rays, observations, frame.stamp)

    # ---- RViz 批量 markers (只观察) ----

    def _publish_markers(self, world_rays, observations, stamp):
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()
        for desc in build_markers(world_rays, observations):
            m = Marker()
            m.header.frame_id = 'maze'
            m.header.stamp = now
            m.id = desc['id']
            m.type = {'LINE_LIST': Marker.LINE_LIST,
                      'POINTS': Marker.POINTS}[desc['type']]
            m.color.r, m.color.g, m.color.b, m.color.a = desc['color']
            m.scale.x = m.scale.y = m.scale.z = desc['scale']
            for (x, y) in desc['points']:
                pt = m.points.add() if hasattr(m.points, 'add') else None
                if pt is None:
                    from geometry_msgs.msg import Point
                    m.points.append(Point(x=float(x), y=float(y), z=0.0))
                else:
                    pt.x, pt.y, pt.z = float(x), float(y), 0.0
            msg.markers.append(m)
        self._markers_pub.publish(msg)

    def shutdown(self):
        if self._frames_file is not None:
            self._frames_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = ScanDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
