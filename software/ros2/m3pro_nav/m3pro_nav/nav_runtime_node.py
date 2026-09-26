#!/usr/bin/env python3
"""nav_runtime —— 正式 ROS 运行节点 (事件驱动, GPT 定稿架构).

    /scan_multi ─→ on_scan callback ──→ RealObservationAdapter ──→ EdgeMap
    /odom_raw   ─→ on_odom callback ─→ 事件桥 + 反馈缓存
    (control timer 50Hz)  ─→ MotionRuntimeCore ─→ /cmd_vel (非 dry-run)
    (planner timer 10Hz)  ─→ ActionHorizon 编链/延长/返航

铁律:
  - 绝不 while 串行: 每类工作一个事件源 (subscription/timer), 长活不进
    callback (规划在独立 timer, 编链失败只记日志不阻塞);
  - dry_run (默认 true): 完整跑传感器→地图→规划→轨迹→控制计算,
    但绝不创建 /cmd_vel publisher;
  - preflight: odom/scan 类型与帧名、外参可用性、配置完整性、
    /cmd_vel 归属 —— 任一不符即拒绝进入 EXPLORING (fail closed);
  - UNCALIBRATED 配置 (__CALIBRATE__/__MEASURE__) 只允许 dry_run;
  - evidence 目录: 每次运行一个 run_<ts>/ (events.jsonl / runtime.csv /
    config 快照 / git SHA) —— 现场日志喂 LLM 分析, 不做一步登天的自动标定;
  - 核心算法 (StreamNav/ActionHorizon/MotionPlanner/MotionRuntimeCore)
    一行不改。"""

import csv
import json
import os
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from m3pro_nav.frame_projector import LaserExtrinsic
from m3pro_nav.nav_runtime import (NavRuntime, NavRuntimePhase,
                                      load_runtime_config)
from m3pro_nav.odometry_adapter import odometry_from_msg
from m3pro_nav.motion_runtime import SafetyState

CONTROL_PERIOD_S = 0.02          # 50 Hz
PLANNER_PERIOD_S = 0.10          # 10 Hz

class NavRuntimeNode(Node):
    def __init__(self):
        super().__init__('nav_runtime')
        self.declare_parameter('config_path',
                               'config/nav_runtime.yaml')
        self.declare_parameter('dry_run', True)          # 默认绝不发车
        self.declare_parameter('evidence_root', 'field_data')
        self.declare_parameter('run_label', 'nav')
        p = lambda n: self.get_parameter(n).value          # noqa: E731
        self._dry_run = bool(p('dry_run'))

        config, uncalibrated = load_runtime_config(str(p('config_path')))
        self._config = config
        self._uncalibrated = uncalibrated
        if uncalibrated and not self._dry_run:
            raise RuntimeError(
                f'refusing real run: uncalibrated config keys {uncalibrated}; '
                'run with dry_run:=true or fill config/nav_runtime.yaml')

        ros_cfg = config.get('ros', {})
        frames_cfg = config.get('frames', {})
        perc_cfg = config.get('perception', {})
        anchor_cfg = config.get('anchor', {})
        ctrl_cfg = config.get('control', {})
        scan_topic = ros_cfg.get('scan_topic', '/scan_multi')
        odom_topic = ros_cfg.get('odom_topic', '/odom_raw')
        self._odom_frame = frames_cfg.get('odom_frame', 'odom')
        self._base_frame = frames_cfg.get('base_frame', 'base_link')
        self._scan_frame = frames_cfg.get('laser_frame', '')

        extrinsic = self._load_extrinsic(config)
        self.runtime = NavRuntime(
            entry=(int(anchor_cfg.get('entry_x', 0)),
                   int(anchor_cfg.get('entry_y', 0))),
            order=str(anchor_cfg.get('order', 'LFR')),
            cell=(int(anchor_cfg.get('cell_x', 0)),
                  int(anchor_cfg.get('cell_y', 0))),
            heading=str(anchor_cfg.get('heading', 'N')),
            dry_run=self._dry_run,
            perception=perc_cfg,
            extrinsic=extrinsic,
            a_acc=float(ctrl_cfg.get('a_acc', 1.0)),
            a_dec=float(ctrl_cfg.get('a_dec', 1.0)),
            max_feedback_age_s=float(
                config.get('safety', {}).get('max_feedback_age_s', 0.5)),
        )
        self.runtime.set_anchor_spec(
            (int(anchor_cfg.get('cell_x', 0)),
             int(anchor_cfg.get('cell_y', 0))),
            str(anchor_cfg.get('heading', 'N')))

        # ---- evidence 目录 (每次运行一份, 供 LLM 分析) ----
        import time
        run_dir = os.path.join(str(p('evidence_root')),
                               f'run_{time.strftime("%Y%m%d_%H%M%S")}_'
                               f'{p("run_label")}'
                               + ('_dryrun' if self._dry_run else ''))
        os.makedirs(run_dir, exist_ok=True)
        self._run_dir = run_dir
        with open(os.path.join(run_dir, 'config.yaml'), 'w') as f:
            import yaml as _y
            _y.safe_dump(config, f)
        try:
            sha = subprocess.check_output(
                ['git', 'rev-parse', 'HEAD'],
                cwd=os.path.dirname(os.path.abspath(__file__)),
                stderr=subprocess.DEVNULL, text=True).strip()
        except Exception:                                 # noqa: BLE001
            sha = 'unknown'
        with open(os.path.join(run_dir, 'git.txt'), 'w') as f:
            f.write(sha + '\n')
        self._events_path = os.path.join(run_dir, 'events.jsonl')
        self._events_file = open(self._events_path, 'a', buffering=1)
        self._runtime_csv = open(os.path.join(run_dir, 'runtime.csv'),
                                 'a', buffering=1, newline='')
        self._csv = csv.writer(self._runtime_csv)
        self._csv.writerow(['t', 'phase', 'state', 'safety',
                            'cmd_vx', 'cmd_vy', 'cmd_wz',
                            'pos_err', 'yaw_err'])
        self._last_flushed_event = 0
        self._scans_seen = 0
        self._odoms_seen = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos)
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos)
        self._planner_timer = self.create_timer(
            PLANNER_PERIOD_S, self._planner_tick)
        self._control_timer = self.create_timer(
            CONTROL_PERIOD_S, self._control_tick)
        self._cmd_pub = None
        if not self._dry_run:
            # preflight: /cmd_vel 已有其他发布者 → 拒绝 (不抢控制权)
            if self.count_publishers('/cmd_vel') > 0:
                raise RuntimeError(
                    'preflight FAIL: /cmd_vel already has a publisher; '
                    'refusing to create another (dry_run only)')
            self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.get_logger().info(
            f"nav_runtime: dry_run={self._dry_run} scan={scan_topic} "
            f"odom={odom_topic} uncalibrated={len(uncalibrated)} "
            f"evidence={run_dir}")

    def _load_extrinsic(self, config):
        ext_cfg = config.get('laser_extrinsic', {})
        source = str(ext_cfg.get('source', 'yaml'))
        if source == 'tf':
            return self._lookup_tf_extrinsic(config)
        x, y, yaw = (ext_cfg.get('x', 0.0), ext_cfg.get('y', 0.0),
                     ext_cfg.get('yaw', 0.0))
        if any(isinstance(v, str) and any(p in v for p in _PLACEHOLDERS)
               for v in (x, y, yaw)):
            self.get_logger().warn(
                'laser extrinsic UNCALIBRATED: all observations ABSTAIN')
            return LaserExtrinsic.missing()
        return LaserExtrinsic.from_yaml(x, y, yaw)

    def _lookup_tf_extrinsic(self, config):
        frames = config.get('frames', {})
        laser = frames.get('laser_frame', 'laser')
        base = frames.get('base_frame', 'base_link')
        try:
            from tf2_ros import Buffer, TransformListener
            from time import sleep
            import math as _m
            buffer = Buffer()
            TransformListener(buffer, self)
            for _ in range(50):
                if buffer.can_transform(base, laser, rclpy.time.Time()):
                    break
                sleep(0.1)
            tf = buffer.lookup_transform(base, laser, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = _m.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return LaserExtrinsic.from_tf(t.x, t.y, yaw)
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().warn(f'TF extrinsic unavailable: {exc}')
            return LaserExtrinsic.missing()

    # ---- 事件源: 只做消息适配, 长活在 timer ----

    def _on_odom(self, msg):
        self._odoms_seen += 1
        try:
            state = odometry_from_msg(
                msg, expected_odom_frame=self._odom_frame,
                expected_base_frame=self._base_frame)
        except ValueError as exc:
            self.get_logger().warn(f'odom rejected: {exc}', 
                                   throttle_duration_sec=5.0)
            return
        self.runtime.on_odom(state, receive_stamp=
                             self.get_clock().now().nanoseconds * 1e-9)
        self._flush_events()

    def _on_scan(self, msg):
        self._scans_seen += 1
        if self._scan_frame and msg.header.frame_id != self._scan_frame:
            self.get_logger().warn(
                f"scan frame {msg.header.frame_id!r} != expected "
                f"{self._scan_frame!r}", throttle_duration_sec=10.0)
        stats = self.runtime.on_scan(msg)
        if stats is not None:
            self._events_file.write(json.dumps({
                't': stats.stamp, 'kind': 'scan',
                **stats.to_json()}) + '\n')

    def _planner_tick(self):
        try:
            self.runtime.planner_tick(
                self.get_clock().now().nanoseconds * 1e-9)
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().error(f'planner tick error: {exc}')
            self._events_file.write(json.dumps(
                {'t': self.get_clock().now().nanoseconds * 1e-9,
                 'kind': 'planner_error', 'error': str(exc)}) + '\n')
        self._flush_events()

    def _control_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            out = self.runtime.control_tick(now)
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().error(f'control tick error: {exc}')
            return
        fs = out.follower
        self._csv.writerow([
            f'{now:.3f}', self.runtime.phase, out.state.value,
            out.safety.value,
            f'{out.command.vx:.4f}', f'{out.command.vy:.4f}',
            f'{out.command.wz:.4f}',
            f'{fs.position_error:.4f}' if fs else '',
            f'{fs.yaw_error:.4f}' if fs else ''])
        if self._cmd_pub is not None and out.safety == SafetyState.ARMED:
            msg = Twist()
            msg.linear.x = float(out.command.vx)
            msg.linear.y = float(out.command.vy)
            msg.angular.z = float(out.command.wz)
            self._cmd_pub.publish(msg)
        elif self._cmd_pub is not None:
            self._cmd_pub.publish(Twist())     # DISARMED/FAULT: 零指令

    def _flush_events(self):
        evs = self.runtime.events
        while self._last_flushed_event < len(evs):
            self._events_file.write(json.dumps(
                evs[self._last_flushed_event]) + '\n')
            self._last_flushed_event += 1

    def shutdown(self):
        if self._cmd_pub is not None:          # 关闭: 连发零指令 ~0.5s
            for _ in range(25):
                self._cmd_pub.publish(Twist())
        self._flush_events()
        snapshot = self.runtime.evidence_snapshot()
        with open(os.path.join(self._run_dir, 'summary.json'), 'w') as f:
            json.dump(snapshot, f, indent=1, ensure_ascii=False)
        self._events_file.close()
        self._runtime_csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = NavRuntimeNode()
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
