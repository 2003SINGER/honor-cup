#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
决策层节点骨架 —— 英才杯 A 题探索/定位/控制的上位机核心

数据流（见 docs/design/感知与定位.md 第 7 节）：
    /scan0 /scan1          → 墙登记册（地图系吸附定位 + 里程计校正）
    /odom_raw /imu/data_raw → 高频推算（被上者持续校准）
    /camera/color/image_raw → 巡线 PID（保底）+ 路口检测 + 方块识别
    决策层（MazeMap frontier）→ 融合控制器 → /cmd_vel

状态机（与仿真 explore/localize 一致）：
    IDLE → EXPLORE（MazeMap frontier DFS）→ [出口已见 且 方块收齐] → RETURN（path_between）
    丢线 → 位置环兜底导航到最近路口

标 ⛔ 的地方 = 必须实车填的传感器接口，其余逻辑已在仿真验证过。
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image

from .mazemap import MazeMap          # 与仿真同一份数据结构
from .tracker import HolonomicTracker  # 与仿真同一份跟踪控制器

ROS_DOMAIN_ID = 30


class DecisionNode(Node):

    def __init__(self):
        super().__init__('m3pro_decision')
        # ---- 参数 ----
        self.declare_parameter('v_max', 0.35)        # 提速: 0.2 → 实测阶梯上调（上限 0.7）
        self.declare_parameter('t_turn_pivot', 1.5)  # 停-转-走单次耗时（保底模式用）
        self.declare_parameter('corner_mode', 'holo')  # pivot|holo|arc（仿真已对比）

        v_max = self.get_parameter('v_max').value
        self.map = MazeMap(n=7, entry=(0, 0))        # ⛔ 入口格按实际摆放填
        self.tracker = HolonomicTracker(v_max=v_max)
        self.path = []                               # 当前剩余路径（世界系, m）
        self.state = 'IDLE'

        # ---- 订阅（⛔ 实车核对话题名与类型）----
        self.sub_scan0 = self.create_subscription(LaserScan, '/scan0', self.on_scan0, 1)
        self.sub_scan1 = self.create_subscription(LaserScan, '/scan1', self.on_scan1, 1)
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.on_rgb, 1)
        self.sub_odom = self.create_subscription(
            __import__('nav_msgs.msg', fromlist=['Odometry']).Odometry,
            '/odom_raw', self.on_odom, 1)

        # ---- 发布 ----
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 1)

        # ---- 控制循环 30Hz（与相机帧率同步即可，非硬实时）----
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)
        self.get_logger().info('decision node ready (state=IDLE)')

    # ---------- 传感器回调（⛔ 实车填写） ----------

    def on_scan0(self, msg: LaserScan):
        """左后雷达：墙登记册的观测来源之一"""
        # TODO: _raycast 对齐——把 msg.ranges 转成「向 ±x/±y 的墙距观测」
        #       喂给定位器（reg 模式），参考 software/sim/maze_sim.py localize()
        pass

    def on_scan1(self, msg: LaserScan):
        """右前雷达：同上"""
        pass

    def on_odom(self, msg):
        """里程计推算（高频预测），由雷达登记册持续校正"""
        # TODO: 更新 self.pose (x, y, th)
        pass

    def on_rgb(self, msg: Image):
        """相机三职：巡线偏差 / 路口分叉 / 方块识别"""
        # TODO: ① Roi_hsv 标定后的黑线提取 → 线的米制横向偏差（地面平面投影）
        #       ② 路口分叉形态 → 决策层 junction 事件
        #       ③ color_recognize 四色 → 方块
        pass

    # ---------- 决策主循环（逻辑已在仿真验证） ----------

    def on_timer(self):
        if self.state == 'IDLE':
            return
        if self.state == 'EXPLORE':
            # 与仿真 explore() 同构：
            #   路口到达 → map.open_edge 全分支 → frontier 选向（order 优先级）
            #   无 frontier → 栈回溯
            #   出口已见 且 方块收齐 → state=RETURN, path = map.path_between(cur, exit)
            pass
        elif self.state == 'RETURN':
            pass

        # 融合控制器（Phase 3）：位置环 + 巡线 PID 加权
        vx, vy, wz = self.fused_control()
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy          # 麦轮横移（不转向过弯用）
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)

    def fused_control(self):
        """位置环(主) + 巡线PID(保底) 加权融合，权重随置信度动态调整"""
        vx_p, vy_p, wz_p, _ = self.tracker.update(self.pose(), self.path)
        # TODO Phase 3: 巡线 PID 输出 vx_l/y_l（线的米制横向偏差），
        #   权重 w = 线检测置信度；直道两路天然一致，丢线时 w→0 自动全给位置环
        w = 0.0                                 # 保底未就绪前 = 纯位置环
        return ((1 - w) * vx_p, (1 - w) * vy_p, wz_p)

    def pose(self):
        """当前位姿 (x, y, th)：定位器输出（⛔ 接雷达登记册+里程计融合）"""
        return (0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
