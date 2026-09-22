#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.ros_planning_interface import MoveGroupInterface
import random
import math

class RandomArmMovement(Node):
    def __init__(self):
        super().__init__('random_arm_mover')
        
        # 初始化MoveGroupInterface，参数为规划组名称（根据实际机器人配置修改）
        self.move_group = MoveGroupInterface("arm_group", "robot_description", None)

        # 设置规划参数（可选）
        self.move_group.set_max_velocity_scaling_factor(0.5)  # 最大速度比例
        self.move_group.set_max_acceleration_scaling_factor(0.5)  # 最大加速度比例

        # 获取关节名称列表（根据实际机器人配置修改）
        self.joint_names = [
            "arm1_Joint",
            "arm2_Joint",
            "arm3_Joint",
            "arm4_Joiint",
            "arm5_Joint"
        ]

        # 定义关节运动范围（根据实际机器人配置修改）
        self.joint_limits = {
            "arm1_Joint": (-1.57,1.57),
            "arm1_Joint": (-1.57,1.57),
            "arm1_Joint": (-1.57,1.57),
            "arm1_Joint": (-1.57,1.57),
            "arm1_Joint": (-1.57,1.57),
        }

    def generate_random_joint_angles(self):
        """生成随机关节角度"""
        return {
            joint: random.uniform(self.joint_limits[joint][0], self.joint_limits[joint][1])
            for joint in self.joint_names
        }

    def move_to_random_position(self):
        """移动到随机位置"""
        # 生成随机目标
        target_joints = self.generate_random_joint_angles()
        
        # 设置关节目标
        self.move_group.set_joint_value_target(target_joints)

        # 执行运动规划
        success = self.move_group.move()

        # 返回运动结果
        return success

def main(args=None):
    rclpy.init(args=args)
    
    arm_mover = RandomArmMovement()
    
    try:
        while rclpy.ok():
            # 执行随机运动
            success = arm_mover.move_to_random_position()
            
            if success:
                arm_mover.get_logger().info("Movement completed successfully!")
            else:
                arm_mover.get_logger().warn("Movement planning failed!")
            
            # 等待5秒后执行下一次运动
            rclpy.spin_once(arm_mover, timeout_sec=5.0)
            
    except KeyboardInterrupt:
        arm_mover.get_logger().info("Shutting down...")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()