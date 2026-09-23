from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='m3pro_nav',
            executable='decision',
            name='m3pro_decision',
            output='screen',
            parameters=[{
                'v_max': 0.35,          # 提速阶梯: 0.2 起步, 实测上调（下位机钳位 0.7）
                'corner_mode': 'holo',  # 过弯: pivot|holo|arc
            }],
        ),
    ])
