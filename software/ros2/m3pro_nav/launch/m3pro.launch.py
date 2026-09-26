from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='m3pro_nav',
            executable='driver_probe',
            name='m3pro_driver_probe',
            output='screen',
        ),
    ])
