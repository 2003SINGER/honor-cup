"""scan_debug.launch.py —— 实车静态感知调试一键启动.

用法:
    ros2 launch m3pro_nav scan_debug.launch.py cell_x:=3 cell_y:=2 heading:=N
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('cell_x', default_value='0'),
        DeclareLaunchArgument('cell_y', default_value='0'),
        DeclareLaunchArgument('heading', default_value='N'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_raw'),
        DeclareLaunchArgument('imu_topic', default_value=''),
        DeclareLaunchArgument('expected_laser_frame', default_value=''),
        DeclareLaunchArgument('expected_odom_frame', default_value='odom'),
        DeclareLaunchArgument('expected_base_frame', default_value='base_link'),
        DeclareLaunchArgument('use_tf_extrinsic', default_value='true'),
        DeclareLaunchArgument('laser_extrinsic_yaml', default_value=''),
        DeclareLaunchArgument('session_dir', default_value=''),
        DeclareLaunchArgument('label', default_value=''),
    ]
    node = Node(
        package='m3pro_nav',
        executable='scan_debug',
        name='scan_debug',
        output='screen',
        parameters=[{
            'cell_x': LaunchConfiguration('cell_x'),
            'cell_y': LaunchConfiguration('cell_y'),
            'heading': LaunchConfiguration('heading'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'expected_laser_frame': LaunchConfiguration('expected_laser_frame'),
            'expected_odom_frame': LaunchConfiguration('expected_odom_frame'),
            'expected_base_frame': LaunchConfiguration('expected_base_frame'),
            'use_tf_extrinsic': LaunchConfiguration('use_tf_extrinsic'),
            'laser_extrinsic_yaml': LaunchConfiguration('laser_extrinsic_yaml'),
            'session_dir': LaunchConfiguration('session_dir'),
        }],
    )
    return LaunchDescription(args + [node])
