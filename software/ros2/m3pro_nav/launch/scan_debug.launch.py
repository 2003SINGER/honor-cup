"""scan_debug.launch.py —— 实车静态感知调试一键启动.

用法:
    ros2 launch m3pro_nav scan_debug.launch.py cell_x:=3 cell_y:=2 heading:=N
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HEADING_WORDS = {'N': 'North', 'E': 'East', 'S': 'South', 'W': 'West'}


def string_parameter(name):
    """Prevent launch's YAML coercion from turning strings like N into bools."""
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def heading_parameter():
    """Map YAML-1.1 boolean-like compass letters to unambiguous words."""
    mapping = str(HEADING_WORDS).replace(' ', '')
    return ParameterValue(
        PythonExpression([
            f'{mapping}["', LaunchConfiguration('heading'), '"]',
        ]),
        value_type=str,
    )


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
            'heading': heading_parameter(),
            'scan_topic': string_parameter('scan_topic'),
            'odom_topic': string_parameter('odom_topic'),
            'imu_topic': string_parameter('imu_topic'),
            'expected_laser_frame': string_parameter('expected_laser_frame'),
            'expected_odom_frame': string_parameter('expected_odom_frame'),
            'expected_base_frame': string_parameter('expected_base_frame'),
            'use_tf_extrinsic': LaunchConfiguration('use_tf_extrinsic'),
            'laser_extrinsic_yaml': string_parameter('laser_extrinsic_yaml'),
            'session_dir': string_parameter('session_dir'),
        }],
    )
    return LaunchDescription(args + [node])
