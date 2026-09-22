from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    map_name = "yahboom_map"
    default_map_path = os.path.join(get_package_share_path("M3Pro_navigation"), 'map', map_name)

    map_arg = DeclareLaunchArgument(name='map_path', default_value=str(default_map_path),
                                    description='The path of the map')

    map_saver_node = Node(
    package='nav2_map_server',
    executable='map_saver_cli',
    arguments=[
        '-f', LaunchConfiguration('map_path'),'--free', '0.196','--occ', '0.65'],
    )

    return LaunchDescription([
        map_arg,
        map_saver_node
    ])

