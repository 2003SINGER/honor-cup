import os   #系统文件操作
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    laser_merge_launch_file = os.path.join(
        get_package_share_directory('ira_laser_tools'),
        'launch',
        'merge_multi.launch.py'
    )
    laser_filter_launch_file = os.path.join(
        get_package_share_directory('yahboom_laser_filter'),
        'launch',
        'laser_filter_node.launch.py'
    )


    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_merge_launch_file)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_filter_launch_file)
        )

    ])














