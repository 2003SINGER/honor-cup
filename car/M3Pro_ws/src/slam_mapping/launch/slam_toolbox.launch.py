from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    declared_arguments = []
    declared_env_vars = []
    declared_parameters = []
    
    
    
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

    
    imu_filter_madgwick_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
        )
        
    ekf_odom_launch_file = os.path.join(
        get_package_share_directory('ekf_bringup'),
        'launch',
        'ekf.launch.py'
    )
    
    
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_mapping'),
        'launch',
        'online_async_launch.py'
    )
    

    return LaunchDescription([
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_merge_launch_file)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_filter_launch_file)
        ),
        
        imu_filter_madgwick_node,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_odom_launch_file)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file)
        )

        
    ])
