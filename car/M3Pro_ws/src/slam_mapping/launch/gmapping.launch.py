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
    
    
    ekf_odom_launch_file = os.path.join(
        get_package_share_directory('ekf_bringup'),
        'launch',
        'ekf.launch.py'
    )
    
    gmapping_launch_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'launch',
        'slam_gmapping.launch.py'
        
    )
    
    imu_filter_madgwick_node = Node(
            package='imu_filter_madgwick',
             executable='imu_filter_madgwick_node',
            #  remappings=[('/imu/data_raw', '/imu')],
            remappings=[('/imu','/imu/data_raw')],
             name='imu_filter_madgwick_node',
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
            PythonLaunchDescriptionSource(gmapping_launch_file)
        )
        
        
        


        
    ])
