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

    imu_filter_madgwick_launch_file = os.path.join(
        get_package_share_directory('imu_filter_madgwick'),
        'launch',
        'imu_filter.launch.py'
        
    )

    ekf_odom_launch_file = os.path.join(
        get_package_share_directory('ekf_bringup'),
        'launch',
        'ekf.launch.py'
        
    )

    updatecostmap= Node(
            package='updatecostmap',
            executable='update_costmap',
            name='update_costmap',
            output='screen')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_filter_madgwick_launch_file)#/imu滤波节点
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_merge_launch_file)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_filter_launch_file)
        ),

        #ekf定位包，发布融合后的odom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_odom_launch_file)
        ),

        # updatecostmap,

    ])














