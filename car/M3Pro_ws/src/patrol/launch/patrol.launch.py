import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
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
    patrol= Node(
            package='patrol',
            executable='patrol',
            name='patrol',
            parameters=[
                {'base_frame': 'base_footprint'},
                {'odom_frame': 'odom'}
                ],
            output='screen'
            )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_filter_madgwick_launch_file)#/imu滤波节点
        ),

        #ekf定位包，发布融合后的odom-base_rootpringt的tf变换
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_odom_launch_file)
        ),
        patrol,

    ])



