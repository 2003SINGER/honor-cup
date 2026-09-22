from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default=os.path.join(get_package_share_directory('slam_mapping'),'rviz','slam_rviz.rviz'))
    rviz_config_arg=DeclareLaunchArgument('rviz_config', default_value=rviz_config)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config,use_sim_time]
    )
    
    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
     
    ])
