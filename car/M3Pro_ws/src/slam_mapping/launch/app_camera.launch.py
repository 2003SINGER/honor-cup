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

        
    camera_launch = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'dabai_dcw2.launch.py'
    )


    
    rgb_image_node = Node(
            package='laserscan_to_point_publisher',
            executable='pub_rgb_image',
            name='publish_rgb_frame',
    )
    
    
    return LaunchDescription([

        rgb_image_node,
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch)
        ),
               
    ])
