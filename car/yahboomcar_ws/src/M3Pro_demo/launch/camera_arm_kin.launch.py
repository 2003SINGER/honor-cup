from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    camera_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch'),
         '/dabai_dcw2.launch.py'])
    )

    kin_node = Node(
     package='arm_kin',
     executable='kin_srv',
     name='kin_ik_fk',
    )

    
    return LaunchDescription([camera_driver_launch,kin_node])

