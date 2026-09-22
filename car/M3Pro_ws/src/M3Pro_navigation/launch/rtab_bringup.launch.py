import os   #ÏµÍ³ÎÄ¼þ²Ù×÷
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

    camera_launch_file = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'dabai_dcw2.launch.py'
        
    )

    updatecostmap= Node(
            package='updatecostmap',
            executable='update_costmap',
            name='update_costmap',
            output='screen')

    base_footprint_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera',
            arguments=['0.05', '0.0', '0.38' ,'0', '0', '0', 'base_link','camera_link']
        )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_filter_madgwick_launch_file)#/imuÂË²¨½Úµã
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_merge_launch_file)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_filter_launch_file)
        ),

        #ekf¶¨Î»°ü£¬·¢²¼ÈÚºÏºóµÄodom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_odom_launch_file)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_file)
        ),


        updatecostmap,

        base_footprint_tf,

    ])















