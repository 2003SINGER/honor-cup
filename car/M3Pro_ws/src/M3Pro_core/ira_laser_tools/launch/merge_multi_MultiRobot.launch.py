from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
import launch

from launch.utilities import perform_substitutions
from launch.launch_context import LaunchContext

def print_parameter_value(context):
    # 直接通过context获取参数值
    param_value = context.launch_configurations.get('ns', 'default_fallback')
    print(f"Parameter value: {param_value}")
    
    # 如果需要使用LaunchConfiguration对象
    param = LaunchConfiguration('ns')
    # 正确的perform_substitutions用法
    resolved_value = param.perform(context)
    print(f"Resolved value: {resolved_value}")
    return []

def generate_launch_description():
    declared_arguments = []
    declared_env_vars = []
    declared_parameters = []

    params_file = LaunchConfiguration("params_file")

    ns_arg = DeclareLaunchArgument('ns', default_value='robot1', description='Name of the robot')


    OpaqueFunction(function=print_parameter_value)

	
	
    declared_arguments.append(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("ira_laser_tools"),
                "config",
                "laserscan_merge_MultiRobot.yaml",
            ),
            description="Path to param config in yaml format",
        ),
    )
    imu_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera',
            arguments=['0.111766', '-0.0916', '0.0', '0', '0', '0', 'robot1/base_link', "robot1/imu_frame"],
	        namespace=launch.substitutions.LaunchConfiguration('namespace')		
        )
    
    
    laser0_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera',
            arguments=['0.111766', '-0.0916', '0.0', '0', '0', '0', 'robot1/base_link', 'robot1/laser1_frame'],
            namespace=launch.substitutions.LaunchConfiguration('namespace')
        )
        
    laser1_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera',
            arguments=['-0.11117', '0.09156', '0.0', '0', '0', '0', 'robot1/base_link', 'robot1/laser0_frame'],
            namespace=launch.substitutions.LaunchConfiguration('namespace')
        )

    laser_merge = Node(
        package="ira_laser_tools",
        executable="laserscan_multi_merger",
        name="laserscan_multi_merger",
        parameters=[params_file],
        output="both",
        namespace=launch.substitutions.LaunchConfiguration('namespace')
        # respawn=True,
        # respawn_delay=2,
    )

    
    X5PLUS_V15_launch_file = os.path.join(
        get_package_share_directory('M3Pro'),
        'launch',
        'display_Robot1.launch.py'
    )

    nodes = [laser1_tf, laser0_tf, imu_tf,laser_merge, IncludeLaunchDescription(
            PythonLaunchDescriptionSource(X5PLUS_V15_launch_file)
       )]
    

    return LaunchDescription(
        declared_parameters + declared_arguments + declared_env_vars + nodes+ ns_arg
		
    )
