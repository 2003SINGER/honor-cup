import os  # 系统文件操作
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration  # 引用启动参数的值


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("M3Pro_navigation"), "map", "yahboom_map.yaml"
        ),
    )

    param_file_name = "yahboom_M3Pro.yaml"
    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("M3Pro_navigation"), "param", param_file_name
        ),
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=map_dir,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_launch_file_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_dir,
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
                }.items(),
            ),
        ]
    )
