"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import Shutdown

def generate_launch_description():

    load_state_filename_arg = DeclareLaunchArgument(
        'load_state_filename',
        default_value='/home/jetson/yahboom_map.pbstream'
        )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
    )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', FindPackageShare('M3Pro_navigation').find('M3Pro_navigation') + '/param',
            '-configuration_basename', 'yahboom_M3Pro.lua',
            '-load_state_filename', LaunchConfiguration('load_state_filename')],
        remappings = [
            ('imu', '/imu/data')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )
    

    return LaunchDescription([
        load_state_filename_arg,
        use_rviz_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,

    ])

    