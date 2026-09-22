from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      
        Node(
            package='yahboom_laser_filter',
            executable='laser_filter_node',
            name='laser_filter_node',
            parameters=[
                {'angle_min': -180.0, 'angle_max': 180.0},  
            ]
        )
    ])
