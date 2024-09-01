from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='tb3_simulation_launch.py',  # Using a different launch file
            name='nav2_bringup',
            output='screen',
        ),
        Node(
            package='webots_ros2_driver',
            executable='driver',
            name='webots_driver',
            output='screen',
        ),
    ])
