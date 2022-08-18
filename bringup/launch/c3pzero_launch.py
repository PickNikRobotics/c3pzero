from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cp3zero_bringup',
            executable='cp3zero_node',
            parameters=[],
            arguments=[],
            output='screen')
    ])