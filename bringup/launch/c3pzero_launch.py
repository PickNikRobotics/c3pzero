from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cp3zero_node',
            executable='cp3zero_exec',
            parameters=[],
            arguments=[],
            output='screen')
    ])