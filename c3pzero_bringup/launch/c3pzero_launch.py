# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):

    # Configure launch arguments
    world = LaunchConfiguration("world")

    # Configure some helper variables and paths
    pkg_c3pzero_ignition = get_package_share_directory("c3pzero_ignition")
    pkg_c3pzero_navigation = get_package_share_directory("c3pzero_navigation")

    # Bringup robot hardware
    ignition_launch_py = PythonLaunchDescriptionSource(
        [
            pkg_c3pzero_ignition,
            "/launch/ignition.launch.py",
        ]
    )
    ignition_launch = IncludeLaunchDescription(
        ignition_launch_py,
        launch_arguments={
            "world": world,
        }.items(),
    )

    # Spawn simulated robot in Ignition Gazebo
    spawn_launch_py = PythonLaunchDescriptionSource(
        [
            pkg_c3pzero_ignition,
            "/launch/spawn_robot.launch.py",
        ]
    )
    spawn_launch = IncludeLaunchDescription(
        spawn_launch_py,
        launch_arguments={
            "rviz": "False",
        }.items(),
    )

    # Bringup Navigation2
    nav2_launch_py = PythonLaunchDescriptionSource(
        [
            pkg_c3pzero_navigation,
            "/launch/navigation.launch.py",
        ]
    )
    nav2_launch = IncludeLaunchDescription(
        nav2_launch_py,
    )

    nodes_and_launches = [
        ignition_launch,
        TimerAction(period=15.0, actions=[spawn_launch]),
        TimerAction(period=25.0, actions=[nav2_launch]),
    ]

    return nodes_and_launches


def generate_launch_description():
    declared_arguments = []
    world_arg = DeclareLaunchArgument(
        "world",
        description="Name of world to display",
        choices=["empty", "simple", "house"],
        default_value="house",
    )
    declared_arguments.append(world_arg)

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
