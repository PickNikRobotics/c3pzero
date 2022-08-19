# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    # Ignition Gazebo
    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")
    pkg_c3pzero_ignition = get_package_share_directory("c3pzero_ignition")
    world_str = "-v 3 -r " + os.path.join(
        pkg_c3pzero_ignition, "worlds", world.perform(context) + "_world.sdf"
    )

    if IfCondition(headless).evaluate(context):
        world_str = " -s " + world_str

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py")
        ),
        launch_arguments={"ign_args": world_str}.items(),
    )

    nodes_and_launches = [ignition]

    return nodes_and_launches


def generate_launch_description():
    declared_arguments = []
    world_arg = DeclareLaunchArgument(
        "world",
        description="Name of world to display",
        choices=[
            "empty",
            "simple"
        ],
        default_value="empty",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        description="If true, launch the simulation without UI",
        default_value="false",
    )
    declared_arguments.append(world_arg)
    declared_arguments.append(headless_arg)

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
