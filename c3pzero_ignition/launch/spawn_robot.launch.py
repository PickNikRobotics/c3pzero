# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):

    # Configure launch arguments
    simulation = LaunchConfiguration("simulation")
    rviz = LaunchConfiguration("rviz")

    # Configure some helper variables and paths
    pkg_robot_description = get_package_share_directory("c3pzero_description")
    pkg_robot_ignition = get_package_share_directory("c3pzero_ignition")

    rviz_config_file = PathJoinSubstitution(
        [pkg_robot_ignition, "rviz", "ignition.rviz"]
    )

    # Parse xacro and publish robot state
    robot_description_path = os.path.join(
        pkg_robot_description, "urdf", "c3pzero_base.xacro"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
            " ",
            "simulation:=",
            simulation,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn the robot
    spawn_node = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-topic",
            "robot_description",
            "-z",
            "1.5",
        ],
        output="screen",
    )

    # IMU Sensor Static Tf
    imu_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_static_transform_publisher",
        output="log",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            "robot/base_link/imu",
        ],
    )

    lidar_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_static_transform_publisher",
        output="log",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "lidar_link",
            "robot/base_link/lidar",
        ],
    )

    ign_ros_bridge_node = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            # cmd_vel bridge (ROS2 -> IGN)
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            # Clock (IGN -> ROS2)
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # Lidar (IGN -> ROS2)
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            # Joint states (IGN -> ROS2)
            "/world/default/model/robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            # IMU (IGN -> ROS2)
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # Ground Truth Odometry (IGN -> ROS2)
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # Wheel Odometry (IGN -> ROS2)
            "/wheel/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ],
        remappings=[
            ("/world/default/model/robot/joint_state", "joint_states"),
            ("/imu", "imu/data"),
        ],
        output="screen",
    )

    # Broadcast tf from odom
    tf_broadcaster_node = Node(
        package="c3pzero_ignition",
        executable="odom2tf_broadcaster",
        name="odom2tf_broadcaster",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    nodes_and_launches = [
        robot_state_publisher_node,
        spawn_node,
        rviz_node,
        imu_static_tf_node,
        tf_broadcaster_node,
        ign_ros_bridge_node,
        lidar_static_tf_node,
    ]

    return nodes_and_launches


def generate_launch_description():

    declared_arguments = []
    simulation_arg = DeclareLaunchArgument(
        "simulation",
        description="If true, enable simulated parameters in urdf",
        default_value="true",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", description="If true, launch rviz", default_value="false"
    )
    declared_arguments.append(simulation_arg)
    declared_arguments.append(rviz_arg)

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
