import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess
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
    # world = LaunchConfiguration("world")
    # robot = LaunchConfiguration("robot")
    simulation = LaunchConfiguration("simulation")
    rviz = LaunchConfiguration("rviz")

    # Configure some helper variables and paths
    # world_name = world.perform(context)
    # robot_name = robot.perform(context)
    pkg_robot_description = get_package_share_directory("c3pzero_description")
    pkg_robot_ignition = get_package_share_directory("c3pzero_ignition")

    rviz_config_file = PathJoinSubstitution(
        [pkg_robot_ignition, "rviz", "ignition.rviz"]
    )

    # Parse xacro and publish robot state
    robot_description_path = os.path.join(
        pkg_robot_description, "urdf",  "c3pzero_base.xacro"
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
    imu_static_tf = Node(
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
            "robot/imu_link/imu",
        ],
    )

    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            # JointTrajectory bridge (ROS2 -> IGN)
            "/joint_trajectory_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory",
            # Wheel Joint Trajectory bridge (ROS2 -> IGN)
            "/velocity_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory",
            # Clock (IGN -> ROS2)
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # JointTrajectoryProgress bridge (IGN -> ROS2)
            "/joint_trajectory_controller/joint_trajectory_progress@std_msgs/msg/Float32[ignition.msgs.Float",
            # Wheel JointTrajectory Progress bridge (IGN -> ROS2)
            "/velocity_controller/joint_trajectory_progress@std_msgs/msg/Float32[ignition.msgs.Float",
            # IMU (IGN -> ROS2)
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # Odometry (IGN -> ROS2)
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ],
        remappings=[
            # ("/world/" + world_name + "/model/robot/joint_state", "joint_states"),
            ("/imu", "imu/data"),
        ],
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

    ros2_controllers_path = os.path.join(
        pkg_robot_description,
        "config",
        "c3pzero_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "diff_drive_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )

    nodes_and_launches = [
        robot_state_publisher_node,
        spawn_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        # rviz_node,
        # imu_static_tf,
        # bridge,
    ]

    return nodes_and_launches  # + load_controllers


def generate_launch_description():

    declared_arguments = []
    simulation_arg =  DeclareLaunchArgument(
        "simulation",
        description="If true, enable simulated parameters in urdf",
        default_value="true"
    )
    rviz_arg =  DeclareLaunchArgument(
        "rviz",
        description="If true, launch rviz",
        default_value="false"
    )
    declared_arguments.append(simulation_arg)
    declared_arguments.append(rviz_arg)
    # declared_arguments.append(launch_helper.DeclareWorldArg())
    # declared_arguments.append(launch_helper.DeclareRobotArg())

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
