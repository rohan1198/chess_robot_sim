#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo GUI",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("so_arm_101_gazebo"), "config", "initial_positions.yaml"]
            ),
            description="Configuration file of robot initial positions for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("so_arm_101_gazebo"), "worlds", "chess_world.sdf"]
            ),
            description="Chess world file for Gazebo simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_x",
            default_value="0.7475",
            description="X position of the robot base",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_y", 
            default_value="0.275",
            description="Y position of the robot base",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_z",
            default_value="0.375",  # Table center height (robot sits ON table)
            description="Z position of the robot base",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_yaw",
            default_value="0.0",  # Facing straight toward the board center for symmetric reach
            description="Yaw rotation of the robot base (radians)",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    world_file = LaunchConfiguration("world_file")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y") 
    robot_z = LaunchConfiguration("robot_z")
    robot_yaw = LaunchConfiguration("robot_yaw")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("so_arm_101_gazebo"), "urdf", "so_arm_101_gazebo.urdf.xacro"]
            ),
            " initial_positions_file:=",
            initial_positions_file,
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={"gz_args": ["-r -v4 ", world_file], "on_exit_shutdown": "true"}.items(),
    )

    # Spawn robot at specified position
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_so_arm_101",
        arguments=[
            "-topic", "robot_description",
            "-name", "so_arm_101",
            "-allow_renaming", "true",
            "-x", robot_x,
            "-y", robot_y, 
            "-z", robot_z,
            "-R", "0.0",
            "-P", "0.0",
            "-Y", robot_yaw,
        ],
        output="screen",
    )

    # Bridge for clock
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # Bridge for camera (chess camera)
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            "/chess_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/chess_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen",
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Arm controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Gripper controller spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Delay controller spawners after robot spawn
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper_controller_after_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # Camera bridge delay (start after robot is spawned)
    delay_camera_bridge_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[camera_bridge],
        )
    )

    # Create info node to display chess environment information
    chess_info_node = Node(
        package="so_arm_101_gazebo",
        executable="chess_info.py",
        name="chess_info",
        output="screen",
        parameters=[
            {"robot_position": [robot_x, robot_y, robot_z, robot_yaw]},
        ],
        condition=lambda context: True,  # Always run
    )

    nodes = [
        robot_state_publisher_node,
        gazebo,
        spawn_robot,
        clock_bridge,
        delay_joint_state_broadcaster_after_spawn,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_gripper_controller_after_arm_controller,
        delay_camera_bridge_after_spawn,
        # chess_info_node,  # Commented out since we haven't created this script yet
    ]

    return LaunchDescription(declared_arguments + nodes)
