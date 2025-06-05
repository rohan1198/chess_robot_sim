#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("so_arm_101_gazebo"), "config", "initial_positions.yaml"]
            ),
            description="Configuration file of robot initial positions",
        )
    )

    # Initialize Arguments
    initial_positions_file = LaunchConfiguration("initial_positions_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("so_arm_101_gazebo"), "urdf", "so_arm_101.urdf.xacro"]
            ),
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

    # Joint state publisher GUI (for manual control)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
