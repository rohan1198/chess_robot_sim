from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    pkg_share = FindPackageShare('so_101_arm')
    
    # URDF file path
    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'so_101_arm_6dof.urdf'
    ])
    
    # Model path for Gazebo
    model_path = PathJoinSubstitution([pkg_share, 'models'])
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),
        
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            additional_env={'GAZEBO_MODEL_PATH': model_path.perform(None)}
        ),
        
        # Spawn robot
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_so_101_arm',
                    arguments=[
                        '-file', urdf_file,
                        '-entity', 'so_101_arm',
                        '-x', '0', '-y', '0', '-z', '0.1'
                    ],
                    output='screen'
                )
            ]
        ),
        
        # Load controllers
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=9.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
                    output='screen'
                )
            ]
        ),
    ])
