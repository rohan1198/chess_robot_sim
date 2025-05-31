from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    
    # Get package path
    pkg_share = FindPackageShare('so_101_arm')
    
    # Get the actual install path for models
    models_path = PathJoinSubstitution([pkg_share, 'models'])
    
    # FIXED: Generate robot description with substituted controller config path
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_share, 'urdf', 'so_101_arm_6dof.urdf']),
            ' controller_config_file:=',
            PathJoinSubstitution([pkg_share, 'config', 'controllers_6dof.yaml'])
        ]),
        value_type=str
    )
    
    # Controller configuration
    controller_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'controllers_6dof.yaml'
    ])
    
    return LaunchDescription([
        # Set environment variables for Gazebo resource resolution
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[models_path, ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
        ),
        
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', 
            value=[models_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),
        
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[models_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
        ),

        # FIXED: Add the system plugin path to the environment
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=['/opt/ros/humble/lib:', os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': True
            }]
        ),
        
        # Controller Manager with configuration
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config, {'use_sim_time': True}],
            output='screen'
        ),
        
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '--verbose'],
            output='screen',
            additional_env={
                'IGN_GAZEBO_RESOURCE_PATH': models_path,
                'GZ_SIM_RESOURCE_PATH': models_path,
                'GAZEBO_MODEL_PATH': models_path,
                'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/humble/lib'
            }
        ),
        
        # Bridge for ROS-Gazebo communication
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            output='screen'
        ),

        # Spawn robot in Gazebo
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'python3', 
                        PathJoinSubstitution([pkg_share, 'scripts', 'spawn_robot_gazebo.py'])
                    ],
                    output='screen'
                )
            ]
        ),

        # FIXED: Simplified controller startup using spawner
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=17.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_trajectory_controller'],
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=19.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['gripper_controller'],
                    output='screen'
                ),
            ]
        ),
    ])
