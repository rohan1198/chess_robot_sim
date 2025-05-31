from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package path
    pkg_share = FindPackageShare('so_101_arm')
    
    # Get the actual install path for models
    models_path = PathJoinSubstitution([pkg_share, 'models'])
    
    # Robot description parameter
    robot_description_content = ParameterValue(
        Command([
            'cat ',
            PathJoinSubstitution([
                pkg_share,
                'urdf',
                'so_101_arm_6dof.urdf'
            ])
        ]),
        value_type=str
    )
    
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
        
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '--verbose'],
            output='screen',
            additional_env={
                'IGN_GAZEBO_RESOURCE_PATH': models_path,
                'GZ_SIM_RESOURCE_PATH': models_path,
                'GAZEBO_MODEL_PATH': models_path
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

        # Spawn robot in Gazebo - wait longer for Gazebo to be ready
        TimerAction(
            period=10.0,  # Increased delay to ensure Gazebo is fully loaded
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

        # Load and start controllers with proper sequencing
        TimerAction(
            period=15.0,  # Wait for robot to be spawned
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=17.0,  # Start joint_state_broadcaster first
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'joint_state_broadcaster', 'active'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=19.0,  # Load trajectory controller
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint_trajectory_controller'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=21.0,  # Start trajectory controller
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'joint_trajectory_controller', 'active'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=23.0,  # Load gripper controller
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'gripper_controller'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=25.0,  # Start gripper controller
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'gripper_controller', 'active'],
                    output='screen'
                )
            ]
        ),
    ])
