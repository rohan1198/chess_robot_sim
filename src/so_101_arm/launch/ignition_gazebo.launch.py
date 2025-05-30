from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import re

def process_urdf_for_gazebo(context, *args, **kwargs):
    """Convert package:// URIs to file:// URIs for Gazebo compatibility"""
    
    # Get the URDF content first
    pkg_share = FindPackageShare('so_101_arm')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'so_101_arm_6dof.urdf'])
    
    # Read the URDF file
    try:
        package_share_dir = get_package_share_directory('so_101_arm')
        urdf_path = os.path.join(package_share_dir, 'urdf', 'so_101_arm_6dof.urdf')
        
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
        
        # Replace package:// with file:// using the actual package path
        def replace_package_uri(match):
            relative_path = match.group(1)
            full_path = os.path.join(package_share_dir, relative_path)
            return f'file://{full_path}'
        
        # Pattern to match package://so_101_arm/...
        pattern = r'package://so_101_arm/([^"]+)'
        processed_content = re.sub(pattern, replace_package_uri, urdf_content)
        
        return processed_content
        
    except Exception as e:
        print(f"Error processing URDF for Gazebo: {e}")
        # Fallback to original URDF if processing fails
        urdf_cmd = Command(['cat ', urdf_file])
        return urdf_cmd.perform(context)

def generate_launch_description():
    
    # Get package path
    pkg_share = FindPackageShare('so_101_arm')
    
    # Get the actual install path for models
    models_path = PathJoinSubstitution([pkg_share, 'models'])
    
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

        # Robot State Publisher with original URDF (uses package:// URIs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'cat ',
                    PathJoinSubstitution([
                        pkg_share,
                        'urdf',
                        'so_101_arm_6dof.urdf'
                    ])
                ]),
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

        # Spawn robot in Gazebo using processed URDF
        TimerAction(
            period=5.0,
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

        # Load joint state broadcaster
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                    output='screen'
                )
            ]
        ),

        # Load joint trajectory controller
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
                    output='screen'
                )
            ]
        ),

        # Load gripper controller
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
                    output='screen'
                )
            ]
        ),
    ])
