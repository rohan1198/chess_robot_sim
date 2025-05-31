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
    
    # Generate robot description with correct .xacro file
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_share, 'urdf', 'so_101_arm_6dof.urdf.xacro']),
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
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )
    
    # Controller Manager with configuration
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {'use_sim_time': True}],
        output='screen'
    )
    
    # Launch Ignition Gazebo with AUTOSTART (-r flag)
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r'],  # -r flag starts simulation immediately
        output='screen',
        additional_env={
            'IGN_GAZEBO_RESOURCE_PATH': str(models_path),
            'GZ_SIM_RESOURCE_PATH': str(models_path)
        }
    )
    
    # Bridge for ROS-Gazebo communication
    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # Spawn robot in Ignition Gazebo - wait for Gazebo to be ready
    spawn_robot_timer = TimerAction(
        period=8.0,  # Reduced wait time since simulation auto-starts
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', 
                    PathJoinSubstitution([pkg_share, 'scripts', 'spawn_robot_gazebo.py'])
                ],
                output='screen'
            )
        ]
    )

    # Auto-start simulation after robot is spawned
    start_simulation_timer = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['ign', 'service', '-s', '/world/default/control', '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000', '--req', 'pause: false'],
                output='screen'
            )
        ]
    )

    # Joint state broadcaster spawner - wait for simulation to start
    joint_state_broadcaster_timer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Arm controller spawner - wait for joint state broadcaster
    arm_controller_timer = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Gripper controller spawner - wait for arm controller
    gripper_controller_timer = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Set environment variables for Ignition Gazebo resource resolution
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[str(models_path), ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
        ),
        
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', 
            value=[str(models_path), ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=['/opt/ros/humble/lib:', os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]
        ),

        # Start core nodes
        robot_state_publisher_node,
        controller_manager_node,
        gazebo_process,
        gazebo_bridge_node,
        
        # Spawn robot after delay
        spawn_robot_timer,
        
        # Start simulation
        start_simulation_timer,
        
        # Start controllers with delays
        joint_state_broadcaster_timer,
        arm_controller_timer,
        gripper_controller_timer,
    ])
