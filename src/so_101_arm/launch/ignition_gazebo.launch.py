from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package path
    pkg_share = FindPackageShare('so_101_arm')
    
    # Path to custom world file
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'chess_table_world.sdf'])
    
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
        parameters=[
            {'robot_description': robot_description_content},
            controller_config, 
            {'use_sim_time': True}
        ],
        output='screen',
    )
    
    # Launch Ignition Gazebo with custom world file
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', world_file],
        output='screen',
    )
    
    # Bridge for ROS-Gazebo communication
    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/so_101_arm/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
             ('/model/so_101_arm/pose', '/tf')
        ],
        output='screen'
    )

    # Spawn robot in Ignition Gazebo - FIXED positioning for proper contact with mount
    spawn_robot_timer = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-file', '/tmp/so_101_arm_processed.urdf',
                    '-name', 'so_101_arm',
                    '-x', '0.4',
                    '-y', '0',
                    '-z', '0.885',  # FIXED: Adjusted height to account for collision geometry offset
                    '-R', '0',
                    '-P', '0',
                    '-Y', '-1.5708'  # 90 degrees clockwise rotation to match mount orientation
                ],
                output='screen'
            )
        ]
    )
    
    # Create the processed URDF file before spawning
    create_urdf_timer = TimerAction(
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
    )

    # Robot should now be properly positioned on the mount platform
    # The heavy base mass (50.0 kg) and correct positioning should keep it stable

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner_timer = TimerAction(
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

    # Arm controller spawner
    arm_controller_spawner_timer = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # Gripper controller spawner
    gripper_controller_spawner_timer = TimerAction(
        period=21.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Auto-start simulation
    start_simulation_timer = TimerAction(
        period=24.0,
        actions=[
            ExecuteProcess(
                cmd=['ign', 'service', '-s', '/world/chess_table_world/control', 
                     '--reqtype', 'ignition.msgs.WorldControl', 
                     '--reptype', 'ignition.msgs.Boolean', 
                     '--timeout', '1000', '--req', 'pause: false'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Set environment variables
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[PathJoinSubstitution([pkg_share, 'models']), os.pathsep, os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', 
            value=[PathJoinSubstitution([pkg_share, 'models']), os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=[os.path.join(get_package_share_directory('gz_ros2_control'), 'lib'),
                   os.pathsep,
                   os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]
        ),

        # Start core nodes
        robot_state_publisher_node,
        controller_manager_node,
        
        # Start Gazebo
        gazebo_process,
        gazebo_bridge_node,
        
        # Create URDF and spawn robot
        create_urdf_timer,
        spawn_robot_timer,
        
        # Spawn controllers
        joint_state_broadcaster_spawner_timer,
        arm_controller_spawner_timer,
        gripper_controller_spawner_timer,
        
        # Start simulation
        start_simulation_timer,
    ])
