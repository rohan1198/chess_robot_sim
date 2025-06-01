from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    auto_start_chess_arg = DeclareLaunchArgument(
        'auto_start_chess',
        default_value='true',
        description='Whether to automatically start the chess controller'
    )
    
    spawn_chess_pieces_arg = DeclareLaunchArgument(
        'spawn_chess_pieces',
        default_value='true',
        description='Whether to spawn chess pieces automatically'
    )
    
    # Get package path
    pkg_share = FindPackageShare('so_101_arm')
    
    # Path to custom world file with chess setup
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
        n='robot_state_publisher',
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
    
    # Launch Ignition Gazebo with chess world
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', world_file],
        output='screen',
    )
    
    # Bridge for ROS-Gazebo communication
    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        n='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/so_101_arm/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
             ('/model/so_101_arm/pose', '/tf')
        ],
        output='screen'
    )

    # Create the processed URDF file before spawning robot
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

    # Spawn robot in Ignition Gazebo
    spawn_robot_timer = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-file', '/tmp/so_101_arm_processed.urdf',
                    '-n', 'so_101_arm',
                    '-x', '0.4',
                    '-y', '0',
                    '-z', '0.7751',
                    '-R', '0',
                    '-P', '0',
                    '-Y', '-1.5708'  # 90 degrees clockwise rotation
                ],
                output='screen'
            )
        ]
    )

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

    # Chess piece controller node
    chess_controller_timer = TimerAction(
        period=30.0,  # Start after everything else is ready
        actions=[
            Node(
                package='so_101_arm',
                executable='chess_piece_controller.py',
                n='chess_piece_controller',
                output='screen',
                condition=IfCondition(LaunchConfiguration('auto_start_chess'))
            )
        ]
    )

    # Optional: RViz for visualization
    rviz_timer = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                n='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('so_101_arm'),
                    'config',
                    'urdf.rviz'
                ])],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Launch arguments
        auto_start_chess_arg,
        spawn_chess_pieces_arg,
        
        # Set environment variables
        SetEnvironmentVariable(
            n='IGN_GAZEBO_RESOURCE_PATH',
            value=[PathJoinSubstitution([pkg_share, 'models']), os.pathsep, os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
        ),
        SetEnvironmentVariable(
            n='GZ_SIM_RESOURCE_PATH', 
            value=[PathJoinSubstitution([pkg_share, 'models']), os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),
        SetEnvironmentVariable(
            n='GZ_SIM_SYSTEM_PLUGIN_PATH',
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
        
        # Start simulation and additional nodes
        start_simulation_timer,
        chess_controller_timer,
        rviz_timer,
    ])
