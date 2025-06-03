import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare('chess_robot_sim')
    
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'chess_table_world.sdf'])
    
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_share, 'urdf', 'so_arm101.urdf.xacro']),
            ' controller_config_file:=',
            PathJoinSubstitution([pkg_share, 'config', 'controllers_6dof.yaml'])
        ]),
        value_type=str
    )
    
    controller_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'controllers_6dof.yaml'
    ])
    
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
    
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', world_file],
        output='screen',
    )
    
    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/chess_robot_sim/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
             ('/model/chess_robot_sim/pose', '/tf')
        ],
        output='screen'
    )

    spawn_robot_timer = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-file', '/tmp/so_arm101_processed.urdf',
                    '-name', 'so_arm101',
                    '-x', '0.35',
                    '-y', '0.02',
                    '-z', '0.778',
                    '-R', '0',
                    '-P', '0',
                    '-Y', '-1.5708',
                ],
                output='screen'
            )
        ]
    )
    
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

        robot_state_publisher_node,
        controller_manager_node,
        
        gazebo_process,
        gazebo_bridge_node,
        
        create_urdf_timer,
        spawn_robot_timer,
        
        joint_state_broadcaster_spawner_timer,
        arm_controller_spawner_timer,
        gripper_controller_spawner_timer,
        
        start_simulation_timer,
    ])
