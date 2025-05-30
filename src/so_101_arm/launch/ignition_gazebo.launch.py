from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Get package path
    pkg_share = FindPackageShare('so_101_arm')
    
    # Get URDF content using Command
    robot_description_content = Command([
        'cat ',
        PathJoinSubstitution([
            pkg_share,
            'urdf',
            'so_101_arm_6dof.urdf'
        ])
    ])

    return LaunchDescription([
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
        
        # Launch Gazebo - Use ign gazebo for Fortress (ROS2 Humble default)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4'],
            output='screen'
        ),
        
        # Bridge for ROS-Gazebo communication
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            output='screen'
        ),

        # Spawn robot in Gazebo
        TimerAction(
            period=5.0,  # Increased delay to ensure Gazebo starts
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_so_101_arm',
                    arguments=[
                        '-file', PathJoinSubstitution([
                            pkg_share,
                            'urdf',
                            'so_101_arm_6dof.urdf'
                        ]),
                        '-name', 'so_101_arm',
                        '-x', '0',
                        '-y', '0',
                        '-z', '0.1'
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
