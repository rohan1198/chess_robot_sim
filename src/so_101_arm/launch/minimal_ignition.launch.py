from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Get URDF content using cat command
    robot_description_content = Command([
        'cat ',
        PathJoinSubstitution([
            FindPackageShare('so_101_arm'),
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
                'robot_description': robot_description_content
            }]
        ),
        
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4'],
            output='screen'
        ),
        
        # Simple spawn after delay (no controllers)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_robot',
                    arguments=[
                        '-file', PathJoinSubstitution([
                            FindPackageShare('so_101_arm'),
                            'urdf',
                            'so_101_arm_6dof.urdf'
                        ]),
                        '-name', 'so_101_arm'
                    ],
                    output='screen'
                )
            ]
        ),
    ])
