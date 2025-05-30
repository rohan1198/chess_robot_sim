from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Get URDF content using xacro command
    robot_description_content = Command([
        'cat ',
        PathJoinSubstitution([
            FindPackageShare('so_101_arm'),
            'urdf',
            'so_101_arm_simple.urdf'
        ])
    ])

    return LaunchDescription([
        # Robot State Publisher (with proper URDF content)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content
            }]
        ),
        
        # Launch newer Gazebo (Ignition)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4'],
            output='screen'
        ),
        
        # Spawn simple robot after delay
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_simple_robot',
                    arguments=[
                        '-file', PathJoinSubstitution([
                            FindPackageShare('so_101_arm'),
                            'urdf',
                            'so_101_arm_simple.urdf'
                        ]),
                        '-name', 'simple_robot',
                        '-x', '0',
                        '-y', '0',
                        '-z', '0.1'
                    ],
                    output='screen'
                )
            ]
        ),
    ])
