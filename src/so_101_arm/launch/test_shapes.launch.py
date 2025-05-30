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
            'simple_shapes.urdf'
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
        
        # ROS-Gazebo bridge
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='bridge',
                    arguments=[
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
                    ],
                    output='screen'
                )
            ]
        ),
        
        # Spawn robot
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
                            'simple_shapes.urdf'
                        ]),
                        '-name', 'simple_robot'
                    ],
                    output='screen'
                )
            ]
        ),
    ])
