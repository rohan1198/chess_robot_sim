from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF file content using cat command
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
                'robot_description': robot_description_content,
                'use_sim_time': False
            }]
        ),
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('so_101_arm'),
                'config',
                'urdf.rviz'
            ])],
            output='screen'
        )
    ])
