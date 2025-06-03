from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('chess_robot_sim')
    
    controller_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'controllers_6dof.yaml'
    ])
    
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                pkg_share,
                'urdf',
                'so_arm101.urdf.xacro'
            ]),
            ' controller_config_file:=',
            controller_config
        ]),
        value_type=str
    )

    return LaunchDescription([
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

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('chess_robot_sim'),
                'config',
                'urdf.rviz'
            ])],
            parameters=[{
                'use_sim_time': True
            }],
            output='screen'
        )
    ])
