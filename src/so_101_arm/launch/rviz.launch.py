from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # FIXED: Get URDF content using ParameterValue with proper type
    robot_description_content = ParameterValue(
        Command([
            'cat ',
            PathJoinSubstitution([
                FindPackageShare('so_101_arm'),
                'urdf',
                'so_101_arm_6dof.urdf'
            ])
        ]),
        value_type=str
    )

    return LaunchDescription([
        # Robot State Publisher with FIXED parameter handling
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content
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
