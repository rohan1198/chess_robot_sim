from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Get package path
    pkg_share = FindPackageShare('so_101_arm')
    
    # Controller configuration path (for xacro parameter)
    controller_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'controllers_6dof.yaml'
    ])
    
    # XACRO: Get URDF content using ParameterValue with xacro processing
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                pkg_share,
                'urdf',
                'so_101_arm_6dof.urdf.xacro'
            ]),
            ' controller_config_file:=',
            controller_config
        ]),
        value_type=str
    )

    return LaunchDescription([
        # Robot State Publisher with XACRO processing
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
