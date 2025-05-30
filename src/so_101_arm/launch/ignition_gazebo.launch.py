from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def get_robot_description(context, *args, **kwargs):
    pkg_share = FindPackageShare('so_101_arm').find('so_101_arm')
    urdf_path = os.path.join(pkg_share, 'urdf', 'so_101_arm_6dof.urdf')
    
    # Read URDF content
    with open(urdf_path, 'r') as file:
        urdf_content = file.read()
        
    return urdf_content

def generate_launch_description():
    
    # Set up model path for Gazebo
    pkg_share = FindPackageShare('so_101_arm').find('so_101_arm')
    model_path = os.path.join(pkg_share, 'models')
    
    # Set environment variable for Gazebo resource path
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        ign_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + model_path
    else:
        ign_resource_path = model_path

    def launch_setup(context, *args, **kwargs):
        robot_description_content = get_robot_description(context)
        
        return [
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
                output='screen',
                additional_env={'IGN_GAZEBO_RESOURCE_PATH': ign_resource_path}
            ),
            
            # Bridge for ROS-Gazebo communication
            TimerAction(
                period=2.0,
                actions=[
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
                    )
                ]
            ),

            # Spawn robot in Gazebo
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        name='spawn_so_101_arm',
                        arguments=[
                            '-file', PathJoinSubstitution([
                                FindPackageShare('so_101_arm'),
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
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                        output='screen'
                    )
                ]
            ),

            # Load joint trajectory controller
            TimerAction(
                period=7.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
                        output='screen'
                    )
                ]
            ),

            # Load gripper controller
            TimerAction(
                period=9.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
                        output='screen'
                    )
                ]
            ),
        ]

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
