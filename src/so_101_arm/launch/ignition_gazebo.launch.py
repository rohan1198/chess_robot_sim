from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
# Import the missing function
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package path
    pkg_share = FindPackageShare('so_101_arm')
    
    # Get the actual install path for models, ensuring it's an absolute path string
    models_path_sub = PathJoinSubstitution([pkg_share, 'models'])
    # For environment variables, we need a string. We can use a LaunchConfiguration
    # or try to resolve it if context is available, but for direct use in ExecuteProcess,
    # it's often simpler to construct the string path if possible or use a helper.
    # Here, we'll assume the substitution will be resolved correctly by Launch.
    
    # Path to custom world file
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'chess_table_world.sdf'])
    
    # Generate robot description with correct .xacro file
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_share, 'urdf', 'so_101_arm_6dof.urdf.xacro']),
            ' controller_config_file:=',
            PathJoinSubstitution([pkg_share, 'config', 'controllers_6dof.yaml'])
        ]),
        value_type=str
    )
    
    # Controller configuration
    controller_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'controllers_6dof.yaml'
    ])
    
    # Robot State Publisher
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
    
    # Controller Manager with configuration
    # This node is responsible for loading and managing the hardware interfaces and controllers
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node', # This is the controller manager executable
        parameters=[
            {'robot_description': robot_description_content}, # Often needed by CM too
            controller_config, 
            {'use_sim_time': True}
        ],
        output='screen',
        # Ensure this starts after robot_state_publisher has published robot_description
        # This can be tricky with dependencies. For now, direct start.
    )
    
    # Launch Ignition Gazebo with custom world file
    # The -r flag starts Gazebo paused after loading the world.
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', world_file],
        output='screen',
        # additional_env will be handled by SetEnvironmentVariable actions for clarity
    )
    
    # Bridge for ROS-Gazebo communication
    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # TF (Gazebo -> ROS) - For Gazebo-published frames if any (e.g., world to model)
            # Note: robot_state_publisher handles robot's internal TFs based on joint_states
            '/model/so_101_arm/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V', # Example if Gazebo publishes base pose
            #'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V', # Generic TF bridge if needed
            #'/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
             ('/model/so_101_arm/pose', '/tf') # Remap specific model pose to /tf
        ],
        output='screen'
    )

    # Spawn robot in Ignition Gazebo - positioned on the chess table
    # This script will run after Gazebo has loaded the world (due to TimerAction)
    spawn_robot_timer = TimerAction(
        period=8.0,  # Wait for Gazebo to initialize and load the world
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

    # --- Controller Spawner Timers ---
    # These timers ensure that the spawners are called after the controller_manager is likely up
    # and the robot model has been spawned in Gazebo.

    # Joint state broadcaster spawner
    # Waits for controller_manager and robot to be ready
    joint_state_broadcaster_spawner_timer = TimerAction(
        period=15.0, # Increased delay to ensure CM is ready after robot spawn
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Arm controller spawner
    # Waits for joint_state_broadcaster to be active
    arm_controller_spawner_timer = TimerAction(
        period=18.0, # Start shortly after JSB
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )
    
    # Gripper controller spawner
    # Waits for arm_controller to be active
    gripper_controller_spawner_timer = TimerAction(
        period=21.0, # Start shortly after arm_controller
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Auto-start simulation AFTER controllers are expected to be active
    # The -r flag for ign gazebo starts it paused. This will unpause it.
    start_simulation_timer = TimerAction(
        period=24.0, # Unpause after all controllers are spawned
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
        # Set environment variables for Ignition Gazebo resource resolution
        # These should allow Gazebo to find models specified with model:// in SDFs/URDFs
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[PathJoinSubstitution([pkg_share, 'models']), os.pathsep, os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
        ),
        SetEnvironmentVariable( # GZ_SIM_RESOURCE_PATH is the newer variable for Ignition/Gazebo Sim
            name='GZ_SIM_RESOURCE_PATH', 
            value=[PathJoinSubstitution([pkg_share, 'models']), os.pathsep, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),
        # For gz_ros2_control system plugin if not found by default
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH', # For Gazebo Sim (Ignition)
            value=[os.path.join(get_package_share_directory('gz_ros2_control'), 'lib'), # Path to gz_ros2_control libs
                   os.pathsep,
                   os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]
        ),

        # Start core ROS nodes
        robot_state_publisher_node,
        controller_manager_node, # Start ControllerManager early
        
        # Start Gazebo
        gazebo_process,
        gazebo_bridge_node, # Start bridge early
        
        # Spawn robot into Gazebo
        spawn_robot_timer,
        
        # Spawn controllers
        joint_state_broadcaster_spawner_timer,
        arm_controller_spawner_timer,
        gripper_controller_spawner_timer,
        
        # Unpause simulation after everything is set up
        start_simulation_timer,
    ])
