import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    namespace = LaunchConfiguration('namespace')
    mir_nav_dir = get_package_share_directory('manual_navigation')
    twist_config = os.path.join(
        get_package_share_directory('manual_navigation'),
        'config',
        'twist_mux.yaml'
    )
    joy_config = os.path.join(
        get_package_share_directory('manual_navigation'),
        'config',
        'joy_config.yaml'
    )
    joy_initialization = os.path.join(
        get_package_share_directory('manual_navigation'),
        'config',
        'joy_init.yaml'
    )
    manual_mode_path = os.path.join(mir_nav_dir, 'launch', 'include', 'manual_mode.py')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        # Declare namespace argument
        DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Namespace for launched nodes'
        ),

        # Twist multiplexer node with source prioritization
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                twist_config
            ],
        ),

        # Start teleoperation node for controlling the robot via keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            namespace=namespace,
            prefix='xterm -e',
            output='screen',
            remappings=[
                ('cmd_vel', 'cmd_vel_keyb')
            ],
        ),

        # Start joystick node for controlling the robot via joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                joy_initialization
            ],
        ),

        # Node for converting joy messages to Twist messages
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                joy_config
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel_joy')
            ],
        ),

        Node(
            package='manual_navigation',
            executable='manual_mode',
            name='manual_mode',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
        )
        
    ])