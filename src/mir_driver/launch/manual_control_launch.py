from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    namespace = LaunchConfiguration('namespace')
    default_config_topics = os.path.join(
        get_package_share_directory('mir_driver'),
        'config',
        'twist_mux.yaml'
    )
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
                default_config_topics,
                {'use_sim_time': use_sim_time}
            ],
        ),

        # Start joystick node for controlling the robot via joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                default_config_topics,
                {'use_sim_time': use_sim_time}
            ],
        ),

        # Node for converting joy messages to Twist messages
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[
                default_config_topics,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel_joy'),
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
                ('cmd_vel', 'cmd_vel_key'),
            ],
        ),
    ])