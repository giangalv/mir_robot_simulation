"""
@file manual_control_launch.py
@brief Launch file for manual control of the MiR robot.

This launch file sets up the nodes required for manual control of the MiR robot,
including twist multiplexer, keyboard teleoperation, joystick control, and a custom manual mode node.

@section dependencies Dependencies
- twist_mux
- teleop_twist_keyboard
- joy
- teleop_twist_joy
- manual_navigation

@section parameters Launch Parameters
- namespace (string, default=""): Namespace for launched nodes.
- use_sim_time (bool, default=false): Use simulation time if true.

@section nodes Nodes
- twist_mux: Twist multiplexer node for prioritizing control sources.
- teleop_twist_keyboard: Node for keyboard teleoperation.
- joy_node: Node for reading joystick input.
- teleop_twist_joy: Node for converting joystick input to Twist messages.
- manual_mode: Custom node for manual navigation mode.

@section usage Usage
ros2 launch manual_navigation manual_control_launch.py [parameters]
"""

import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description for manual control setup.

    @return LaunchDescription object containing all nodes and parameters.
    """
    
    namespace = LaunchConfiguration('namespace')
    manual_navigation_path = get_package_share_directory('mir_manual_navigation')
    mir_driver_dir = get_package_share_directory('mir_driver')

    # Define paths to configuration files
    twist_config = os.path.join(
        manual_navigation_path,
        'config',
        'twist_mux.yaml'
    )
    joy_config = os.path.join(
        manual_navigation_path,
        'config',
        'joy_config.yaml'
    )
    joy_initialization = os.path.join(
        manual_navigation_path,
        'config',
        'joy_init.yaml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        # Declare namespace argument
        DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Namespace for launched nodes'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_driver_dir, 'launch', 'mir_launch.py'))
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

        # Custom node for manual navigation mode
        Node(
            package='mir_manual_navigation',
            executable='manual_mode',
            name='manual_mode',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
        ) 
    ])