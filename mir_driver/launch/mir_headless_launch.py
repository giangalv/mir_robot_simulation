"""!
@file mir_headless_launch.py
@brief Launch file for MiR robot without GUI components.

This launch file sets up the MiR robot's core functionalities without any graphical user interface components.
It includes the robot description, MiR bridge, twist stamper, and laser scan merger.

@section dependencies Dependencies
- mir_description
- mir_driver
- twist_stamper
- dual_laser_merger

@section parameters Launch Parameters
- namespace (string, default=""): Namespace to push all topics into.
- use_sim_time (bool, default=false): Use simulation time if true.
- mir_hostname (string, default="130.251.13.90"): Hostname or IP address of the MiR robot.
- disable_map (bool, default=false): Disable the map topic and map -> odom_comb TF transform from the MiR.
- robot_state_publisher_enabled (bool, default=true): Publish tf using mir_description if true.

@section nodes Nodes
- mir_bridge: Bridge node for MiR robot communication.
- twist_stamper: Stamps twist messages for cmd_vel.
- laser_merger: Merges laser scan data (included from separate launch file).

@section usage Usage
ros2 launch mir_driver mir_headless_launch.py [parameters]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """!
    Generates the launch description for the MiR robot headless setup.

    @return LaunchDescription object containing all nodes and parameters.
    """

    mir_description_dir = get_package_share_directory('mir_description')

    robot_displays_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_250_launch.py')
        )
    )

    use_sim_time_standard = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time') 

    name_standard = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics into.'
    )

    name_space = LaunchConfiguration('namespace')

    mir_launcher = Node(
        package='mir_driver',
        executable='mir_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tf_prefix': name_space
        }],
        namespace=LaunchConfiguration('namespace'),
        output='screen'
    )

    twister_launcher = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper_cmd_vel_mir',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': '' #empty frame_id
        }], 
        remappings=[
            ('cmd_vel_in', 'cmd_vel_out'),
            ('cmd_vel_out', 'cmd_vel_stamped'),],
        namespace=LaunchConfiguration('namespace'),
    )

    return LaunchDescription([
        use_sim_time_standard,
        name_standard,
        mir_launcher,
        twister_launcher,       
        robot_displays_launcher
    ])
