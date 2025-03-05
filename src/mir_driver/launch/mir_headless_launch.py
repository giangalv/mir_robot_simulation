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
    scan_merger_dir = get_package_share_directory('dual_laser_merger')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace to push all topics into.'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=''),

        DeclareLaunchArgument(
            'mir_hostname',
            default_value='130.251.13.90',
            description=''),

        DeclareLaunchArgument(
             'disable_map',
             default_value='false',
             description='Disable the map topic and map -> odom_comb TF transform from the MiR'),

        DeclareLaunchArgument(
            'robot_state_publisher_enabled',
            default_value='true',
            description='Set to true to publish tf using mir_description'),

        # Include MiR description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_description_dir, 'launch', 'mir_250_launch.py')),
            launch_arguments={
                'joint_state_publisher_enabled': 'true', # was false
                'namespace': LaunchConfiguration('namespace')
            }.items(),
            condition=IfCondition(LaunchConfiguration(
                'robot_state_publisher_enabled'))
        ),

        # Mir bridge node
        Node(
            package='mir_driver',
            executable='mir_bridge',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'tf_prefix': LaunchConfiguration('namespace')}],
            namespace=LaunchConfiguration('namespace'),
            output='screen'),

        # Twist stamper node for cmd_vel topic from MiR (used for teleop)
        Node(
            package='twist_stamper',
            executable='twist_stamper',
            name='twist_stamper_cmd_vel_mir',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': ''}], #empty frame_id
                #'frame_id': '1955'}], #last update
            remappings=[
                ('cmd_vel_in', 'cmd_vel'),
                ('cmd_vel_out', 'cmd_vel_stamped'),],
            namespace=LaunchConfiguration('namespace'),
        ),

        # Include laser scan merger launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scan_merger_dir, 'launch', 'demo_laser_merger.launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    ])
