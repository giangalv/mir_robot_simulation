"""!
@file mir_display_launch.py
@brief Launch file for displaying the MiR robot in RViz. Useful for debugging and testing

This launch file sets up the robot description and launches RViz for visualizing the MiR robot.
It includes the mir_250_launch.py and adds an RViz node with a specific configuration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """!
    @brief Generate the launch description for displaying the MiR robot in RViz.
    @return LaunchDescription object containing all the nodes and parameters.
    """

    mir_description_dir = get_package_share_directory('mir_description')
    rviz_config_file = os.path.join(
    mir_description_dir, 'rviz', 'mir_description.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    return LaunchDescription([

        DeclareLaunchArgument(
            'joint_state_publisher_enabled',
            default_value='true',
            description='Enable to publish joint states using joint state publisher'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_description_dir, 'launch', 'mir_250_launch.py')),
            launch_arguments={
                'joint_state_publisher_enabled':
                LaunchConfiguration('joint_state_publisher_enabled'),
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ])