"""
This module contains the launch file for the dual laser merger demo.

It sets up and launches the dual_laser_merger node with specific parameters
for merging data from two laser scanners.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate a launch description for the dual laser merger demo.

    This function creates a LaunchDescription that includes a single Node
    for the dual_laser_merger. The Node is configured with various parameters
    to control the behavior of the laser merger.

    Returns:
        LaunchDescription: A LaunchDescription object containing the configured Node.
    """
    return LaunchDescription([
        Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            name='dual_laser_merger',
            output='screen',
        )
    ])