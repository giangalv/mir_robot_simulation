"""
@file mir_launch.py
@brief Launch file for the MiR robot driver and additional components.

This launch file sets up the MiR robot driver and includes additional launch files
for a complete MiR robot setup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Generate the launch description for the MiR robot.

    This function creates a LaunchDescription that includes the main MiR robot
    driver launch file (mir_headless_launch.py) and an additional launch file
    for extra components (additions_launch.py).

    @return LaunchDescription: The complete launch description for the MiR robot setup.
    """

    # Get the directory where the mir_driver package is installed
    mir_driver = get_package_share_directory('mir_driver')

    return LaunchDescription([

      # Include the main MiR robot driver launch file
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(mir_driver, 'launch', 'mir_headless_launch.py')),
      ),

      # Include additional components launch file
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(mir_driver, 'launch', 'additions_launch.py')),
      )

    ])
