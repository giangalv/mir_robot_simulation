"""
@file mir_launch.py
@brief Launch file for MiR robot with teleoperation and RViz2 visualization.

This launch file sets up the MiR robot's teleoperation and visualization components.
It includes the teleoperation keyboard controller and RViz2 for monitoring the robot state.

@section dependencies Dependencies
- teleop_twist_keyboard
- rviz2
- mir_description

@section parameters Launch Parameters
- namespace (string, default=""): Namespace to push all topics into.
- use_sim_time (bool, default=false): Use simulation time if true.
- rviz_config_file (string, default="mir_visu_full.rviz"): Path to the RViz configuration file.

@section nodes Nodes
- teleop_twist_keyboard: Node for controlling the MiR robot using keyboard commands.
- rviz2: Visualization tool for monitoring the MiR robot state.

@section usage Usage
ros2 launch mir_description mir_launch.py [parameters]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import FrontendLaunchDescriptionSource


def generate_launch_description():
    """
    Generate a ROS2 launch description.

    This function sets up the launch description for running RViz2 and Foxglove bridge for a MiR robot.
    It declares necessary launch arguments and starts the required nodes.

    @return LaunchDescription: The launch description containing the defined nodes and arguments.
    """
    
    # Get the package share directory for mir_description
    mir_description_dir = get_package_share_directory('mir_description')
    
    # Define launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    foxglove_launch = os.path.join(get_package_share_directory('foxglove_bridge'), 
                                  'launch', 'foxglove_bridge_launch.xml')
    
    return LaunchDescription([
        
        
        # Declare RViz configuration file argument
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(mir_description_dir, 'rviz', 'mir_visu_full.rviz'),
            description='Define RViz config file to be used'
        ),
        
        # Start RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_file]
        ),

        #Foxglove for visualization 
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_launch)
        ),
    ])