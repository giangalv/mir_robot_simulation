"""
@file mir_nav_launch.py
@brief Main navigation launch file for MiR robot including AMCL and navigation stack.

This launch file combines:
- MiR driver and visualization
- AMCL localization
- Navigation stack (planner, controller, behaviors)

@section dependencies Dependencies
- mir_driver: For MiR robot interface
- mir_navigation: For navigation configurations

@section parameters Launch Parameters
- namespace (string, default=""): Top-level namespace
- use_sim_time (bool, default=false): Use simulation clock if true
- map (string): Path to map file (relative to mir_navigation/maps or absolute path)
- rviz_config_file (string): Path to RViz configuration file
- slam_params_file (string): Path to SLAM configuration file

@section included_launches Included Launch Files
- mir_launch.py: Driver and visualization
- amcl.py: AMCL localization
- navigation.py: Navigation stack

@section usage Usage
ros2 launch mir_navigation mir_nav_launch.py map:=path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    """Generate launch description for MiR navigation system."""
    # Get package directories
    mir_driver_dir = get_package_share_directory('mir_driver')
    mir_nav_dir = get_package_share_directory('mir_navigation')
    mir_manual_nav_dir = get_package_share_directory('mir_manual_navigation')

    # Launch configurations
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=PathJoinSubstitution([mir_nav_dir, 'rviz', 'mir_nav.rviz'])
    )

    def resolve_map_path(context):
        """Resolve map file path from relative or absolute input."""
        map_arg = context.launch_configurations.get('map', '')
        if not map_arg:
            return [SetLaunchConfiguration('map_file', '')]
            
        # Check relative path first
        rel_path = os.path.join(mir_nav_dir, 'maps', map_arg)
        if os.path.isfile(rel_path):
            return [SetLaunchConfiguration('map_file', rel_path)]
        # Check absolute path
        elif os.path.isfile(map_arg):
            return [SetLaunchConfiguration('map_file', map_arg)]
        # Fallback to empty if not found
        return [SetLaunchConfiguration('map_file', '')]

    # Declare launch arguments
    declare_arguments = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace for all nodes and topics'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            description='Relative path to map in mir_navigation/maps or absolute path to map.yaml'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([mir_nav_dir, 'rviz', 'mir_nav.rviz']),
            description='Full path to RViz configuration file'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([mir_nav_dir, 'config', 'mir_mapping_async.yaml']),
            description='Full path to SLAM parameters file'
        )
    ]

    # Include other launch files
    included_launches = [
        # Main driver and visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_driver_dir, 'launch', 'mir_launch.py')
            ),
            launch_arguments={
                'rviz_config_file': rviz_config_file,
                'namespace': namespace,
                'use_sim_time': use_sim_time
            }.items()
        ),
        # AMCL localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_nav_dir, 'launch', 'include', 'amcl.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map_file'),
                'use_sim_time': use_sim_time
            }.items(),
        ),
        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_nav_dir, 'launch', 'include', 'navigation.py')
            ),
            launch_arguments={
                'map_subscribe_transient_local': 'true',
                'use_sim_time': use_sim_time
            }.items()
        ),
        # Manual control 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_manual_nav_dir, 'launch', 'manual_control_launch.py')
            )
        )
    ]

    return LaunchDescription([
        *declare_arguments,
        OpaqueFunction(function=resolve_map_path),

        *included_launches
    ])