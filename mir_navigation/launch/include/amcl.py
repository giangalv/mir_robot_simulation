## @file amcl.py
#  @brief Launch file for AMCL (Adaptive Monte Carlo Localization) in ROS2.
#  This file sets up the necessary nodes and configurations for AMCL.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

## @file amcl.py
#  @brief Launch file for AMCL (Adaptive Monte Carlo Localization) in ROS2.
#  This file sets up the necessary nodes and configurations for AMCL.
def generate_launch_description():
    # Get the launch directory
    mir_nav_dir = get_package_share_directory('mir_navigation')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    ## @brief Declare the namespace launch argument.
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    ## @brief Declare the map YAML file launch argument.
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(mir_nav_dir, 'maps', 'test_TER_1.yaml'),
        description='Full path to map yaml file to load'
    )

    ## @brief Declare the use_sim_time launch argument.
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    ## @brief Declare the parameters file launch argument.
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(mir_nav_dir, 'config', 'mir_nav_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    ## @brief Declare the autostart launch argument.
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    ## @brief Declare the use_composition launch argument.
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    ## @brief Declare the container name launch argument.
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of container that nodes will load in if use composition')

    ## @brief Declare the use_respawn launch argument.
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    ## @brief Declare the log level launch argument.
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    ## @brief Load nodes for AMCL and related components.
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    ## @brief Create the launch description and populate it with actions.
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localization nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
