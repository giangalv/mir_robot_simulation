"""
@file navigation.py
@brief Launch file for navigation with the MiR robot.

This launch file sets up the navigation stack for the MiR robot.
It configures the necessary parameters and launches the required nodes for autonomous navigation.

@section dependencies Dependencies
- nav2_controller
- nav2_smoother
- nav2_planner
- nav2_behaviors
- nav2_bt_navigator
- nav2_waypoint_follower
- nav2_velocity_smoother
- nav2_lifecycle_manager

@section parameters Launch Parameters
- namespace (string, default=""): Top-level namespace.
- use_sim_time (bool, default=false): Use simulation (Gazebo) clock if true.
- params_file (string): Full path to the ROS2 parameters file.
- autostart (bool, default=true): Automatically startup the nav2 stack.
- use_composition (bool, default=false): Use composed bringup if True.
- container_name (string, default="nav2_container"): Name of container that nodes will load in if using composition.
- use_respawn (bool, default=false): Whether to respawn if a node crashes. Applied when composition is disabled.
- log_level (string, default="info"): Log level for nodes.
- cmd_vel_topic (string, default="cmd_vel_out"): Define cmd_vel topic.
- default_nav_to_pose_bt_xml (string): Path to the default navigation to pose behavior tree XML file.
- default_nav_through_pose_bt_xml (string): Path to the default navigation through poses behavior tree XML file.

@section nodes Nodes
- controller_server: Controller server for navigation.
- smoother_server: Path smoothing server.
- planner_server: Global path planner server.
- behavior_server: Navigation behavior server.
- bt_navigator: Behavior tree navigator.
- waypoint_follower: Waypoint following node.
- velocity_smoother: Velocity smoothing node.
- lifecycle_manager_navigation: Lifecycle manager for navigation nodes.

@section usage Usage
Include this launch file in a main launch file or run directly:
ros2 launch mir_navigation navigation.py params_file:=/path/to/nav_params.yaml
"""

# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    Generate launch description for navigation stack.

    @return LaunchDescription object containing all nodes and parameters for navigation.
    """
    # Get the launch directory
    mir_nav_dir = get_package_share_directory('mir_navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_pose_bt_xml = LaunchConfiguration('default_nav_through_pose_bt_xml')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    command_topic = LaunchConfiguration('cmd_vel_w_prefix')

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
        'autostart': autostart,
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_pose_bt_xml
    }

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(mir_nav_dir, 'config', 'mir_nav_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_cmd_vel_cmd = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel_out',
        description='Define cmd_vel topic')

    declare_bt_nav_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml', default_value='')

    declare_bt_nav_through_cmd = DeclareLaunchArgument(
        'default_nav_through_pose_bt_xml', default_value='')

    def add_prefix_to_cmd_vel(context):
        """
        Add namespace prefix to cmd_vel topic if namespace is provided.

        @param context: Launch context
        @return List containing SetLaunchConfiguration action
        """
        topic = context.launch_configurations['cmd_vel_topic']
        try:
            namespace = context.launch_configurations['namespace']
            topic = namespace + '/' + topic
        except KeyError:
            pass
        return [SetLaunchConfiguration('cmd_vel_w_prefix', topic)]

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', command_topic)]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', command_topic)]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', command_topic), ('cmd_vel_smoothed', command_topic)]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', command_topic)]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', command_topic), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_cmd_vel_cmd)
    ld.add_action(declare_bt_nav_cmd)
    ld.add_action(declare_bt_nav_through_cmd)
    ld.add_action(OpaqueFunction(function=add_prefix_to_cmd_vel))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
