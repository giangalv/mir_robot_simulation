import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    mir_description_dir = get_package_share_directory('mir_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_description_dir, 'launch', 'mir_250_launch.py')),
            launch_arguments={
                'joint_state_publisher_enabled': 'false',
                'namespace': LaunchConfiguration('namespace')
            }.items(),
            condition=IfCondition(LaunchConfiguration(
                'robot_state_publisher_enabled'))
        ),

        Node(
            package='mir_driver',
            executable='mir_bridge',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'tf_prefix': LaunchConfiguration('namespace')}],
            namespace=LaunchConfiguration('namespace'),
            output='screen'),

        Node(
            package='mir_driver',
            executable='fake_mir_joint_publisher',
            remappings=[('use_sim_time', LaunchConfiguration('use_sim_time'))],
            parameters=[{'tf_prefix': LaunchConfiguration('namespace')}],
            namespace=LaunchConfiguration('namespace'),
            output='screen'),

        Node(
            package='twist_stamper',
            executable='twist_stamper',
            name='twist_stamper_cmd_vel_mir',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('cmd_vel_in', 'cmd_vel'),
                ('cmd_vel_out', 'cmd_vel_stamped'),
            ],
            namespace=LaunchConfiguration('namespace'),
        ),
  
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scan_merger_dir, 'launch', 'demo_laser_merger.launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    ])
