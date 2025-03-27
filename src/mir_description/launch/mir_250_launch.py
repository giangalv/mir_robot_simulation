"""!
@file mir_250_launch.py
@brief Launch file for the MiR 250 robot description.

This launch file sets up the robot description for the MiR 250 robot,
including the robot state publisher and optionally the joint state publisher.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """!
    @brief Generate the launch description for the MiR 250 robot.
    @return LaunchDescription object containing all the nodes and parameters.
    """

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mir_description_dir = get_package_share_directory('mir_description')

    def create_robot_description(context):
        """!
        @brief Create the robot description from XACRO file.
        @param context The launch context.
        @return A list containing a SetLaunchConfiguration action with the robot description.
        """
        ns = context.launch_configurations['namespace']
        if ns.startswith('/'):
            ns = ns[1:]
        urdf_dir = os.path.join(mir_description_dir, 'urdf')
        xacro_file = os.path.join(urdf_dir, 'mir.urdf.xacro')
        doc = xacro.process_file(xacro_file, mappings={'tf_prefix': ns})
        robot_desc = doc.toprettyxml(indent='  ')
        return [SetLaunchConfiguration('robot_description', robot_desc)]

    return LaunchDescription([

      DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Robot prefix'),

      DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics to'),

      OpaqueFunction(function=create_robot_description),

      Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time,
                    'robot_description': LaunchConfiguration('robot_description')}],
        namespace=LaunchConfiguration('namespace'),
      ),

      Node(
          package='mir_manual_navigation',
          executable='encoder_to_joint_state',
          name='encoder_to_joint_state',
          output='screen',
      )
    ])