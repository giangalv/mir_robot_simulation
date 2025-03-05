from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            name='dual_laser_merger',
            output='screen',
        )
    ])
