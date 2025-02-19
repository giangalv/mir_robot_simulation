from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_scan_merger',
            executable='scans_merger_node',
            name='scans_merger',
            output='screen',
            remappings=[
                ('/f_scan', '/front/scan'),
                ('/b_scan', '/rear/scan')
            ],
            parameters=[
                {'active': True},
                {'publish_scan': True},
                {'publish_pcl2': False},
                {'ranges_num': 1000},
                {'min_scanner_range': 0.05},
                {'max_scanner_range': 10.0},
                {'min_x_range': -10.0},
                {'max_x_range': 10.0},
                {'min_y_range': -10.0},
                {'max_y_range': 10.0},
                {'fixed_frame_id': 'map'},
                {'target_frame_id': 'map'}
            ]
        )
    ])
