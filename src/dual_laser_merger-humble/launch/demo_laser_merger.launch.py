from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            name='dual_laser_merger',
            output='screen',
            parameters=[{
                'laser_1_topic': '/f_scan', # Input topic name for laser 1
                'laser_2_topic': '/b_scan', # Input topic name for laser 2
                'merged_topic': '/mir_scan', # Output topic name for merged laser scan
                'target_frame': 'base_link', # Target frame
                'laser_1_x_offset': 0.0, 
                'laser_1_y_offset': 0.0,
                'laser_1_yaw_offset': 0.0,
                'laser_2_x_offset': -0.04,
                'laser_2_y_offset': 0.0,
                'laser_2_yaw_offset': 0.0,
                'tolerance': 0.01, 
                'queue_size': 5,
                'angle_increment': 0.0029088794253766537, # Angular distance between measurements [rad] of merged laser scan
                'scan_time': 0.000014, # Time between measurements [s] of merged laser scan
                'range_min': 0.01, # Minimum range to consider for merging [m]
                'range_max': 30.0, # Maximum range to consider for merging [m]
                'min_height': -1.0,  # Minimum height to consider for merging [m] (negative value means no height constraint)
                'max_height': 1.0, # Maximum height to consider for merging [m] (negative value means no height constraint)
                'angle_min': -3.141592654, # Minimum angle for considering in the merged laser scan [rad]
                'angle_max': 3.141592654, # Maximum angle for considering in the merged laser scan [rad]
                'inf_epsilon': 1.0,
                'use_inf': False, # If true reports infinite values as [+inf], else reported as [range_max+1]
                'allowed_radius': 0.45,
                'enable_shadow_filter': False,
                'enable_average_filter': False,
            }],
        )
    ])