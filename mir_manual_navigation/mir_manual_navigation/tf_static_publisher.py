#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import subprocess  
from ament_index_python.packages import get_package_share_directory
import os


class TFStaticRepublisher(Node):
    def __init__(self):
        super().__init__('tf_static_republisher')
        
        # Use Transient Local durability QoS for tf_static
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static_starter', self.tf_static_callback, qos_profile)
        self.tf_pub = self.create_publisher(TFMessage, '/tf_static', qos_profile)
        
        self.cached_transforms = []
        self.add_virtual_laser_static_transform()
        self.timer = self.create_timer(0.1, self.publish_tf)  # 20 Hz (0.05)

    def add_virtual_laser_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        #transform.header.stamp = Time(sec=0, nanosec=0)
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'virtual_laser_link'

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.2

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.cached_transforms.append(transform)

    def tf_static_callback(self, msg):
        for transform in msg.transforms:
            # Exclude transforms involving 'map' or 'odom'
            if transform.header.frame_id in ['map', 'odom'] or transform.child_frame_id in ['map', 'odom']:
                continue  
            
            # Check if the transform is already stored
            if not any(t.child_frame_id == transform.child_frame_id and t.header.frame_id == transform.header.frame_id for t in self.cached_transforms):
                self.cached_transforms.append(transform)

    def publish_tf(self):
        if self.cached_transforms:
            self.tf_pub.publish(TFMessage(transforms=self.cached_transforms))
    
    def on_shutdown_callback(self):
        """Called when ROS2 is shutting down"""
        print("ROS is shutting down...")

        try:
            # Try both possible locations for the script
            script_locations = [
                os.path.join(get_package_share_directory('mir_manual_navigation'), 'mir_manual_navigation', 'pause_mode_setup.sh'),
                os.path.join(os.path.dirname(__file__), 'pause_mode_setup.sh')
            ]
            
            for script_path in script_locations:
                if os.path.exists(script_path):
                    print(f"Executing shutdown script: {script_path}")
                    subprocess.run([script_path], check=True)
                    print("Shutdown script executed successfully.")
                    return
                    
            print("Error: Could not find pause_mode_setup.sh in any of these locations:")
            for loc in script_locations:
                print(f" - {loc}")
                
        except subprocess.CalledProcessError as e:
            print(f"Script failed with return code {e.returncode}")
        except Exception as e:
            print(f"Error running shutdown script: {e}")

def main():
    rclpy.init()
    node = TFStaticRepublisher()
    try:     
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        node.on_shutdown_callback()
        rclpy.try_shutdown()



if __name__ == '__main__':
    main()
