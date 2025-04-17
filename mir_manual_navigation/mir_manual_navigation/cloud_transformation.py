import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')

        qos = QoSProfile(
            depth=5,  # Keep only the latest message
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Suitable for sensor data
            durability=QoSDurabilityPolicy.VOLATILE  # Don't store old messages
        )

        self.latest_left = None  # Store full message
        self.latest_right = None  # Store full message

        self.subscription_left = self.create_subscription(
            PointCloud2, '/camera_floor_left/obstacles', self.left_callback, qos)
        
        self.subscription_right = self.create_subscription(
            PointCloud2, '/camera_floor_right/obstacles', self.right_callback, qos)

        self.publisher = self.create_publisher(PointCloud2, '/camera_floor/obstacles', 20)

        self.timer = self.create_timer(0.1, self.publish_latest)  # Publish at ~10 Hz

    def left_callback(self, msg: PointCloud2):
        self.latest_left = msg  # Store full message

    def right_callback(self, msg: PointCloud2):
        self.latest_right = msg  # Store full message

    def publish_latest(self):
        if self.latest_left is None or self.latest_right is None:
            return  # Wait until both messages are received

        # Extract points from messages
        left_points = np.array([
            [p[0], p[1], p[2]] for p in pc2.read_points(self.latest_left, field_names=("x", "y", "z"), skip_nans=True)
        ])
        right_points = np.array([
            [p[0], p[1], p[2]] for p in pc2.read_points(self.latest_right, field_names=("x", "y", "z"), skip_nans=True)
        ])

        # Handle empty arrays
        if left_points.size == 0 and right_points.size == 0:
            return
        elif left_points.size == 0:
            # If only one camera found points, publish that
            combined_points = right_points
            self.publisher.publish(self.numpy_to_pointcloud2(combined_points, frame_id=self.latest_right.header.frame_id))
            return
        elif right_points.size == 0:
            # If only one camera found points, publish that
            combined_points = left_points
            self.publisher.publish(self.numpy_to_pointcloud2(combined_points, frame_id=self.latest_left.header.frame_id))
            return
        else:
            # Concatenate both point clouds if both exist
            combined_points = np.vstack((left_points, right_points))

        # Convert back to PointCloud2 message
        merged_msg = self.numpy_to_pointcloud2(combined_points, frame_id=self.latest_left.header.frame_id)
        self.publisher.publish(merged_msg)

    def numpy_to_pointcloud2(self, points, frame_id="map"): # "odom"
        """Converts a Nx3 numpy array to a ROS2 PointCloud2 message with proper timestamp."""
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()  # Use current time
        header.frame_id = frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        return pc2.create_cloud(header, fields, points)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    try:        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        print("POINTCLOUD_SUBSCRIBER shutdown complete.")

if __name__ == '__main__':
    main()
