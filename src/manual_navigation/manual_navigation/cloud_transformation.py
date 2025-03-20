import numpy as np
import struct
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

# Subscribe to the topic
import rclpy
from rclpy.node import Node

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2, '/camera_floor_left/obstacles', self.callback, 10)
        self.publishing_pointcloud = self.create_publisher(
            PointCloud2, '/obstacles/pointcloud', 10)
    
    def callback(self, msg: PointCloud2):
        # Convert binary data to numpy array
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)

        # Extract x, y, z columns (first 3 values of each row)
        xyz_points = points[:, :3]

        # Convert NumPy array to PointCloud2 message
        pointcloud_msg = self.numpy_to_pointcloud2(xyz_points, frame_id=msg.header.frame_id)
        
        # Publish the point cloud
        self.publishing_pointcloud.publish(pointcloud_msg)
        
    def numpy_to_pointcloud2(self, points, frame_id="map"):
        """
        Converts a Nx3 numpy array to a ROS2 PointCloud2 message.
        """
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Create PointCloud2 message
        return pc2.create_cloud(header, fields, points)
        
def main(args=None):
    """
    Initializes the ROS 2 Python client library, creates a PointCloudSubscriber node,
    and starts spinning to process incoming messages.
    """
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
