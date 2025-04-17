import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

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
        
        self.timer = self.create_timer(0.05, self.publish_tf)  # 20 Hz
        self.cached_transforms = []

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

def main():
    rclpy.init()
    node = TFStaticRepublisher()
    try:     
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        print("TF_STATIC_REPUBLISHER shutdown complete.")


if __name__ == '__main__':
    main()
