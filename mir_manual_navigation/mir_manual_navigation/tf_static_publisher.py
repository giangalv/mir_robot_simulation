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
        self.starter_tf = False
        self.cached_transforms = None

    def tf_static_callback(self, msg):
        if not self.starter_tf:
            self.cached_transforms = msg  # Store the static transforms
            self.starter_tf = True

    def publish_tf(self):
        if self.starter_tf:
            self.tf_pub.publish(self.cached_transforms)

def main():
    rclpy.init()
    node = TFStaticRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
