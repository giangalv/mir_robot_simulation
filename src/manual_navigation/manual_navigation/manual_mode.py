import rclpy
from rclpy.node import Node
from mir_msgs.msg import RobotState
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_msgs.msg import String
import subprocess
import time
import json

class RobotManualController(Node):
    def __init__(self):
        super().__init__('robot_manual_controller')

        # MiR REST API settings
        self.mir_hostname = "130.251.13.90"
        self.mir_restapi_auth = "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
        
        # Start the mir_restapi_server
        self.start_mir_restapi_server()

        # Create service client for /mir_set_manual_control
        self.manual_control_client = self.create_client(Trigger, '/mir_set_manual_control')
        
        # Create publisher for the token
        self.token_publisher = self.create_publisher(String, '/manual_control_token', 10)

        # Wait for the service to be available
        self.get_logger().info("Waiting for /mir_set_manual_control service...")
        while not self.manual_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for /mir_set_manual_control service...")

        # Subscribe to topics
        self.robot_state_subscriber = self.create_subscription(RobotState, '/robot_state', self.robot_state_callback, 10)
        self.twistmux_subscriber = self.create_subscription(Twist, '/cmd_vel_out', self.twistmux_callback, 10)

        # Request manual control
        self.call_set_manual_control()

    def start_mir_restapi_server(self):
        """Start the mir_restapi_server with the required parameters."""
        command = [
            "ros2", "run", "mir_restapi", "mir_restapi_server",
            "--ros-args",
            "-p", f"mir_hostname:={self.mir_hostname}",
            "-p", f"mir_restapi_auth:={self.mir_restapi_auth}"
        ]
        self.get_logger().info("Starting mir_restapi_server...")
        self.restapi_process = subprocess.Popen(command)

        # Give time for the server to start
        time.sleep(5)

    def call_set_manual_control(self):
        """Call the /mir_set_manual_control service asynchronously to get the token."""
        request = Trigger.Request()
        future = self.manual_control_client.call_async(request)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self, future):
        if future.result() is not None:
            response = future.result()
            self.token = response.message  # Save the token
            self.get_logger().info(f"Received token: {self.token}")
            token_msg = String()
            token_msg.data = self.token
            self.token_publisher.publish(token_msg)
        else:
            self.get_logger().error("Failed to call /mir_set_manual_control")
            pass

    def robot_state_callback(self, msg):
        self.get_logger().info(f'Received RobotState message: {msg}')

    def twistmux_callback(self, msg):
        #self.get_logger().info(f'Received CMD_VEL message: {msg}')
        pass

    def destroy_node(self):
        """Ensure subprocess is killed before shutting down the node."""
        if hasattr(self, 'restapi_process'):
            self.get_logger().info("Shutting down mir_restapi_server...")
            self.restapi_process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotManualController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
