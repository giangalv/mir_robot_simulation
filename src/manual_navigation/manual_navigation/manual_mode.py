"""
@file manual_mode.py
@brief ROS2 node for manual control of the MiR robot.

This node sets up manual control for the MiR robot by interfacing with the MiR REST API
and managing the manual control token. It subscribes to robot state messages and can
optionally subscribe to twist messages for velocity control.

@section dependencies Dependencies
- rclpy
- mir_msgs
- std_srvs
- subprocess
- time

@section topics Topics
- Subscribes to:
  - /robot_state (mir_msgs/RobotState): Robot state information
  - /cmd_vel_out (geometry_msgs/Twist): Velocity commands (optional, currently commented out)
- Publishes to:
  - /manual_control_token (std_msgs/String): Manual control token (optional, currently commented out)

@section services Services
- Calls:
  - /mir_set_manual_control (std_srvs/Trigger): Service to request manual control

@section usage Usage
ros2 run manual_navigation manual_mode
"""

import rclpy
from rclpy.node import Node
from mir_msgs.msg import RobotState
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_msgs.msg import String
import subprocess
import time

class RobotManualController(Node):
    """
    @class RobotManualController
    @brief Node for managing manual control of the MiR robot.

    This class initializes the necessary ROS2 components, starts the MiR REST API server,
    and manages the manual control token for the robot.
    """

    def __init__(self):
        """
        @brief Initialize the RobotManualController node.
        """
        super().__init__('robot_manual_controller')

        # MiR REST API settings
        self.mir_hostname = "130.251.13.90"
        self.mir_restapi_auth = "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
        
        # Start the mir_restapi_server
        self.start_mir_restapi_server()

        # Create service client for /mir_set_manual_control
        self.manual_control_client = self.create_client(Trigger, '/mir_set_manual_control')
        
        # Create publisher for the token
        #self.token_publisher = self.create_publisher(String, '/manual_control_token', 10)

        # Wait for the service to be available
        self.get_logger().info("Waiting for /mir_set_manual_control service...")
        while not self.manual_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for /mir_set_manual_control service...")

        # Subscribe to topics
        self.robot_state_subscriber = self.create_subscription(RobotState, '/robot_state', self.robot_state_callback, 10)
        #self.twistmux_subscriber = self.create_subscription(Twist, '/cmd_vel_out', self.twistmux_callback, 10)

        # Request manual control
        self.call_set_manual_control()

    def start_mir_restapi_server(self):
        """
        @brief Start the mir_restapi_server with the required parameters.
        """
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
        """
        @brief Call the /mir_set_manual_control service asynchronously to get the token.
        """
        request = Trigger.Request()
        future = self.manual_control_client.call_async(request)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self, future):
        """
        @brief Callback function for the /mir_set_manual_control service response.
        @param future The future object containing the service response.
        """
        if future.result() is not None:
            response = future.result()
            self.token = response.message  # Save the token
            self.get_logger().info(f"Received token: {self.token}")
            token_msg = String()
            token_msg.data = self.token
            #self.token_publisher.publish(token_msg)
            pass
        else:
            self.get_logger().error("Failed to call /mir_set_manual_control")
            pass

    def robot_state_callback(self, msg):
        """
        @brief Callback function for the /robot_state topic.
        @param msg The received RobotState message.
        """
        self.get_logger().info(f'Received RobotState message: {msg}')

    #def twistmux_callback(self, msg):
    #    """
    #    @brief Callback function for the /cmd_vel_out topic.
    #    @param msg The received Twist message.
    #    """
    #    self.get_logger().info(f'Received CMD_VEL message: {msg}')
    #    pass

    def destroy_node(self):
        """
        @brief Clean up resources before shutting down the node.
        """
        if hasattr(self, 'restapi_process'):
            self.get_logger().info("Shutting down mir_restapi_server...")
            self.restapi_process.terminate()
        super().destroy_node()


def main(args=None):
    """
    @brief Main function to initialize and run the RobotManualController node.
    @param args Command-line arguments (default: None).
    """
    rclpy.init(args=args)
    node = RobotManualController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()