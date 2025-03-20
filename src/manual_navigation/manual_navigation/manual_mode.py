"""
@file manual_mode.py
@brief ROS2 node for manual control of the MiR robot.

This node initializes the necessary ROS2 components, starts the MiR REST API server,
and starts manual control using automatic mode on the MiR robot.
After the motion, the robot is set into the paused state.
"""

import rclpy
import signal
import subprocess
import time
from rclpy.node import Node
from std_srvs.srv import Trigger

class RobotManualController(Node):
    """
    @class RobotManualController
    @brief Node for managing manual control of the MiR robot.
    """

    def __init__(self):
        super().__init__('robot_manual_controller')
        
        self.mir_hostname = "130.251.13.90"
        self.mir_restapi_auth = "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
        
        self.start_mir_restapi_server()
        
        self.ready_manual_control = self.create_client(Trigger, '/mir_set_ready_control')
        self.pause_manual_control = self.create_client(Trigger, '/mir_set_pause_control')
        
        self.wait_for_service(self.ready_manual_control, '/mir_set_ready_control')
        self.wait_for_service(self.pause_manual_control, '/mir_set_pause_control')
        self.call_service(self.ready_manual_control, "Robot set to READY mode.")

    def wait_for_service(self, client, service_name):
        """Wait for a ROS2 service to be available."""
        self.get_logger().info(f"Waiting for {service_name} service...")
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Still waiting for {service_name} service...")
    
    def call_service(self, client, success_message):
        """Call a ROS2 service and log the response."""
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(success_message)
        else:
            self.get_logger().error("Service call failed.")
    
    def start_mir_restapi_server(self):
        """Start the MiR REST API server."""
        command = [
            "ros2", "run", "mir_restapi", "mir_restapi_server",
            "--ros-args",
            "-p", f"mir_hostname:={self.mir_hostname}",
            "-p", f"mir_restapi_auth:={self.mir_restapi_auth}"
        ]
        self.get_logger().info("Starting MiR REST API server...")
        self.restapi_process = subprocess.Popen(command)
        time.sleep(5)
    
    def shutdown(self):
        """Shutdown the node and set the robot to PAUSE mode."""
        self.call_service(self.pause_manual_control, "Robot set to PAUSE mode.")
        self.get_logger().info("Robot set to PAUSE mode.")
        self.get_logger().info("Shutting down node...")
        if hasattr(self, 'restapi_process'):
            self.restapi_process.terminate()

def main():
    """Main function to initialize and run the RobotManualController node."""
    rclpy.init()
    node = RobotManualController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()

    # Shutdown rclpy if the node is not ok
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
