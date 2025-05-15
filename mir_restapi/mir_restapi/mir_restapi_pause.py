#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import mir_restapi.mir_restapi_lib

class MirPauseController(Node):
    """ROS2 node for controlling MiR robot's pause state via REST API"""
    
    def __init__(self):
        super().__init__('mir_pause_controller')
        
        # Parameters
        self.declare_parameter('mir_hostname', "")
        self.hostname = self.get_parameter('mir_hostname').get_parameter_value().string_value
        self.declare_parameter('mir_restapi_auth', "")
        self.auth = self.get_parameter('mir_restapi_auth').get_parameter_value().string_value
        
        self.api_handle = None
        self.setup_api_handle()
        
        # Service
        self.create_service(Trigger, 'mir_set_pause_control', self.pause_control_callback)
        #self.get_logger().info("Pause control service ready")

    def setup_api_handle(self):
        """Initialize REST API connection if credentials are available"""
        if self.hostname and self.auth:
            self.api_handle = mir_restapi.mir_restapi_lib.MirRestAPI(
                self.get_logger(), 
                self.hostname, 
                self.auth
            )
            self.get_logger().info("REST API connection established")

    def pause_control_callback(self, request, response):
        """Service callback to pause the MiR robot"""
        if not self.api_handle:
            response.success = False
            response.message = "API credentials not configured"
            self.get_logger().error(response.message)
            return response
            
        try:
            result = self.api_handle.set_pause_control()
            response.success = True
            response.message = f"Pause command sent: {result}"
            self.get_logger().info("Robot paused successfully")
        except Exception as e:
            response.success = False
            response.message = f"Pause failed: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MirPauseController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()