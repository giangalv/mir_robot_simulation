import rclpy
import signal
import subprocess
import os
import time
import psutil
from rclpy.node import Node
from std_srvs.srv import Trigger

class RobotManualController(Node):
    def __init__(self):
        super().__init__('robot_manual_controller')
        self._shutdown_flag = False
        self._pid = os.getpid()
        self.mir_hostname = "130.251.13.90"
        self.mir_restapi_auth = "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="

        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Start REST API server
        self.get_logger().info("Starting MiR REST API server...")
        self.restapi_process = subprocess.Popen(
            ["ros2", "run", "mir_restapi", "mir_restapi_server",
             "--ros-args",
             "-p", f"mir_hostname:={self.mir_hostname}",
             "-p", f"mir_restapi_auth:={self.mir_restapi_auth}"],
            preexec_fn=os.setsid
        )
        
        # Initialize service clients
        self.ready_client = self.create_client(Trigger, '/mir_set_ready_control')
        self.pause_client = self.create_client(Trigger, '/mir_set_pause_control')
        
        # Wait for services
        if not self.wait_for_services():
            self.emergency_shutdown()
            return
            
        # Set initial mode
        if not self.set_robot_mode('READY'):
            self.emergency_shutdown()
            return

    def wait_for_services(self, timeout=10.0):
        """Wait for required services to become available"""
        start = time.time()
        while time.time() - start < timeout and not self._shutdown_flag:
            ready = self.ready_client.wait_for_service(timeout_sec=1.0)
            pause = self.pause_client.wait_for_service(timeout_sec=1.0)
            if ready and pause:
                return True
            self.get_logger().warn("Waiting for services...", throttle_duration_sec=2.0)
        return False

    def set_robot_mode(self, mode):
        """Set robot mode with timeout"""
        client = self.ready_client if mode == 'READY' else self.pause_client
        try:
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() and future.result().success:
                self.get_logger().info(f"Robot set to {mode} mode")
                return True
        except Exception as e:
            self.get_logger().error(f"Error setting {mode} mode: {str(e)}")
        return False

    def signal_handler(self, sig, frame):
        """Handle shutdown signal with two distinct phases"""
        if self._shutdown_flag:
            return
        self._shutdown_flag = True
        
        # PHASE 1: Set PAUSE mode (blocking)
        self.get_logger().info("Setting PAUSE mode...")
        pause_success = False
        try:
            pause_success = self.set_robot_mode('PAUSE')
        except Exception as e:
            self.get_logger().error(f"Error during PAUSE: {str(e)}")
        
        # Small delay to ensure mode change propagates
        time.sleep(0.5)
        
        # PHASE 2: Forceful termination
        self.get_logger().info("Terminating processes...")
        self.terminate_processes()
        
        # Immediate exit
        os._exit(0 if pause_success else 1)

    def terminate_processes(self):
        """Forcefully terminate all related processes"""
        try:
            # Kill REST API server process group
            if hasattr(self, 'restapi_process'):
                os.killpg(os.getpgid(self.restapi_process.pid), signal.SIGKILL)
                
            # Kill any remaining child processes
            parent = psutil.Process(self._pid)
            for child in parent.children(recursive=True):
                try:
                    child.kill()
                except:
                    pass
        except Exception as e:
            self.get_logger().error(f"Termination error: {str(e)}")

    def emergency_shutdown(self):
        """Last resort shutdown"""
        if not self._shutdown_flag:
            self.signal_handler(signal.SIGTERM, None)

def main():
    rclpy.init()
    node = RobotManualController()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Fatal error: {str(e)}")
    finally:
        node.emergency_shutdown()

if __name__ == '__main__':
    main()