#!/usr/bin/env python3

import os
import rclpy
import requests
import http.client
import matplotlib


from rclpy.node import Node
from mir_restapi.mir_restapi_lib import MirRestAPI
matplotlib.use('TkAgg')  

class MiRControllerRestAPI(Node):
    def __init__(self):
        super().__init__('mir_control_node')
        self._shutdown_flag = False
        self._pid = os.getpid()
        self._api_initialized = False
        
        # Parameters
        self.declare_parameter('mir_hostname', "130.251.13.90")
        self.declare_parameter('mir_restapi_auth', "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==")

        self.mir_hostname = self.get_parameter('mir_hostname').value
        self.mir_restapi_auth = self.get_parameter('mir_restapi_auth').value

        # Initialize REST API connection
        self.initialize_api()
        self.get_logger().info(f"\nStarting MiR Controller REST API with hostname: {self.mir_hostname}")
       
        self.api.set_ready_control()
        self.get_logger().info("Robot set to READY mode")

        # Initial robot info and footprint image
        self.update_robot_status(initial=True)

        # Periodic status updates every 2 minutes
        self._status_timer = self.create_timer(120.0, self.update_robot_status)


    def initialize_api(self):
        try:
            self.api = MirRestAPI(self.get_logger(), self.mir_hostname, self.mir_restapi_auth)
            self._api_initialized = True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize REST API: {str(e)}")
            self._api_initialized = False

    def update_robot_status(self, initial=False):
        if not self._api_initialized:
            self.get_logger().warn("REST API not initialized, attempting to reconnect...")
            self.initialize_api()
            if not self._api_initialized:
                return

        try:
            status = self.api.get_status()
        except (http.client.HTTPException, requests.exceptions.RequestException, ConnectionError, BrokenPipeError) as e:
            self._api_initialized = False
            self.initialize_api()
            if not self._api_initialized:
                return
            try:
                status = self.api.get_status()
            except Exception as e:
                self.get_logger().error(f"Failed again after reconnecting: {e}")
                return
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")
            return

        if initial:
            self.log_full_status(status)
            if 'footprint' in status:
                self.create_footprint_image(status.get('footprint'))
        else:
            self.log_velocity_and_battery(status)
    
    def log_velocity_and_battery(self, status_data):
        vel = status_data.get('velocity', {})
        battery = status_data.get('battery_percentage', 0.0)
        time_left = status_data.get('battery_time_remaining', 0)
        hours = time_left // 3600
        minutes = (time_left % 3600) // 60

        log_msg = (
            f"\nVELOCITY: Linear={vel.get('linear', 0.0):.2f} m/s, "
            f"Angular={vel.get('angular', 0.0):.2f} rad/s\n"
            f"BATTERY: {battery:.1f}% ({hours}h {minutes}m remaining)\n"
        )
        self.get_logger().info(log_msg)

    def create_footprint_image(self, footprint_coords):
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
            
            if isinstance(footprint_coords, str):
                try:
                    import ast
                    footprint_coords = ast.literal_eval(footprint_coords)
                except Exception as e:
                    self.get_logger().warn(f"Coordinate parse error: {e}")
                    return

            if not footprint_coords or not all(len(p)==2 for p in footprint_coords):
                self.get_logger().warn("Invalid coordinates format")
                return

            fig, ax = plt.subplots(figsize=(12, 12))
            ax.set_aspect('equal')
            ax.set_title('MiR250 FOOTPRINT\nClose the window to control the Battery status')
            ax.set_xlim(-1.0, 1.0)
            ax.set_ylim(-1.0, 1.0)
            
            robot_width = 0.8
            robot_height = 0.58
            robot_corners = [
                (-robot_width/2, -robot_height/2), 
                (-robot_width/2, robot_height/2),   
                (robot_width/2, robot_height/2),    
                (robot_width/2, -robot_height/2)    
            ]
            
            robot = patches.Rectangle(
                robot_corners[0], robot_width, robot_height,
                fill=True, color='blue', alpha=0.3,
                label=f'Robot Body ({robot_width*1000:.0f}×{robot_height*1000:.0f}mm)'
            )
            ax.add_patch(robot)
            
            robot_labels = ['BR', 'BL', 'FL', 'FR']
            for (x,y), label in zip(robot_corners, robot_labels):
                ax.text(x, y, f' {label}\n({x:.2f},{y:.2f})', 
                    color='blue', fontsize=10,
                    ha='center', va='center',
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='blue'))

            poly = patches.Polygon(
                footprint_coords,
                closed=True,
                fill=False,
                edgecolor='red',
                linewidth=3,
                label='Safety Footprint'
            )
            ax.add_patch(poly)

            for i, (x,y) in enumerate(footprint_coords):
                ax.plot(x, y, 'ro', markersize=8)
                ax.text(x, y, f'({x:.2f},{y:.2f})',
                    color='red', fontsize=10,
                    ha='center', va='center',
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='red'))

            ax.arrow(0, 0, 0.25, 0, head_width=0.05, color='green', label='Front')
            ax.grid(True, linestyle=':')
            ax.axhline(0, color='black', linewidth=0.5)
            ax.axvline(0, color='black', linewidth=0.5)
            ax.set_xlabel('X (meters)')
            ax.set_ylabel('Y (meters)')
            ax.legend(loc='upper right', fontsize=10)

            try:
                plt.tight_layout()
                plt.show(block=True)
                plt.pause(0.001)
                self.get_logger().info("\nMIR 250 visualization displayed.\nControlling the BATTERY status every 2 minutes.")
            except Exception as e:
                self.get_logger().info(f"Visualization error or exit: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Visualization error: {e}")
        finally:
            plt.close('all')

    def log_full_status(self, status_data):
        if not status_data:
            self.get_logger().warn("No status data available")
            return

        output = [
            "\n\n===== ROBOT STATUS =====",
            f"Robot Name: {status_data.get('robot_name', 'Unknown')}",
            f"Model: {status_data.get('robot_model', 'Unknown')}",
            f"Serial: {status_data.get('serial_number', 'Unknown')}",
        ]

        vel = status_data.get('velocity', {})
        output.append(f"Velocity: Linear={vel.get('linear', 0.0):.2f} m/s, Angular={vel.get('angular', 0.0):.2f} rad/s")

        battery = status_data.get('battery_percentage', 0.0)
        time_left = status_data.get('battery_time_remaining', 0)
        hours = time_left // 3600
        minutes = (time_left % 3600) // 60
        output.append(f"Battery: {battery:.1f}% ({hours}h {minutes}m remaining)")
        output.append("===========================\n")

        self.get_logger().info('\n'.join(output))



def main():
    rclpy.init()
    node = MiRControllerRestAPI()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Fatal error: {str(e)}")
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
