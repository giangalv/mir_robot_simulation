import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import euler_from_quaternion


import yaml
import math
import networkx as nx
import os


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class GraphNavController(Node):
    def __init__(self):
        super().__init__('graph_nav_controller')
        
        # ===== Configuration =====
        self.PACKAGE_NAME = 'graph_based_navigation_system'
        self.GRAPH_NAME = 'navigation_graph_2'  # Base name without extension
        
        # ===== Path Setup =====
        self.graph_file = self.get_graph_path()
        self.load_graph()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(2.0, self.prompt_user)

    def get_graph_path(self):
        """Get graph file path with fallback to install directory"""
        # First try source directory
        src_dir = os.path.join(
            get_package_share_directory(self.PACKAGE_NAME),
            '../..', 'src', self.PACKAGE_NAME, 'config'
        )
        src_path = os.path.join(src_dir, f'{self.GRAPH_NAME}.yaml')
        
        if os.path.exists(src_path):
            return src_path
        
        # Fall back to install directory
        install_dir = os.path.join(
            get_package_share_directory(self.PACKAGE_NAME),
            'config'
        )
        install_path = os.path.join(install_dir, f'{self.GRAPH_NAME}.yaml')
        
        if os.path.exists(install_path):
            return install_path
        
        raise FileNotFoundError(
            f"Graph file {self.GRAPH_NAME}.yaml not found in either:\n"
            f"- {src_dir}\n"
            f"- {install_dir}"
        )

    def load_graph(self):
        with open(self.graph_file, 'r') as f:
            data = yaml.safe_load(f)

        # Convert node IDs from str to int
        self.nodes = {int(k): v for k, v in data['nodes'].items()}

        self.G = nx.Graph()
        for node_id, props in self.nodes.items():
            self.G.add_node(node_id, **props)

        for edge in data['edges']:
            weight = edge.get('weight', 1.0)  # Default weight to 1.0 if not specified
            self.G.add_edge(edge['from'], edge['to'], weight=weight)

        self.get_logger().info(f"Loaded navigation graph from: {self.graph_file}")

    def get_closest_node(self, current_x, current_y):
        min_dist = float('inf')
        closest = None
        for node_id, props in self.nodes.items():
            dist = euclidean_distance(current_x, current_y, props['x'], props['y'])
            if dist < min_dist:
                min_dist = dist
                closest = node_id
        return closest

    def prompt_user(self):
        self.timer.cancel()

        # TODO: Replace this with real robot pose later
        #current_x, current_y = 1.2, 1.1
        #start_node = self.get_closest_node(current_x, current_y)
        x, y = self.get_robot_position_xy()

        if x is not None and y is not None:
            self.get_logger().info(f"Robot position: x={x:.2f}, y={y:.2f}")
        if x is None:
            print("Could not get robot pose.")
            return

        start_node = self.get_closest_node(x, y)
        print(f"\n[INFO] Robot is currently closest to node {start_node}.")

        while True:
            try:
                goal_node = int(input("Enter the goal node ID to navigate to (or -1 to exit): "))
            except ValueError:
                print("[ERROR] Invalid input. Please enter a number.")
                continue

            if goal_node == -1:
                print("[INFO] Exiting navigation loop.")
                break

            if goal_node not in self.nodes:
                print(f"[ERROR] Node {goal_node} does not exist in the graph.")
                continue

            try:
                path = nx.shortest_path(self.G, source=start_node, target=goal_node, weight='weight')
            except nx.NetworkXNoPath:
                print(f"[ERROR] No path exists from node {start_node} to node {goal_node}.")
                continue

            print(f"[INFO] Planned path: {path}")
            self.execute_path(path)

            # After navigation, set new start_node to the goal
            start_node = goal_node

    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',  # target frame
                'base_link',  # source frame
                now
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ])
            return {
                'x': translation.x,
                'y': translation.y,
                'yaw': yaw
            }
        except LookupException as e:
            self.get_logger().warn(f"Could not get robot pose: {e}")
            return None
        
    def get_robot_position_xy(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',          # target frame
                'base_link',    # source frame
                now
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f"Failed to get robot position: {e}")
            return None, None


    def execute_path(self, path):
        for node_id in path[1:]:  # Skip the current node
            props = self.nodes[node_id]

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = props['x']
            goal.pose.position.y = props['y']
            goal.pose.orientation.z = math.sin(props['yaw'] / 2)
            goal.pose.orientation.w = math.cos(props['yaw'] / 2)

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal

            print(f"[NAV] Navigating to node {node_id}...")

            self.nav_client.wait_for_server()
            send_goal_future = self.nav_client.send_goal_async(nav_goal)
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                print(f"[ERROR] Goal to node {node_id} was rejected by the server.")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result().result

            if result.error_code == 0:  # SUCCEEDED
                print(f"[SUCCESS] Reached node {node_id} successfully!\n")
            else:
                print(f"[FAILURE] Failed to reach node {node_id}. Error code: {result.error_code}\n")
                break
        print(f"[INFO] Completed full path to goal node {path[-1]}!\n")



def main():
    rclpy.init()
    node = GraphNavController()
    rclpy.spin(node)
    rclpy.shutdown()
