import rclpy
from rclpy.node import Node
from mir_msgs.msg import RobotState
from geometry_msgs.msg import Twist

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.publisher_state = self.create_publisher(RobotState, '/robot_state_ordering', 10)
        self.robot_state_subscriber = self.create_subscription(
            RobotState, 
            '/robot_state', 
            self.robot_state_callback, 
            10
        )
        self.twistmux_subscriber = self.create_subscription(
            Twist, 
            '/cmd_vel_out', 
            self.twistmux_callback, 
            10
        )
        self.initial_flag = True
        self.publish_state = self.create_timer(20, self.publish_state)

    def publish_state(self):
        """
        Publishes a RobotState message to set the robot to manual control mode.
        
        This method creates a RobotState message with the robot_state field set to 11,
        which corresponds to ROBOT_STATE_MANUALCONTROL, and publishes it to the
        '/robot_state_ordering' topic. It also logs the action using ROS2 logging.
        
        Parameters:
            None
            
        Returns:
            None
        """
        msg = RobotState()
        msg.robot_state = 11  # ROBOT_STATE_MANUALCONTROL
        msg.robot_state_string = "MANUALCONTROL"
        self.publisher_state.publish(msg)
        self.get_logger().info('Published Robot State: MANUALCONTROL')
        self.get_logger().info('Published Robot State: MANUALCONTROL')
        # it doesn't WORK, it publish but the robot state doesn't change

    def robot_state_callback(self, msg):
        if self.initial_flag:
            self.publish_state()
            self.initial_flag = False
        self.get_logger().info('Received RobotState message: "%s"' % msg)

    def twistmux_callback(self, msg):
        self.get_logger().info('Received CMD_VEL message: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    node = RobotStatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
