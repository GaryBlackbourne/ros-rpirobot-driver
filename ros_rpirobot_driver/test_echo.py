"""Test echo node for driver."""

from rclpy.node import Node
import rclpy
from robot_interfaces.msg import Distance, Velocity


class TestEcho(Node):
    """Test echo node"""

    def __init__(self):
        super().__init__("test_echo_node")
        self.sub_vel = self.create_subscription(
            Velocity,
            'velocity',
            self.sub_vel_callback,
            10)
        self.sub_dist = self.create_subscription(
            Distance,
            'distance',
            self.sub_dist_callback,
            10)

    def sub_vel_callback(self, msg):
        """Execute velocity echo callback"""
        self.get_logger().info(
            "Velocity right: %s left: %s" %
            (msg.right, msg.left)
            )

    def sub_dist_callback(self, msg):
        """Execute distance echo callback"""
        self.get_logger().info(
            "Distance right: %s left: %s forward: %s backward: %s" %
            (msg.right, msg.left, msg.forward, msg.backward)
            )


def main(args=None):
    """Start Control interface echo test node."""
    rclpy.init(args=args)

    test_echo = TestEcho()

    rclpy.spin(test_echo)


if __name__ == '__main__':
    main()
