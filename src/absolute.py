import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Object(Node):

    def __init__(self):
        super().__init__('motion_driver')
        self.publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 1)
        self.timer = self.create_timer(1, self.callback)
        self.msg = Twist()

    def callback(self):
        self.msg.linear.x = 8.;
        self.msg.linear.y = -8.;
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = Object()
    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
