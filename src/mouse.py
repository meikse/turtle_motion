#!/usr/bin/env python3

from pyautogui import position 

import rclpy
from geometry_msgs.msg import Point

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('mouse_locator')
    publisher = node.create_publisher(Point, 'location/mouse', 1)

    msg = Point()

    def timer_callback():
        try:
            x, y = position()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = 0.
            publisher.publish(msg)
            node.get_logger().info("x: {},y: {}".format(x,y))
        except:
            node.get_logger().info("mouse position not obtainable.")

    period = .1
    timer = node.create_timer(period, timer_callback)

    rclpy.spin(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
