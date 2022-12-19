#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class WindowLocator(Node): 

    def __init__(self):
        super().__init__('window_locator')

        self.declare_parameter('window_name', "TurtleSim") 

        self.publisher_ = self.create_publisher(Point, "location/window", 1) 
        self.timer = self.create_timer(1, self.callback)

        self.msg = Point()

    def callback(self):
        self.name = self.get_parameter(
                'window_name').get_parameter_value().string_value

        shell_out= subprocess.Popen(["xwininfo","-name", self.name], 
                                    stdout=subprocess.PIPE,
                                    stderr = subprocess.STDOUT)

        out = shell_out.stdout.read()   # get shell output
        out = out.decode()              # convert from byte to string
        out = out.splitlines()          # delimiter = "\n"

        # find position in list for abs windows location in shell output
        parsed_out = [out[i].find("Absolute") for i, _ in enumerate(out)]
        # receive index of finding
        index = [i for i, val in enumerate(parsed_out) if val != -1]
        # filter values of from list via index
        win_pos = [int(''.join(filter(str.isdigit, out[i]))) for i in index]
        # self.get_logger().info("{}".format(win_pos))
        # assign to ROS msg
        try:
            self.msg.x = float(win_pos[0])
            self.msg.y = float(win_pos[1])
            self.msg.z = 0.                     # empty
            self.publisher_.publish(self.msg)
            self.get_logger().info("x: {},y: {}".format(win_pos[0],win_pos[1]))
        except:
            self.get_logger().info("window position not obtainable.")


def main(args=None):
    rclpy.init(args=args)
    node = WindowLocator()
    rclpy.spin(node)
    node.destroy_node()
    # rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
