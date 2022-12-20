#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class WindowLocator(Node): 

    def __init__(self):
        super().__init__('window_locator')

        self.declare_parameter('window_name', "TurtleSim") 

        self.loc_pub_ = self.create_publisher(Point, "window/location", 1) 
        self.dim_pub_ = self.create_publisher(Point, "window/dimension", 1) 

        self.timer = self.create_timer(1, self.callback)

        self.loc_msg = Point()
        self.dim_msg = Point()

    def callback(self):

        self.name = self.get_parameter(
                'window_name').get_parameter_value().string_value

        shell_out= subprocess.Popen(["xwininfo","-name", self.name], 
                                    stdout=subprocess.PIPE,
                                    stderr = subprocess.STDOUT)

        out = shell_out.stdout.read()   # get shell output
        out = out.decode()              # convert from byte to string
        out = out.splitlines()          # delimiter = "\n"

        # self.get_logger().info("{}".format(win_size))
        # assign to ROS msg
        try:
            win_pos = self.parse(out, "Absolute")
            win_size = [self.parse(out, "Width")[0], self.parse(out,"Height")[0]]

            # absolut position
            self.loc_msg.x = float(win_pos[0])
            self.loc_msg.y = float(win_pos[1])
            self.loc_msg.z = 0.                     # empty
            self.loc_pub_.publish(self.loc_msg)
            # dimensions
            self.dim_msg.x = float(win_size[0])
            self.dim_msg.y = float(win_size[1])
            self.dim_msg.z = 0.                     # empty
            self.dim_pub_.publish(self.dim_msg)

            self.get_logger().info(
                    "x: {},y: {}, width: {}, height: {}".format(
                        win_pos[0],win_pos[1],win_size[0], win_size[1]))
        except:
            self.get_logger().info("window infos not obtainable.")


    def parse(self, out, key):
        # find position in list for abs windows location in shell output
        parsed_out = [out[i].find(key) for i, _ in enumerate(out)]
        # receive index of finding
        index = [i for i, val in enumerate(parsed_out) if val != -1]
        # filter values of from list via index
        return [int(''.join(filter(str.isdigit, out[i]))) for i in index]

def main(args=None):
    rclpy.init(args=args)
    node = WindowLocator()
    rclpy.spin(node)
    node.destroy_node()
    # rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
