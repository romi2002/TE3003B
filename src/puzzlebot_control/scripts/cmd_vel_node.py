#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class CmdVelNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_node')
        self.wl_pub = self.create_publisher(Float32, 'VelocitySetL', 1)
        self.wr_pub = self.create_publisher(Float32, 'VelocitySetR', 1)

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 1
        )

        self.r = 0.05 # Wheel radius
        self.l = 0.19 # Wheel base

    def cmd_vel_cb(self, msg):

        u = msg.linear.x
        omega = msg.angular.z

        wl = (u - (self.l * omega) / 2.0) * (1.0 / self.r)
        wr = (u + (self.l * omega) / 2.0) * (1.0 / self.r)

        min_wheel = 0.1
        max_wheel = 3.0
        if math.fabs(wl) > max_wheel:
            wl = math.copysign(max_wheel, wl)
        elif math.fabs(wl) < min_wheel:
            wl = 0.0

        if math.fabs(wr) > max_wheel:
            wr = math.copysign(max_wheel, wr)
        elif math.fabs(wr) < min_wheel:
            wr = 0.0

        wl_msg = Float32()
        wl_msg.data = wl
        self.wl_pub.publish(wl_msg)

        wr_msg = Float32()
        wr_msg.data = wr
        self.wr_pub.publish(wr_msg)

        self.get_logger().info(f"{wl} {wr}")


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_node = CmdVelNode()
    rclpy.spin(cmd_vel_node)

    cmd_vel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()