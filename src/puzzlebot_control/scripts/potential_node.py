#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import euler_from_quaternion
import numpy as np


class PotentialFieldNode(Node):
    def __init__(self):
        super().__init__('potential_field_node')

        self.max_u = 0.6
        self.max_r = 1.25
        self.max_range = 1
        self.goal_gain = 1
        self.obstacle_gain = 0.1
        self.k_u = 0.25
        self.k_r = 0.5

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.target_sub = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.target_cb,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.timer_cb)
        self.target_x, self.target_y = None, None
        self.obs_x, self.obs_y = 0, 0

    def target_cb(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

    def scan_cb(self, msg):
        fx, fy = 0, 0
        valid_ranges = 0
        for i, range in enumerate(msg.ranges):
            angle = i * msg.angle_increment + msg.angle_min + np.pi
            intensity = msg.intensities[i]

            if intensity == 0:
                continue

            range = np.clip(range, 0, self.max_range)
            norm_range = 1 - (range / self.max_range)
            fx -= norm_range * np.cos(angle)
            fy -= norm_range * np.sin(angle)
            valid_ranges += 1
        if valid_ranges != 0:
            self.obs_x = fx / valid_ranges
            self.obs_y = fy / valid_ranges
        else:
            self.obs_x, self.obs_y = 0, 0
            
    def __del__(self):
        self.get_logger().info("Stopping robot")
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)

    def timer_cb(self):
        if self.target_x is None or self.target_y is None:
            self.get_logger().info("No target specified")
            return
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().error(
                f"Could not transform {ex}"
            )
            return

        q = t.transform.rotation
        roll, pitch, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))

        error_x = self.target_x - t.transform.translation.x
        error_y = self.target_y - t.transform.translation.y

        goal_x = (error_x * np.cos(yaw) - error_y * np.sin(yaw))
        goal_y = (error_x * np.sin(yaw) + error_y * np.cos(yaw))

        self.obs_x, self.obs_y = 0, 0
        fx = goal_x * self.goal_gain + self.obs_x * self.obstacle_gain
        fy = goal_y * self.goal_gain + self.obs_y * self.obstacle_gain

        mag = np.hypot(fx, fy)
        angle = np.arctan2(fy, fx) - yaw

        u = mag * np.cos(angle) * self.k_u
        r = angle * self.k_r
        
        if np.hypot(error_x, error_y) < 0.1:
            u = 0.0
            r = 0.0

        cmd_vel = Twist()
        cmd_vel.linear.x = u
        cmd_vel.angular.z = r
        self.get_logger().info(f"tx: {t.transform.translation.x} ty: {t.transform.translation.y} gx: {goal_x}, gy: {goal_y} u: {u} r: {r}")
        self.cmd_vel_pub.publish(cmd_vel)
        
def main(args=None):
    rclpy.init(args=args)
    pf_node = PotentialFieldNode()
    rclpy.spin(pf_node)
    pf_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()