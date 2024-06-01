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
        self.k_r = 1.5

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

    def timer_cb(self):
        if self.target_x is None or self.target_y is None:
            self.get_logger().info("No target specified")
            return
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",
                "map",
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().error(
                f"Could not transform {ex}"
            )
            return

        roll, pitch, yaw = euler_from_quaternion(t.transform.rotation)

        error_x = self.target_x - t.transform.translation.x
        error_y = self.target_y - t.transform.translation.y

        goal_x = error_x * np.cos(yaw) - error_y * np.sin(yaw)
        goal_y = error_x * np.sin(yaw) + error_y * np.cos(yaw)

        fx = goal_x * self.goal_gain + self.obs_x * self.obstacle_gain
        fy = goal_y * self.goal_gain + self.obs_y * self.obstacle_gain

        mag = np.hypot(fx, fy)
        angle = np.arctan2(fy, fx)

        u = mag * np.cos(angle) * self.k_u
        r = -angle * self.k_r

        cmd_vel = Twist()
        cmd_vel.linear.x = u
        cmd_vel.angular.z = r
        self.cmd_vel_pub.publish(cmd_vel)