#!/usr/bin/env python3

import rclpy
import math
import yaml
import os
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion

import numpy as np
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

class BugAlgorithm(Node):
    def __init__(self):
        super().__init__('bug_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.scan = None
        self.left_range, self.right_range, self.forward_range = None, None, None
        self.goal = (0.75, 0)
        
        self.k_u = 0.25
        self.k_r = 0.85

    def get_scan_within_range(self, scan, min_angle, max_angle):
        a1, a2 = self.wrap_angle(min_angle), self.wrap_angle(max_angle)
        min_angle, max_angle = min(a1, a2), max(a1, a2)
        
        ranges = []
        msg = self.scan
        for i, range in enumerate(scan.ranges):
            angle = self.wrap_angle(i * msg.angle_increment + msg.angle_min + np.pi)
            if min_angle < angle < max_angle:
                ranges.append(range)
        return ranges
        
    @staticmethod
    def wrap_angle(angle):
        angle = math.copysign(math.fmod(angle, 2 * math.pi), angle)
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def scan_callback(self, msg):
        self.scan = msg
        self.left_range = self.get_scan_within_range(self.scan, np.pi/4, 2*np.pi/3)
        self.right_range = self.get_scan_within_range(self.scan, -2*np.pi/3, -np.pi/4)
        self.fwd_range = self.get_scan_within_range(self.scan, -np.pi/4, np.pi/4)
        
    def angle_to_point(self, pose, point):
        x, y, t = pose
        px, py = point
        ex = px - x
        ey = py - y
        return self.wrap_angle(np.arctan2(ey, ex) - t)
    
    def dist_to_point(self, pose, point):
        x, y, _ = pose
        px, py = point
        return np.hypot(px - x, py - y)
        
    def cmd_vel_to_point(self, pose, point):
        d = self.dist_to_point(pose, point)
        a = self.angle_to_point(pose, point)
        return (d * np.cos(a) * self.k_u, a * self.k_r)
        
    def follow_wall(self):
        wall_tol = 0.4
        if min(self.fwd_range) > wall_tol:
            # Can move forward
            return 0.2, 0.0
        return 0.0, 0.2

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                rclpy.time.Time()
            )
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        except TransformException as ex:
            self.get_logger().error(
                f"Could not transform {ex}"
            )
            return
        if self.scan is None:
            return
            
        
                                                        
        obs_tol = 0.4
        pose = (t.transform.translation.x, t.transform.translation.y, yaw)
        # If path towards point is clear
        goal_range = self.get_scan_within_range(self.scan,
            self.angle_to_point(pose, self.goal) - np.pi / 8,
            self.angle_to_point(pose, self.goal) + np.pi / 8
            )
        vel = 0.0, 0.0
        print(f"Left: {min(self.left_range)} Fwd: {min(self.fwd_range)} Right: {min(self.right_range)} Goal: {min(goal_range)}")
        print(min(goal_range))
        
        if self.dist_to_point(pose, self.goal) < 0.1:
            vel = (0.0, 0.0)
            self.get_logger().info("Reached goal")
        elif min(goal_range) > obs_tol:
            # Path is clear
            vel = self.cmd_vel_to_point(pose, self.goal)
            self.get_logger().info("Moving towards goal")
        else:
            vel = self.follow_wall()
            self.get_logger().info("Following wall")
            
        cmd_vel = Twist()
        cmd_vel.linear.x, cmd_vel.angular.z = vel
        self.vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    bug_node = BugAlgorithm()
    rclpy.spin(bug_node)
    bug0_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
