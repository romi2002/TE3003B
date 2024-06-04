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

#Change the world obstacle if u want load walls

class Bug0Algorithm(Node):
    def __init__(self):
        super().__init__('bug0_node')
        #self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.target_sub = self.create_subscription(PoseStamped, 'target_pose', self.target_callback, 10)
        self.target_x, self.target_y = None, None
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/puzzlebot1/odometry',
            self.odom_callback,
            100)
            
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.num_readings = []
        self.tolerance = 0.05  # Tolerance to consider the target reached
        self.distance_obj = 0.2  # Increased tolerance to consider a near object
        self.forward_speed = 0.1  # Maximum speed of the robot
        self.turning_speed = 0.3  # Turning speed
        self.range_min = 0.5  # More than distance obj

        self.k_u = 0.25
        self.k_r = 0.9

        #self.current_x = 0.0
        #self.current_y = 0.0
        #self.current_yaw = 0.0
        self.distance = 0.0

        # Hardcoded for test
        # self.target = PoseStamped()
        # self.target.pose.position.x = 2.0
        # self.target.pose.position.y = 0.0
        
        self.state = 'GO_TARGET'

        self.timer = self.create_timer(0.1, self.timer_callback)

    def target_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

    def wrap_angle_controller(self, angle):
        angle = math.copysign(math.fmod(angle, 2 * math.pi), angle)
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def controller(self):
        msg = Twist()
        max_vel = 0.3
        max_ang = 1.3

        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        #self.get_logger().info(f'TARGETS: target_x={self.target_x}, target_y={self.target_y}')
        distance = math.hypot(error_x, error_y)
        angle = self.wrap_angle_controller(math.atan2(error_y, error_x) - self.current_yaw)

        u = distance * math.cos(angle) * self.k_u
        #u = np.clip(u, 0, max_vel)
        r = angle * self.k_r

        if distance < 0.05:
            u = 0.0
            r = 0.0

        u = min(u, max_vel)
        r = min(r, max_ang)

        msg.linear.x = u
        msg.angular.z = r
        
        self.get_logger().info(f'Publishing cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        self.vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.z = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
        
    @staticmethod
    def wrap_angle(angle):
        angle = math.copysign(math.fmod(angle, 2 * math.pi), angle)
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def get_scan_within_range(self, min_angle, max_angle):
        ranges = []
        msg = self.scan
        for i, range in enumerate(self.num_readings):
            angle = self.wrap_angle(i * msg.angle_increment + msg.angle_min + np.pi)
            if min_angle < angle < max_angle:
                ranges.append(range)
        return ranges

    def follow_wall(self):
        if not self.num_readings:
            return

        d = self.distance_obj

        
        # front_dist = min(min(self.get_scan_within_range(-np.pi, -np.pi/6)), min(self.get_scan_within_range(np.pi/6, np.pi)))
        # left_dist = min(self.get_scan_within_range(np.pi/6, np.pi/2))
        # right_dist = min(self.get_scan_within_range(-np.pi/2, -np.pi/6))

        front_dist = min(min(self.num_readings[0:49]), min(self.num_readings[1031:1080]))
        left_dist = min(self.num_readings[50:230])
        right_dist = min(self.num_readings[890:1030])

        self.get_logger().info(f'Distances - Left: {left_dist}, Front: {front_dist}, Right: {right_dist}')
        #return

        mike = 0.25
        msg = Twist()        
        if (left_dist < d and front_dist > d+mike and right_dist > d) or (left_dist < d and front_dist < d+mike and right_dist > d):
            msg.linear.x = 0.0
            msg.angular.z = -self.turning_speed  # turn right
            self.get_logger().info("Turning RIGHT")
        elif (left_dist > d and front_dist > d+mike and right_dist < d) or (left_dist > d and front_dist < d+mike and right_dist < d) or (left_dist < d and front_dist < d+mike and right_dist < d):
            msg.linear.x = 0.0
            msg.angular.z = self.turning_speed  # turn left
            self.get_logger().info("Turning LEFT")
        else:
            msg.linear.x = self.forward_speed  # move forward
            msg.angular.z = 0.0
            self.get_logger().info("Moving FORWARD")

        self.vel_pub.publish(msg)

    def scan_callback(self, msg):
        self.scan = msg
        self.num_readings = list(msg.ranges)
        for i in range(len(self.num_readings)):
            if math.isnan(self.num_readings[i]):
                self.num_readings[i] = msg.range_min - 0.0001
            elif math.isinf(self.num_readings[i]):
                self.num_readings[i] = msg.range_max + 0.0001

        #self.range_min = min(self.get_scan_within_range(-np.pi, np.pi))
        self.range_min = min(min(self.num_readings[0:230]), min(self.num_readings[890:1080]))

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        list_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.current_yaw = euler_from_quaternion(list_orientation)[2]

        self.get_logger().info(f'Odometry update: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw}')

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                rclpy.time.Time()
            )
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y
            q = t.transform.rotation
            roll, pitch, self.current_yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        except TransformException as ex:
            self.get_logger().error(
                f"Could not transform {ex}"
            )
            return
        
        if self.target_x is None or self.target_y is None:
            return
        # front_dist = min(self.get_scan_within_range(-np.pi/6, np.pi/6))
        # left_dist = min(self.get_scan_within_range(np.pi/6, np.pi*2/3))
        # right_dist = min(self.get_scan_within_range(-np.pi*2/3, -np.pi/6))
        
        # print(f"front: {front_dist} left: {left_dist} right: {right_dist}")
        # return
        ex = self.target_x - self.current_x
        ey = self.target_y - self.current_y
        self.distance = np.hypot(ex, ey)
        #self.follow_wall()
        #return

        if self.distance < self.tolerance:
            self.get_logger().info(f'Reached target: x={self.target_x}, y={self.target_y}')
            self.state = 'STOP'
            #self.stop()
        
        if self.state == 'GO_TARGET':
            if self.range_min < self.distance_obj and self.range_min < 0.22:
                self.state = 'FOLLOW_WALL'
                self.get_logger().info("Following wall")
            else:
                self.controller()
        elif self.state == 'FOLLOW_WALL':
            if self.range_min >= self.distance_obj+0.02:
                self.state = 'GO_TARGET'
                self.get_logger().info("Controlling towards target")
            else:
                self.follow_wall()
        elif self.state == 'STOP':
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    bug0_node = Bug0Algorithm()
    rclpy.spin(bug0_node)
    bug0_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
