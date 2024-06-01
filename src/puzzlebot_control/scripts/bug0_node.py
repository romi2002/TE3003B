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

#Change the world obstacle if u want load walls

class Bug0Algorithm(Node):
    def __init__(self):
        super().__init__('bug0_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/puzzlebot1/odometry',
            self.odom_callback,
            100)
        
        self.scan_sub = self.create_subscription(LaserScan, '/lidar', self.scan_callback, qos_profile_sensor_data)

        self.declare_parameter('points_file', os.path.join(get_package_share_directory('puzzlebot_control'), 'config', 'point.yaml'))
        points_file = self.get_parameter('points_file').get_parameter_value().string_value

        self.get_logger().info(f'Loading points from file: {points_file}')

        try:
            with open(points_file, 'r') as file:
                self.points = yaml.safe_load(file)['points']
        except Exception as e:
            self.get_logger().error(f'Failed to load points from file: {e}')
            self.points = []

        self.num_readings = []
        self.tolerance = 0.05  # Tolerance to consider the target reached
        self.distance_obj = 0.2  # Increased tolerance to consider a near object
        self.forward_speed = 0.1  # Maximum speed of the robot
        self.turning_speed = 0.3  # Turning speed
        self.range_min = 0.5  # More than distance obj

        self.k_u = 0.25
        self.k_r = 0.85

        #self.current_x = 0.0
        #self.current_y = 0.0
        #self.current_yaw = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.distance = 0.0

        self.target = PoseStamped()
        self.target.pose.position.x = 2.5
        self.target.pose.position.y = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def controller(self):
        msg = Twist()
        max_vel = 0.2
        max_ang = 0.2

        error_x = self.target.pose.position.x - self.current_x
        error_y = self.target.pose.position.y - self.current_y
        #self.get_logger().info(f'TARGETS: target_x={self.target_x}, target_y={self.target_y}')
        distance = math.hypot(error_x, error_y)
        angle = self.wrap_angle(math.atan2(error_y, error_x) - self.current_yaw)

        u = distance * math.cos(angle) * self.k_u
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

    def follow_wall(self):
        if not self.num_readings:
            return
        msg = Twist()

        d = self.distance_obj

        front_dist = min(self.num_readings[540:600])
        left_dist = min(self.num_readings[601:860])
        right_dist = min(self.num_readings[280:539])

        self.get_logger().info(f'Distances - Left: {left_dist}, Front: {front_dist}, Right: {right_dist}')

        if (left_dist < d and front_dist > d and right_dist > d) or (left_dist < d and front_dist < d and right_dist > d):
            msg.linear.x = 0.0
            msg.angular.z = -self.turning_speed  # turn right
            self.get_logger().info("Turning RIGHT")
        elif (left_dist > d and front_dist > d and right_dist < d) or (left_dist > d and front_dist < d and right_dist < d) or (left_dist < d and front_dist < d and right_dist < d):
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

        self.range_min = min(self.num_readings[280:860])

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        list_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.current_yaw = euler_from_quaternion(list_orientation)[2]

        self.get_logger().info(f'Odometry update: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw}')

        self.distance = math.sqrt((self.target.pose.position.x - self.current_x) ** 2 + (self.target.pose.position.y - self.current_y) ** 2)

        if self.distance < self.tolerance:
            self.get_logger().info(f'Reached target: x={self.target.pose.position.x}, y={self.target.pose.position.y}')
            self.stop()

    def timer_callback(self):
        if self.range_min < self.distance_obj:
            self.get_logger().info("Following wall")
            self.follow_wall()
        elif self.distance < self.tolerance:
            self.stop()
        else:
            self.get_logger().info("Controlling towards target")
            self.controller()


def main(args=None):
    rclpy.init(args=args)
    bug0_node = Bug0Algorithm()
    rclpy.spin(bug0_node)
    bug0_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
