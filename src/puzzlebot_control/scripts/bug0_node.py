#!/usr/bin/env python3

import rclpy
import math
import yaml
import os
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion


class Bug0Algorithm(Node):
    def __init__(self):
        super().__init__('bug0_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to robot's current pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/puzzlebot1/odometry',
            self.odom_callback,
            10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/lidar', self.scan_callback, qos_profile = qos_profile_sensor_data)

        # Load the points from the YAML file
        self.declare_parameter('points_file', os.path.join(get_package_share_directory('puzzlebot_control'), 'config', 'point.yaml'))
        points_file = self.get_parameter('points_file').get_parameter_value().string_value

        self.get_logger().info(f'Loading points from file: {points_file}')

        try:
            with open(points_file, 'r') as file:
                self.points = yaml.safe_load(file)['points']
        except Exception as e:
            self.get_logger().error(f'Failed to load points from file: {e}')
            self.points = []

        self.current_index = 0
        self.target_reached = True  # Initially, no target is set so we can start with the first point

        self.tolerance = 0.1  # Tolerance to consider the target reached

        # Tolerance to consider a near object
        self.distance_obj = 0.2

        # Maximum speed of the robot
        self.forward_speed = 0.2

        # Turning speed to left
        self.turning_speed = 0.4

        self.k_u = 0.25
        self.k_r = 0.85 

        # Initialize a timer to periodically call timer_callback
        self.timer = self.create_timer(1.0, self.timer_callback)

    def wrapAngle(self, angle):
        # Wraps an angle to [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))

    def controller(self):
        msg = Twist()
        max_vel = 0.3
        max_ang = 0.2

        # Compute error distance and angle to target
        # using the target point and the odometry of the robot
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        distance = math.hypot(error_x, error_y)
        angle_aux = math.atan2(error_y, error_x) - self.current_yaw
        print("ANGEL:", angle_aux)
        angle = self.wrapAngle(math.atan2(error_y, error_x) - self.current_yaw)


        # Compute velocities with proportional control
        u = distance * math.cos(angle) * self.k_u
        r = angle * self.k_r

        # Stop when close
        if distance < 0.05:
            u = 0
            r = 0

        if u > max_vel:
            u = max_vel
        if r > max_ang:
            r = max_ang

        msg.linear.x = u
        msg.angular.z = r

        self.vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

    def follow_wall(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        d = self.distance_obj

        front_dist = min(self.num_readings[542:605])
        left_dist = min(self.num_readings[606:860])
        right_dist = min(self.num_readings[287:541])

        print("Distance left object: ", left_dist)
        print("Distance front object: ", front_dist)
        print("Distance right object: ", right_dist)

        if (left_dist < d and front_dist > d and right_dist > d) or (left_dist < d and front_dist < d and right_dist > d):
            msg.linear.x = 0.0
            msg.angular.z = -self.turning_speed   # Turn right 
            print(" RIGHT")
        elif (left_dist > d and front_dist > d and right_dist < d) or (left_dist > d and front_dist < d and right_dist < d) or (left_dist < d and front_dist < d and right_dist < d):
            msg.linear.x = 0.0
            msg.angular.z = self.turning_speed    # turn left
            print("LEFT")

        else:
            msg.linear.x = self.forward_speed     # forward
            msg.angular.z = 0.0

        self.vel_pub.publish(msg)

    def scan_callback(self, msg):
        self.scan = msg
        self.num_readings = list(msg.ranges)
        for i in range(len(self.num_readings)):
            if math.isnan(self.num_readings[i]):
                self.num_readings[i] = msg.range_min - 0.0001
            elif math.isinf(self.num_readings[i]):
                self.num_readings[i] = msg.range_max + 0.0001

        range_min = min(self.num_readings[287:860])

        if range_min < self.distance_obj:
            print("IN FOLLOWING WALL")
            self.follow_wall()
        elif self.distance < self.tolerance:
            print("TERMINE")
            self.stop()
        else:
            print("IN CONTROLLER")
            self.controller()

        # front
        # self.front_dist = min(min(num_readings[0:31]), min(num_readings[1106:1146]))
        # self.right_dist = min(num_readings[32:287])
        # self.left_dist = min(num_readings[860:1105])

        # back using in simulation
        # self.front_dist = min(num_readings[542:613])
        # self.left_dist = min(num_readings[614:868])
        # self.right_dist = min(num_readings[287:541])

        # print(num_readings)
        # segment = (num_readings // 3)-2
        # print(3*segment)
        # self.right_dist = msg.ranges[segment]
        # self.front_dist = msg.ranges[2*segment]
        # self.left_dist = msg.ranges[3*segment]

    def odom_callback(self, msg):
        if not self.target_reached:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            # get yaw from quaternion
            list_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            self.current_yaw = euler_from_quaternion(list_orientation)[2]

            self.target_x = self.points[self.current_index]['x']
            self.target_y = self.points[self.current_index]['y']

            self.distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
            # print(distance)

            if self.distance < self.tolerance:
                self.target_reached = True
                self.get_logger().info(f'Reached target: x={self.target_x}, y={self.target_y}')
                self.current_index += 1

    def publish_next_target(self):
        if self.current_index < len(self.points):
            point = self.points[self.current_index]
            self.target_reached = False
            
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            msg.pose.position.x = point['x']
            msg.pose.position.y = point['y']
            msg.pose.position.z = 0.0

            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing target_pose: x={point["x"]}, y={point["y"]}')
        else:
            self.get_logger().info('All points have been published')
            rclpy.shutdown()

    def timer_callback(self):
        if self.target_reached:
            self.publish_next_target()
# `    def run(self):
#         self.follow_wall()`


def main(args=None):
    rclpy.init(args=args)

    bug0_node = Bug0Algorithm()
    rclpy.spin(bug0_node)

    bug0_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
