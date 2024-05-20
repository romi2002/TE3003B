#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class SetPointPublisher(Node):

    def __init__(self):
        super().__init__('set_point_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)

        # Subscribe to robot's current pose
        self.subscription_ = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # Load the points from the YAML file
        self.declare_parameter('points_file', os.path.join(get_package_share_directory('puzzlebot_control'), 'config', 'points.yaml'))
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

        # Initialize a timer to periodically call timer_callback
        self.timer = self.create_timer(1.0, self.timer_callback)

    def odom_callback(self, msg):
        if not self.target_reached:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y

            target_x = self.points[self.current_index]['x']
            target_y = self.points[self.current_index]['y']

            distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            print(distance)

            if distance < self.tolerance:
                self.target_reached = True
                self.get_logger().info(f'Reached target: x={target_x}, y={target_y}')
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

def main(args=None):
    rclpy.init(args=args)
    node = SetPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
