#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from arucos_interfaces.msg import ArucoMarkers  # Import the custom message - ArucoMarkers
from puzzlebot_control.action import Gripper

class GripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server')

        # SUBSCRIBERS      
        self._actiton_server = self.create_action_server(Gripper, 'gripper', self.execute_callback)

        self.create_subscription(ArucoMarkers, '/aruco_marker', self.aruco_callback, 10)
        self.create_timer(1.0, self.timer_callback)

        # Messages
        self.aruco_msg = ArucoMarkers()

        # Coordanates of the center of the image
        self.CENTER_IMG_X = 160 # 320/2 (WIDTH_IMG/2)
        self.CENTER_IMG_Y = 120 # 240/2 (HEIGHT_IMG/2)

    def aruco_callback(self, msg):
        self.aruco_msg = msg
    
    def timer_callback(self):
        self.get_logger().info('Received ArucoMarkers: %s' % self.aruco_msg)
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Coordanates of the center of the image
        self.CENTER_IMG_X = 160
