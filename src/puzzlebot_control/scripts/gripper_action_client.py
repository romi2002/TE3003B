#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from arucos_interfaces.msg import ArucoMarkers  # Import the custom message - ArucoMarkers
from puzzlebot_control.action import Gripper
import numpy as np

class GripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_action_client')

        # SUBSCRIBERS      
        self.create_subscription(ArucoMarkers, '/aruco_marker', self.aruco_callback, 10)
        self.create_timer(1.0, self.timer_callback)

        # ACTION CLIENT
        self._action_client = ActionClient(self, Gripper, 'gripper')

        # Messages
        self.aruco_msg = ArucoMarkers()

        # Coordanates of the center of the image
        self.CENTER_IMG_X = 160 # 320/2 (WIDTH_IMG/2)
        self.CENTER_IMG_Y = 120 # 240/2 (HEIGHT_IMG/2)

        # Request variables
        self.initial_error = []
        self.error_x = None
        self.error_y = None

    def aruco_callback(self, msg):
        self.aruco_msg = msg
    
    def timer_callback(self):
        if self.aruco_msg.marker_id == 6:
            self.get_logger().info('Received ArucoMarkers: %s' % self.aruco_msg.marker_id)
            self.send_goal()
    
    def send_goal(self):
        goal_msg = Gripper.Goal()
        self.error_x = ((self.CENTER_IMG_X - self.aruco_msg.centroid.x)* np.pi)/ self.CENTER_IMG_X
        self.error_y = (self.CENTER_IMG_Y - self.aruco_msg.centroid.y) / self.CENTER_IMG_Y
        self.get_logger().info('Initial Error x: %s, Initial Error y: %s' % (self.error_x, self.error_y))
        goal_msg.initial_error = [self.error_x, self.error_y]

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
    
    def get_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.remaining_error))
            

def main(args=None):
    rclpy.init(args=args)
    node = GripperActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()