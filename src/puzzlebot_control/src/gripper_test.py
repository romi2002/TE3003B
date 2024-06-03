#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data


from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
# from puzzlebot_control.msg import Gripper
from arucos_interfaces.msg import ArucoMarkers, ArucosDetected, Gripper  # Import the custom message - ArucoMarkers

import numpy as np

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)


        # SUBSCRIBERS      
        self.create_subscription(ArucosDetected, '/arucos_detected', self.aruco_callback, 10)
        self.create_subscription(Bool, '/gripper_start',self.cube_callback,10)
        self.create_timer(1.0, self.timer_callback)

        # PUBLSHERS
        self.servo_pub = self.create_publisher(Float32, '/ServoAngle', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Gripper, '/gripper_state', 10)
        
        self.robot_velocity = Twist()
        self.servo_angle = Float32()
        self.gripper_start = Bool()
        self.gripper_state = Gripper()

        # Messages
        self.aruco_msg = ArucosDetected()
        self.aruco = ArucoMarkers()

        # Coordanates of the center of the image
        self.CENTER_IMG_X = 640 # 1280/2 (WIDTH_IMG/2)
        self.CENTER_IMG_Y = 360 # 720/2 (HEIGHT_IMG/2)

        # State variables
        self.actual_state = 'IDLE'

    def aruco_callback(self, msg):
        self.aruco_msg = msg

    def cube_callback(self, msg):
        self.gripper_start = msg
    
    def timer_callback(self):

        if self.gripper_start:
            self.actual_state = 'SEARCHING'
        
        if self.actual_state == 'SEARCHING':
            self.get_logger().info('Searching')
            if self.aruco_msg.detections:
                for i in self.aruco_msg.detections:
                    if i.marker_id == 6:
                        self.actual_state = 'ALIGN'
                        self.robot_velocity.angular.z = 0.0
                        self.robot_velocity.linear.x = 0.0
                        break
                    else:
                        self.robot_velocity.angular.z = 0.2
                        self.robot_velocity.linear.x = 0.0
            self.velocity_pub.publish(self.robot_velocity)

        if self.actual_state == 'ALIGN':
            self.get_logger().info('Aligning')
            # Calculate the dot error
            tag = next(tag for tag in self.aruco_msg.detections if tag.marker_id == 6)
            if tag is None:
                self.get_logger().info('No Aruco Detected')
                self.actual_state = 'SEARCHING'
                return
            
            delta_x = self.CENTER_IMG_X - tag.centroid.x
            delta_y = self.CENTER_IMG_Y - tag.centroid.y
            self.error_x = (delta_x* np.pi)/ self.CENTER_IMG_X
            self.error_y = delta_y / self.CENTER_IMG_Y
            self.distance_error = np.sqrt((np.power(delta_x,2))+(np.power(delta_y,2)))
            self.gripper_state.distance_error = self.distance_error
            
            if self.distance_error < 10:
                self.robot_velocity.angular.z = 0.0
                self.robot_velocity.linear.x = 0.0
                self.velocity_pub.publish(self.robot_velocity)
                self.get_logger().info('Aligned')
                self.actual_state = 'GRIP'
            else:
                self.robot_velocity.angular.z = self.kp * self.error_x
                self.robot_velocity.linear.x = self.kp * self.error_y
                self.velocity_pub.publish(self.robot_velocity)

        if self.actual_state == 'GRIP':
            self.get_logger().info('Gripping')
            self.servo_angle.data = 20
            self.actual_state = 'FINISHED'
            self.servo_pub.publish(self.servo_angle)

        if self.actual_state == 'FINISHED':
            return
        
        self.gripper_state.state = self.actual_state
        self.gripper_pub.publish(self.gripper_state)

        
            