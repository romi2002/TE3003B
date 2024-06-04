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
        self.create_timer(0.01, self.timer_callback)

        # PUBLSHERS
        self.servo_pub = self.create_publisher(Float32, '/ServoAngle', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Gripper, '/gripper_state', 10)
        
        self.robot_velocity = Twist()
        self.servo_angle = Float32()
        self.gripper_start = True

        # Messages
        self.aruco_msg = ArucosDetected()
        self.aruco = ArucoMarkers()

        # Coordanates of the center of the image
        self.CENTER_IMG_X = (1280 / 2) + 400  # 1280/2 (WIDTH_IMG/2)
        self.CENTER_IMG_Y = 720 / 2 # 720/2 (HEIGHT_IMG/2)

        self.error_x_integral = 0
        self.error_ang_integral = 0

        # State variables
        self.actual_state = 'FINISHED'
        self.cube_id = 6
        self.kp_linear, self.kp_angular = 0.55, 0.45
        self.ki_linear, self.ki_angular = 0.0001, 0.0001
        self.kd_linear, self.kd_angular = 0.0055, 0.0055
        self.error_tol_x = 0.1
        self.error_tol_y = 0.1
        self.servo_grip_angle = 25.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0

    def aruco_callback(self, msg):
        self.aruco_msg = msg
        self.aruco_msg.detections.sort(key = lambda tag: tag.area.data)

    def cube_callback(self, msg):
        self.gripper_start = msg.data
    
    def timer_callback(self):
        gripper_state = Gripper()
        if self.gripper_start and self.actual_state == 'FINISHED':
            self.actual_state = 'SEARCHING'
        
        if self.actual_state == 'SEARCHING':
            self.gripper_state = False
            open_angle = Float32()
            open_angle.data = 70.0
            self.servo_pub.publish(open_angle)
            self.get_logger().info('Searching')
            self.robot_velocity.angular.z = 0.3
            self.robot_velocity.linear.x = 0.0
            for tag in self.aruco_msg.detections:
                if tag.marker_id != self.cube_id:
                    continue
                self.actual_state = 'ALIGN'
                self.robot_velocity.angular.z = 0.0
                self.robot_velocity.linear.x = 0.0
                break
            self.velocity_pub.publish(self.robot_velocity)

        if self.actual_state == 'ALIGN':
            self.get_logger().info('Aligning')
            # Calculate the dot error
            tag = [tag for tag in self.aruco_msg.detections if tag.marker_id == self.cube_id]
            if len(tag) == 0:
                self.get_logger().info('No Aruco Detected')
                self.actual_state = 'SEARCHING'
                return
            tag = tag[0]
            delta_x = self.CENTER_IMG_X - tag.centroid.x
            delta_y = self.CENTER_IMG_Y - tag.centroid.y
            error_x = delta_x / self.CENTER_IMG_X
            error_y = abs(delta_y / self.CENTER_IMG_Y)
            distance_error = np.hypot(error_x, error_y)
            gripper_state.distance_error = distance_error
            gripper_state.error_x = error_x
            gripper_state.error_y = error_y
            if abs(error_x) < self.error_tol_x and abs(error_y) < self.error_tol_y:
                self.robot_velocity.angular.z = 0.0
                self.robot_velocity.linear.x = 0.0
                self.velocity_pub.publish(self.robot_velocity)
                self.get_logger().info('Aligned')
                self.actual_state = 'GRIP'
            else:
                self.robot_velocity.angular.z = self.kp_angular * error_x + self.ki_angular * self.error_x_integral + self.kd_angular * (error_x - self.last_error_x) 
                self.robot_velocity.linear.x = self.kp_linear * error_y + self.ki_linear * self.error_ang_integral + self.kd_linear * (error_y - self.last_error_y)
                self.error_x_integral += error_x
                self.error_ang_integral += error_y
                self.last_error_x = error_x
                self.last_error_y = error_y
                self.velocity_pub.publish(self.robot_velocity)

        if self.actual_state == 'GRIP':
            zero_vel = Twist()
            self.velocity_pub.publish(zero_vel)
            self.get_logger().info('Gripping')
            self.servo_angle.data = self.servo_grip_angle
            #self.actual_state = 'FINISHED'
            self.servo_pub.publish(self.servo_angle)
            rclpy.shutdown()

        gripper_state.state = self.actual_state
        self.gripper_pub.publish(gripper_state)

        
def main(args=None):
    rclpy.init(args=args)
    gripper_node = GripperNode()
    rclpy.spin(gripper_node)
    gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
