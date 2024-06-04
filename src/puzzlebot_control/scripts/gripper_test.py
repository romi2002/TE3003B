#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data


from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
# from puzzlebot_control.msg import Gripper
from arucos_interfaces.msg import ArucoMarkers, ArucosDetected, Gripper  # Import the custom message - ArucoMarkers

from std_srvs.srv import Empty
from enum import Enum
import numpy as np

class GripperNodeState(Enum):
    IDLE = 0
    SEARCHING = 1
    ALIGN = 2
    MOVE_TO_GRIP = 3
    GRIP = 4

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)

        # SUBSCRIBERS      
        self.create_subscription(ArucosDetected, '/arucos_detected', self.aruco_callback, 10)
        self.create_timer(0.01, self.timer_callback)

        # PUBLSHERS
        self.servo_pub = self.create_publisher(Float32, '/ServoAngle', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Gripper, '/gripper_state', 10)
        self.idle_pub = self.create_publisher(Bool, '/gripper/idle', 10)
        
        self.start_task_srv = self.create_service(Empty, 'start_grip', self.start_grip_cb)
        self.start_grip = False

        self.robot_velocity = Twist()
        self.servo_angle = Float32()

        # Messages
        self.aruco_msg = ArucosDetected()
        self.aruco = ArucoMarkers()

        # Coordanates of the center of the image
        self.CENTER_IMG_X = (1280 / 2)  # 1280/2 (WIDTH_IMG/2)
        self.CENTER_IMG_Y = (720 / 2) + 250 # 720/2 (HEIGHT_IMG/2)

        self.error_x_integral = 0
        self.error_ang_integral = 0

        # State variables
        self.actual_state = GripperNodeState.IDLE
        self.cube_id = 6
        self.kp_linear, self.kp_angular = 0.1, 0.255
        self.ki_linear, self.ki_angular = 0.0001, 0.0001
        self.kd_linear, self.kd_angular = 0.0, 0.0
        self.error_tol_x = 0.1
        self.error_tol_y = 0.2
        self.servo_grip_angle = 50.0
        self.servo_open_angle = 110.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.max_linear_vel = 0.07
        
        self.grip_start_time = 0
        self.kMoveTime = 1.5

    def aruco_callback(self, msg):
        self.aruco_msg = msg
        self.aruco_msg.detections.sort(key = lambda tag: tag.centroid.y, reverse=True)

    def start_grip_cb(self, req, res):
        self.start_grip = True
        self.get_logger().info("Start grip service called")
        return res

    def timer_callback(self):
        gripper_state = Gripper()
        idle_msg = Bool()
        idle_msg.data = self.actual_state == GripperNodeState.IDLE
        self.idle_pub.publish(idle_msg)       
         
        #self.get_logger().info(f"start_grip {self.start_grip}")
        if self.actual_state == GripperNodeState.IDLE:
            zero_vel = Twist()
            self.velocity_pub.publish(zero_vel)    
        if (self.actual_state == GripperNodeState.IDLE or self.actual_state == GripperNodeState.GRIP) and self.start_grip:
            self.actual_state = GripperNodeState.SEARCHING
            self.start_grip = False

        if self.actual_state == GripperNodeState.SEARCHING:
            self.gripper_state = False
            self.servo_angle.data = self.servo_open_angle
            self.servo_pub.publish(self.servo_angle)
            #self.get_logger().info('Searching')
            self.robot_velocity.angular.z = 0.25
            self.robot_velocity.linear.x = 0.0
            cube_tags = [tag for tag in self.aruco_msg.detections if tag.marker_id == self.cube_id]
            if len(cube_tags):
                self.actual_state = GripperNodeState.ALIGN
                self.robot_velocity.angular.z = 0.0
                self.robot_velocity.linear.x = 0.0
            self.velocity_pub.publish(self.robot_velocity)

        if self.actual_state == GripperNodeState.ALIGN:
            #self.get_logger().info('Aligning')

            # Calculate the dot error
            tag = [tag for tag in self.aruco_msg.detections if tag.marker_id == self.cube_id]
            if len(tag) == 0:
                self.get_logger().info('No Aruco Detected')
                self.actual_state = GripperNodeState.SEARCHING
                return
            tag = tag[0]
            delta_x = self.CENTER_IMG_X - tag.centroid.x
            delta_y = self.CENTER_IMG_Y - tag.centroid.y
            error_x = delta_x / self.CENTER_IMG_X
            error_y = delta_y / self.CENTER_IMG_Y
            distance_error = np.hypot(error_x, error_y)
            gripper_state.distance_error = distance_error
            gripper_state.error_x = error_x
            gripper_state.error_y = error_y
            if abs(error_x) < self.error_tol_x and abs(error_y) < self.error_tol_y:
                self.robot_velocity.angular.z = 0.0
                self.robot_velocity.linear.x = 0.0
                self.velocity_pub.publish(self.robot_velocity)
                #self.get_logger().info('Aligned')
                self.actual_state = GripperNodeState.MOVE_TO_GRIP
                self.grip_start_time = self.get_clock().now()
            else:
                self.robot_velocity.angular.z = self.kp_angular * error_x + self.ki_angular * self.error_x_integral + self.kd_angular * (error_x - self.last_error_x) 
                self.robot_velocity.linear.x = self.kp_linear * error_y + self.ki_linear * self.error_ang_integral + self.kd_linear * (error_y - self.last_error_y)
                self.error_x_integral += error_x
                self.error_ang_integral += error_y
                self.last_error_x = error_x
                self.last_error_y = error_y
                self.robot_velocity.linear.x = min(self.robot_velocity.linear.x, self.max_linear_vel)
                self.velocity_pub.publish(self.robot_velocity)
                
        if self.actual_state == GripperNodeState.MOVE_TO_GRIP:
            if (self.get_clock().now() - self.grip_start_time).nanoseconds > self.kMoveTime * 1e9:
                self.actual_state = GripperNodeState.GRIP
            else:
                move_forward = Twist()
                move_forward.linear.x = 0.7
                move_forward.angular.z = 0.0
                self.velocity_pub.publish(move_forward)

        if self.actual_state == GripperNodeState.GRIP:
            zero_vel = Twist()
            self.velocity_pub.publish(zero_vel)
            #self.get_logger().info('Gripping')
            self.servo_angle.data = self.servo_grip_angle
            self.servo_pub.publish(self.servo_angle)
            #self.actual_state = GripperNodeState.IDLE

        gripper_state.state = str(self.actual_state)
        self.gripper_pub.publish(gripper_state)

        
def main(args=None):
    rclpy.init(args=args)
    gripper_node = GripperNode()
    rclpy.spin(gripper_node)
    gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
