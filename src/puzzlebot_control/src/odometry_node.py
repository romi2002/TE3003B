#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import math
from tf_transformations import quaternion_from_euler
import numpy as np

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Publisher to publish odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        
        # Subscribers to wheel velocities
        self.left_sub = self.create_subscription(
            Float32, 'VelocityEncL', self.left_callback, qos)
        self.right_sub = self.create_subscription(
            Float32, 'VelocityEncR', self.right_callback, qos)
        
        self.br = TransformBroadcaster(self)
        
        self.current_odom = Odometry()
        self.current_odom.pose.pose.position.x = 0.0
        self.current_odom.pose.pose.position.y = 0.0
        self.current_odom.pose.pose.position.z = 0.0
        
        # Wheel variables
        self.yaw = 0.0 # Current yaw angle
        self.wl = 0.0 # Left wheel velocity
        self.wr = 0.0 # Right wheel velocity

        self.r = 0.05 # Radius of each wheel
        self.l = 0.19 # Distance between the wheels (wheelbase)
        self.dt = 0.01 # Time step

        # Constants for error model
        self.kr = 0.00001  # Error coefficient for the right wheel
        self.kl = 0.00001  # Error coefficient for the left wheel
        
        self.timer = self.create_timer(self.dt, self.update)
        
        self.get_logger().info("Finished initializing odometry.")
    
    def left_callback(self, msg):
        self.wl = msg.data
    
    def right_callback(self, msg):
        self.wr = msg.data

    def update(self):
        
        v = self.r * (self.wr + self.wl) / 2.0  # X Vel
        omega = self.r * (self.wr - self.wl) / self.l  # Angular vel
        
        #self.get_logger().info(f"Wl: {self.wl} Wr: {self.wr} v: {v} omega: {omega} yaw: {self.yaw}")
        
        self.current_odom.header.frame_id = 'map'
        self.current_odom.child_frame_id = 'odom'
        # Update position
        self.current_odom.pose.pose.position.x += math.cos(self.yaw) * v * self.dt
        self.current_odom.pose.pose.position.y += math.sin(self.yaw) * v * self.dt

        # Update orientation
        self.yaw += omega * self.dt # Update yaw
        quat = quaternion_from_euler(0, 0, self.yaw)
        self.current_odom.pose.pose.orientation.x = quat[0]
        self.current_odom.pose.pose.orientation.y = quat[1]
        self.current_odom.pose.pose.orientation.z = quat[2]
        self.current_odom.pose.pose.orientation.w = quat[3]
        
        # Calculate Jacobian
        H_k = np.array([
            [1, 0, -v * math.sin(self.yaw) * self.dt],
            [0, 1, v * math.cos(self.yaw) * self.dt],
            [0, 0, 1]
        ])
        # DEfine the error covariance matrix
        Q_k = np.diag([self.kr * abs(self.wr) * self.dt, 
                       self.kl * abs(self.wl) * self.dt, 
                       (self.kr * abs(self.wr) + self.kl * abs(self.wl)) * self.dt])

        # Update covariance matrix using the previous covariance matrix
        if not hasattr(self, 'sigma'):
            self.sigma = np.eye(3)  # Initializes the covariance matrix if it hasn't been defined

        # Predict the new covariance matrix
        self.sigma = H_k.dot(self.sigma).dot(H_k.T) + Q_k

        # Extend 3x3 matrix to 6x6 for ROS compatibility
        sigma_full = np.zeros((6, 6))
        sigma_full[:3, :3] = self.sigma  # Fill in the 3x3 position covariance
        sigma_full[3, 3] = 0.001  # Small value for orientation around x (roll)
        sigma_full[4, 4] = 0.001  # Small value for orientation around y (pitch)
        sigma_full[5, 5] = 0.001  # Small value for orientation around z (yaw)
        
        # Set pose covariance (example values)
        self.current_odom.pose.covariance = sigma_full.flatten().tolist()  # Set the pose covariance matrix as a list
        
        # Set linear and angular velocity
        self.current_odom.twist.twist.linear.x = v
        self.current_odom.twist.twist.angular.z = omega
        
        self.current_odom.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(self.current_odom)
        
        # Publish the transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map'
        transform_stamped.child_frame_id = 'odom'
        transform_stamped.transform.translation.x = self.current_odom.pose.pose.position.x
        transform_stamped.transform.translation.y = self.current_odom.pose.pose.position.y
        transform_stamped.transform.translation.z = self.current_odom.pose.pose.position.z
        transform_stamped.transform.rotation.x = quat[0]
        transform_stamped.transform.rotation.y = quat[1]
        transform_stamped.transform.rotation.z = quat[2]
        transform_stamped.transform.rotation.w = quat[3]
        
        self.br.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
