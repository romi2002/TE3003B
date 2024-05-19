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
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        
        self.left_sub = self.create_subscription(
            Float32, 'VelocityEncL', self.left_callback, qos)
        self.right_sub = self.create_subscription(
            Float32, 'VelocityEncR', self.right_callback, qos)
        
        self.br = TransformBroadcaster(self)
        
        self.current_odom = Odometry()
        self.current_odom.pose.pose.position.x = 0.0
        self.current_odom.pose.pose.position.y = 0.0
        self.current_odom.pose.pose.position.z = 0.0
        
        self.yaw = 0.0
        self.wl = 0.0
        self.wr = 0.0

        self.r = 0.05
        self.l = 0.19
        self.dt = 0.01
        
        self.timer = self.create_timer(self.dt, self.update)
        
        self.get_logger().info("Finished initializing odometry.")
    
    def left_callback(self, msg):
        self.wl = msg.data
    
    def right_callback(self, msg):
        self.wr = msg.data

    def update(self):
        # r = 0.05  # wheel radius
        # l = 0.19  # wheel base
        # dt = 0.01  # time step
        
        v = self.r * (self.wr + self.wl) / 2.0  # X Vel
        omega = self.r * (self.wr - self.wl) / self.l  # Angular vel
        
        #self.get_logger().info(f"Wl: {self.wl} Wr: {self.wr} v: {v} omega: {omega} yaw: {self.yaw}")
        
        self.current_odom.pose.pose.position.x += math.cos(self.yaw) * v * self.dt
        self.current_odom.pose.pose.position.y += math.sin(self.yaw) * v * self.dt
        self.yaw += omega * self.dt
        
        quat = quaternion_from_euler(0, 0, self.yaw)
        self.current_odom.pose.pose.orientation.x = quat[0]
        self.current_odom.pose.pose.orientation.y = quat[1]
        self.current_odom.pose.pose.orientation.z = quat[2]
        self.current_odom.pose.pose.orientation.w = quat[3]
        
        self.current_odom.twist.twist.linear.x = v
        self.current_odom.twist.twist.angular.z = omega
        
        # Set pose covariance (example values)
        self.current_odom.pose.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]
        
        # Set twist covariance (example values)
        self.current_odom.twist.covariance = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]
        
        self.current_odom.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(self.current_odom)
        
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
