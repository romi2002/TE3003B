#!/usr/bin/env python3

from collections import namedtuple
import numpy as np
import matplotlib.pyplot as plt
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from arucos_interfaces.msg import ArucosDetected, ArucoMarkers
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Float32

TagDetection = namedtuple("TagDetection", "idx range bearing")


# From probabilistic robotics
class EKF_SLAM_Correspondence:
    def __init__(self, R, Q, valid_tags):
        self.max_landmarks = len(valid_tags)
        self.N2 = self.max_landmarks * 2
        self.state = np.zeros((3 + self.N2))

        # 3x3 -> robot states
        self.sigma = 1e-4 * np.ones((3 + self.N2, 3 + self.N2))
        for i in range(3, 3 + self.N2):
            # Large values for feature variances, as we do not have any info.
            self.sigma[i][i] = 1e6

        # State cov matrix
        self.R = R

        # Meas cov matrix
        self.Q = Q
 
        # Kalman gain
        self.K = np.zeros((3, 3))

        self.valid_tags = valid_tags
        self.observed_tags = set()
        self.last_time = None

    # Control Input -> [v, omega]
    def update(self, u, time):
        if self.last_time is None:
            self.last_time = time
            return

        dt = time - self.last_time
        v, omega = u
        t_cos, t_sin = np.cos(self.state[2]), np.sin(self.state[2])
        x_t = self.state[0] + v * t_cos * dt
        y_t = self.state[1] + v * t_sin * dt
        theta_t = self.state[2] + omega * dt

        self.state[:3] = np.array((x_t, y_t, theta_t))

        # Linearize with Jacobian
        G = np.identity((3 + self.N2))
        G[0][2] = dt * -v * t_sin
        G[1][2] = dt * v * t_cos

        # Covariance update
        self.sigma = G @ self.sigma @ G.T
        self.sigma[0][0] += self.R[0][0]
        self.sigma[1][1] += self.R[1][1]
        self.sigma[2][2] += self.R[2][2]
        self.last_time = time

    def measurement_update(self, z: TagDetection):
        if z.idx not in self.valid_tags:
            return False

        x, y, theta = self.state[:3]

        # Tag id != index of tag in state.
        # Index is given by order from valid_tags
        tag_state_index = self.valid_tags.index(z.idx)
        # print(tag_state_index)

        if z.idx not in self.observed_tags:
            # Initialize tag location in state vector
            x_l = x + z.range * np.cos(z.bearing + theta)
            y_l = y + z.range * np.sin(z.bearing + theta)
            self.state[2 * tag_state_index + 1 : 2 * tag_state_index + 3] = x_l, y_l
            self.observed_tags.add(z.idx)
        else:
            x_l, y_l = self.state[2 * tag_state_index + 1 : 2 * tag_state_index + 3]

        # Measurement update
        dx = x_l - x
        dy = y_l - y
        q = dx**2 + dy**2
        expected_range = np.hypot(dx, dy)
        expected_bearing = np.arctan2(dy, dx) - theta

        # Linearize model
        F = np.zeros((5, 3 + self.N2))
        F[0][0], F[1][1], F[2][2] = 1, 1, 1
        F[3][2 * tag_state_index + 1] = 1
        F[3][2 * tag_state_index + 2] = 1

        H = (
            np.array(
                [
                    (
                        -dx / expected_range,
                        -dy / expected_range,
                        0,
                        dx / expected_range,
                        dy / expected_range,
                    ),
                    (dy / q, -dx / q, -1, -dy / q, dx / q),
                    np.zeros(5),
                ]
            )
            @ F
        )

        # Gain update
        self.K = self.sigma @ H.T @ np.linalg.inv(H @ self.sigma @ H.T + self.Q)

        # Prediction
        self.state = self.state + self.K @ np.array(
            [z.range - expected_range, z.bearing - expected_bearing, 0]
        )

        # Cov update
        self.sigma = (np.identity(self.sigma.shape[0]) - self.K @ H) @ self.sigma
        return True


class EKFSlamNode(Node):
    def __init__(self):
        super().__init__("ekf_slam_node")

        self.measurement_sub = self.create_subscription(
            ArucosDetected, "arucos_detected", self.measurement_cb, 10
        )

        qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        self.left_sub = self.create_subscription(
            Float32, 'VelocityEncL', self.left_cb, qos)
        self.right_sub = self.create_subscription(
            Float32, 'VelocityEncR', self.right_cb, qos)
        self.wl, self.wr = None, None

        R = np.diagflat(np.array([0.1, 0.1, 1]) ** 2)
        Q = np.diagflat(np.array((1, 1, 1e16)) ** 2)
        self.slam = EKF_SLAM_Correspondence(R, Q, range(12))

        self.r = 0.05 # Radius of each wheel
        self.l = 0.19 # Distance between the wheels (wheelbase)

        self.br = TransformBroadcaster(self)
        self.odom_timer = self.create_timer(0.01, self.odom_update)
        self.timer = self.create_timer(0.1, self.timer_update)

    def left_cb(self, msg):
        self.wl = msg.data

    def right_cb(self, msg):
        self.wr = msg.data

    def odom_update(self):
        if self.wl is None or self.wr is None:
            return
        
        v = self.r * (self.wr + self.wl) / 2.0  # X Vel
        omega = self.r * (self.wr - self.wl) / self.l  # Angular vel
        self.get_logger().info(f"v: {v} omega: {omega}")
        self.slam.update(np.array((v, omega)), self.get_clock().now().nanoseconds * 1e-9)

    def measurement_cb(self, msg : ArucosDetected):
        for d in msg.detections:
            # if d.marker_id == 6:
            #     # Ignore cube tag ID
            #     return
            
            self.get_logger().info(f"Seen: {d.marker_id}")
            det = TagDetection(d.marker_id, d.range.data, d.bearing.data)
            ret = self.slam.measurement_update(det)
            if not ret:
                self.get_logger().error("Failed to perform measurment update.")
    
    # Utility function, creates quat and translation from [x, y, theta]
    @staticmethod
    def create_transform(pose: np.array):
        ts = TransformStamped()
        ts.transform.translation.x = pose[0]
        ts.transform.translation.y = pose[1]
        ts.transform.translation.z = 0.0
        
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = quaternion_from_euler(0, 0, pose[2])
        return ts


    def publish_slam_state(self):
        robot_pose = self.slam.state[:3]
        transform = EKFSlamNode.create_transform(robot_pose)
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        self.br.sendTransform(transform)

        for tag in self.slam.observed_tags:
            idx = self.slam.valid_tags.index(tag)
            x, y = self.slam.state[2 * idx + 1 : 2 * idx + 3]
            tag_transform = EKFSlamNode.create_transform(np.array((x, y, 0)))
            tag_transform.header.stamp = self.get_clock().now().to_msg()
            tag_transform.header.frame_id = 'map'
            tag_transform.child_frame_id = f'tag_{idx}'
            self.br.sendTransform(tag_transform)

    def timer_update(self):
        self.publish_slam_state()
        #self.get_logger().info(f"State: {self.slam.state}")

if __name__ == "__main__":
    rclpy.init()
    slam_node = EKFSlamNode()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()