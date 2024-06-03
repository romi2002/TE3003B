#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from enum import Enum

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from arucos_interfaces.msg import ArucoMarkers, ArucosDetected
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from tf_transformations import euler_from_quaternion
from puzzlebot_interfaces import StateControllerStatus


def wrap_angle(angle):
    angle = math.copysign(math.fmod(angle, 2 * math.pi), angle)
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle


class RandomWalkState(Enum):
    FORWARD = 1
    TURNING = 2


class MoveToPoint:
    def __init__(self):
        self.k_u = 0.5
        self.k_r = 0.75

    def update(self, pose, target):
        x, y, theta = pose
        tx, ty = target

        ex, ey = tx - x, ty - y
        dist = np.hypot(ex, ey)
        angl = wrap_angle(np.arctan2(ey, ex) - theta)

        return dist * np.cos(angl) * self.k_u, angl * self.k_r


class RandomWalk:
    def __init__(self):
        self.state = RandomWalkState.FORWARD
        self.k_obstacle_threshold = 0.5
        self.turn_angle = None
        self.k_angle_to_turn = np.pi / 4
        self.k_angle_to_turn_tol = 0.1
        self.k_r = 0.1

    def update(self, pose, front_scan):
        _, _, theta = pose
        # Condition changes
        if (
            self.state == RandomWalkState.FORWARD
            and min(front_scan) < self.k_obstacle_threshold
        ):
            self.state = RandomWalkState.TURNING
            self.turn_angle = wrap_angle(theta + self.k_angle_to_turn)
        elif (
            self.state == RandomWalkState.TURNING
            and np.abs(theta - self.turn_angle) < self.k_angle_to_turn_tol
        ):
            self.state = RandomWalkState.FORWARD

        u, r = 0, 0
        if self.state == RandomWalkState.FORWARD:
            u = 0.5
            r = 0
        elif self.state == RandomWalkState.TURNING:
            u = 0
            r = (theta - self.turn_angle) * self.k_r

        return (u, r)


class ControllerState(Enum):
    MAPPING = 1
    MOVE_TO_POINT = 2
    GRIP = 3
    RELEASE = 4


class ControllerNode(Node):
    def __init__(self):
        super().__init__("master_controller_node")

        self.pose_controller_vel_sub = self.create_subscription(
            Twist, "controller/cmd_vel", self.controller_cmd_vel_cb, 1
        )
        self.gripper_cmd_sub = self.create_subscription(
            Twist, "gripper/cmd_vel", self.gripper_cmd_vel_cb, 1
        )
        self.gripper_cmd_vel = None

        self.aruco_sub = self.create_subscription(
            ArucosDetected, "arucos_detected", self.arucos_detected_cb, 1
        )

        self.gripper_cmd_pub = self.create_publisher(Bool, "/gripper_start", 1)
        self.servo_pub = self.create_publisher(Float32, "/ServoAngle", 1)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data
        )
        self.front_scan = None

        self.grip_sub = self.create_subscription(
            Gripper, "/gripper_state", self.gripper_cb
        )
        self.grip_succesful = False

        self.controller_state_pub = self.create_publisher(
            StateControllerStatus, "/state_controller/status", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.01, self.tf_timer_cb)
        self.pose = None  # X, Y, Theta

        self.state = ControllerState.MAPPING
        self.target_pose = None
        self.distance_tol = 0.1
        self.next_state = None
        self.timer = self.create_timer(0.01, self.update)

        self.detected_arucos = None
        self.detected_aruco_ids = set()
        self.cube_id = 6
        self.random_walk = RandomWalk()
        self.move_to_point = MoveToPoint()

    def tf_timer_cb(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        except TransformException as ex:
            self.get_logger().error(f"Could not transform {ex}")
            return
        self.pose = np.array(
            (t.transform.translation.x, t.transform.translation.y, yaw)
        )

    def scan_cb(self, msg):
        pass

    def controller_cmd_vel_cb(self, msg):
        pass

    def gripper_cmd_vel_cb(self, msg):
        pass

    def arucos_detected_cb(self, msg):
        self.detected_arucos = msg
        self.detected_aruco_ids = set([t.marker_id for t in msg.detections])

    def update(self):
        status = StateControllerStatus()
        if self.front_scan is None or self.pose is None:
            return

        distance_to_goal = (
            np.hypot(
                self.target_pose[0] - self.pose[0], self.target_pose[1] - self.pose[1]
            )
            if self.target_pose
            else 0
        )

        # State changes
        if (
            self.state == ControllerState.MAPPING
            and self.cube_id in self.detected_aruco_ids
        ):
            self.state = ControllerState.GRIP
            self.grip_succesful = False
        elif self.state == ControllerState.GRIP and self.grip_succesful:
            self.state = ControllerState.MOVE_TO_POINT
            self.next_state = ControllerState.RELEASE
        elif (
            self.state == ControllerState.MOVE_TO_POINT
            and distance_to_goal < self.distance_tol
        ):
            self.state = self.next_state
        else:
            # Reached RELEASE, stay here (for now)
            pass

        u, r = 0, 0
        if self.state == ControllerState.MAPPING:
            u, r = self.random_walk.update(self.pose, self.front_scan)
            status.random_walk_state = str(self.random_walk.state)
        elif self.state == ControllerState.GRIP:
            u, r = self.gripper_cmd_vel if self.gripper_cmd_vel else (0, 0)
        elif self.state == ControllerState.MOVE_TO_POINT:
            u, r = self.move_to_point.update(self.pose, self.target_pose)
            status.distance_to_goal.data = distance_to_goal
        elif self.state == ControllerState.RELEASE:
            u, r = (0, 0)

        status.state = str(self.state)

        cmd_vel = Twist()
        cmd_vel.linear.x, cmd_vel.angular.z = u, r
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)

    master_node = ControllerNode()
    rclpy.spin(master_node)

    master_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
