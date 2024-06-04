#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from enum import Enum

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, PoseStamped
from arucos_interfaces.msg import ArucoMarkers, ArucosDetected, Gripper
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from tf_transformations import euler_from_quaternion
from puzzlebot_interfaces.msg import StateControllerStatus

from std_srvs.srv import Empty

def wrap_angle(angle):
    angle = math.copysign(math.fmod(angle, 2 * math.pi), angle)
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle

def get_scan_within_range(scan, min_angle, max_angle):
    a1, a2 = wrap_angle(min_angle), wrap_angle(max_angle)
    min_angle, max_angle = min(a1, a2), max(a1, a2)
    
    ranges = []
    for i, range in enumerate(scan.ranges):
        angle = wrap_angle(i * scan.angle_increment + scan.angle_min + np.pi)
        if min_angle < angle < max_angle:
            ranges.append(range)
    return ranges

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
    def __init__(self, logger, clock):
        self.state = RandomWalkState.FORWARD
        self.k_obstacle_threshold = 0.5
        self.k_angle_to_turn = np.pi / 4
        self.k_angle_to_turn_tol = 0.5
        self.k_turn_time = 5
        self.turn_time_remaining = 0
        self.clock = clock        
        self.turn_start_time = self.clock.now()
        self.logger = logger

    def update(self, pose, front_scan):
        _, _, theta = pose
        self.turn_time_remaining = (self.clock.now() - self.turn_start_time).nanoseconds - self.k_turn_time * 1e9
        # Condition changes
        if (
            self.state == RandomWalkState.FORWARD
            and min(front_scan) < self.k_obstacle_threshold
        ):
            self.state = RandomWalkState.TURNING
            self.turn_start_time = self.clock.now()
            self.logger.info("Turning")
        elif (
            self.state == RandomWalkState.TURNING
            and self.turn_time_remaining > 0
        ):
            self.state = RandomWalkState.FORWARD
            self.logger.info("Finished turning")

        u, r = 0, 0
        if self.state == RandomWalkState.FORWARD:
            u = 0.5
            r = 0
        elif self.state == RandomWalkState.TURNING:
            u = 0
            r = 0.75

        return (u, r)


class ControllerState(Enum):
    MAPPING = 1
    MOVE_TO_POINT = 2
    GRIP = 3
    RELEASE = 4
    BACKUP = 5
    RETURN_TO_HOME = 6


class ControllerNode(Node):
    def __init__(self):
        super().__init__("master_controller_node")

        self.pose_controller_vel_sub = self.create_subscription(
            Twist, "controller/cmd_vel", self.controller_cmd_vel_cb, 1
        )
        self.controller_cmd_vel = None

        self.gripper_cmd_sub = self.create_subscription(
            Twist, "gripper/cmd_vel", self.gripper_cmd_vel_cb, 1
        )
        self.gripper_cmd_vel = None
        self.gripper_servo_sub = self.create_subscription(
            Float32, "gripper/servo_angle", self.gripper_servo_cb, 10
        )
        self.gripper_servo_cmd = 110.0

        self.aruco_sub = self.create_subscription(
            ArucosDetected, "arucos_detected", self.arucos_detected_cb, 1
        )

        self.gripper_srv = self.create_client(Empty, '/start_grip')
        while not self.gripper_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available')
        
        self.servo_pub = self.create_publisher(Float32, "/ServoAngle", 1)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data
        )
        self.front_scan = None
        
        self.target_pub = self.create_publisher(
            PoseStamped, "/target_pose", 10
        )

        self.grip_sub = self.create_subscription(
            Gripper, "/gripper_state", self.gripper_cb, 10
        )
        self.grip_state = Gripper()
        self.grip_idle_sub = self.create_subscription(
            Bool, "/gripper/idle", self.gripper_idle_cb, 10
        )
        self.is_gripper_idle = False
        self.grip_succesful = False

        self.controller_state_pub = self.create_publisher(
            StateControllerStatus, "/state_controller/status", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.01, self.tf_timer_cb)
        self.pose = None  # X, Y, Theta

        self.state = ControllerState.MAPPING
        self.last_state = None
        self.target_pose = None
        self.distance_tol = 0.2
        self.next_state = None
        self.timer = self.create_timer(0.01, self.update)

        self.detected_arucos = None
        self.detected_aruco_ids = set()
        self.cube_id = 6
        self.random_walk = RandomWalk(self.get_logger(), self.get_clock())
        self.move_to_point = MoveToPoint()
        self.target_id = 8
        self.home_id = 9
        
        self.backup_start = self.clock.now()
        self.k_backup_time = 2.5
        
    def gripper_servo_cb(self, msg):
        self.gripper_servo_cmd = msg.data
        
    def get_tag_pose(self, id):
        try:
            t = self.tf_buffer.lookup_transform("map", f"tag_{id}", rclpy.time.Time())
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        except TransformException as ex:
            self.get_logger().error(f"Could not transform {ex}")
            return None
        return np.array(
            (t.transform.translation.x, t.transform.translation.y)
        )
        
    def send_target_pose(self, target_pose):
        msg = PoseStamped()
        msg.pose.position.x, msg.pose.position.y = target_pose
        self.target_pub.publish(msg)

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
        
    def gripper_idle_cb(self, msg):
        self.is_gripper_idle = msg.data

    def gripper_cb(self, msg):
        self.grip_state = msg

    def scan_cb(self, msg):
        self.front_scan = get_scan_within_range(msg, -np.pi/4, np.pi/4)

    def controller_cmd_vel_cb(self, msg):
        self.controller_cmd_vel = msg.linear.x, msg.angular.z

    def gripper_cmd_vel_cb(self, msg):
        self.gripper_cmd_vel = msg.linear.x, msg.angular.z

    def arucos_detected_cb(self, msg):
        self.detected_arucos = msg
        self.detected_aruco_ids = set([t.marker_id for t in msg.detections])

    def update(self):
        status = StateControllerStatus()
        if self.front_scan is None or self.pose is None:
            self.get_logger().warn(f"No scan {self.front_scan} or pose {self.pose}")
            return

        distance_to_goal = (
            np.hypot(
                self.target_pose[0] - self.pose[0], self.target_pose[1] - self.pose[1]
            )
            if self.target_pose is not None
            else 0
        )

        # State changes
        if (
            self.state == ControllerState.MAPPING
            and self.cube_id in self.detected_aruco_ids
        ):
            self.state = ControllerState.GRIP
            self.grip_succesful = False
        elif self.state == ControllerState.GRIP and self.grip_state.state == "GripperNodeState.GRIP":
            tag_pose = self.get_tag_pose(self.target_id)
            self.target_pose = tag_pose
            if tag_pose is None:
                self.get_logger().warn("Waiting for tag pose")
                return
            
            self.state = ControllerState.MOVE_TO_POINT
            self.next_state = ControllerState.RELEASE
        elif (
            self.state == ControllerState.MOVE_TO_POINT
            and distance_to_goal < self.distance_tol
        ):
            self.state = self.next_state
        elif self.state == ControllerState.RELEASE:
            self.state = ControllerState.BACKUP
        elif self.state == ControllerState.BACKUP and (self.clock.now() - self.turn_start_time).nanoseconds - self.k_turn_time * 1e9

        u, r = 0, 0
        if self.state == ControllerState.MAPPING:
            u, r = self.random_walk.update(self.pose, self.front_scan)
            status.random_walk_state = str(self.random_walk.state)
            #status.distance_to_goal = self.random_walk.turn_time_remaining
        elif self.state == ControllerState.GRIP:
            if self.last_state != self.state:
                self.get_logger().info("Calling gripper service")         
                self.req = Empty.Request()       
                self.gripper_srv.call_async(self.req)
                self.get_logger().info("Called gripper service")
            u, r = self.gripper_cmd_vel if self.gripper_cmd_vel else (0, 0)
            servo_cmd = Float32()
            servo_cmd.data = self.gripper_servo_cmd
            self.servo_pub.publish(servo_cmd)
        elif self.state == ControllerState.MOVE_TO_POINT:
            # TODO Change this over to bug0
            self.send_target_pose(self.get_tag_pose(self.target_id))            
            u, r = self.controller_cmd_vel if self.controller_cmd_vel else (0, 0)
            #status.distance_to_goal = distance_to_goal
        elif self.state == ControllerState.RELEASE:
            u, r = (0, 0)
            servo_cmd = Float32()
            servo_cmd.data = 110.0
            self.servo_pub.publish(servo_cmd)

        status.state = str(self.state)
        self.controller_state_pub.publish(status)

        cmd_vel = Twist()
        cmd_vel.linear.x, cmd_vel.angular.z = float(u), float(r)
        self.cmd_vel_pub.publish(cmd_vel)
        self.last_state = self.state

def main(args=None):
    rclpy.init(args=args)

    master_node = ControllerNode()
    rclpy.spin(master_node)

    master_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
