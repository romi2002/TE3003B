import signal
import threading
from concurrent import futures
import cv2
import base64
import numpy as np

import grpc
import sys
sys.path.insert(1, './proto')

import RobotState_pb2
import RobotState_pb2_grpc

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import math
import base64

def rad2deg(x):
    return x * 180.0 / math.pi

class RobotStateImpl(RobotState_pb2_grpc.RobotStateServicer, Node):
    def __init__(self):
        Node.__init__(self, "robot_state_subscriber")
        # ROS2 Subscribers
        self.image_sub = self.create_subscription(
            Image,
            "image_in",
            self.image_cb,
            10
        )
        self.sub_encR = self.create_subscription(Float32,'VelocityEncR',self.encR_callback,qos.qos_profile_sensor_data) # Right wheel encoder velocity
        self.sub_encL = self.create_subscription(Float32,'VelocityEncL',self.encL_callback,qos.qos_profile_sensor_data) # Left wheel encoder velocity
        
        self.lidar_info_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_cb,
            10
        )

        print("Initialized gRPC Server")
        # Variables
        self.img_compressed = None # Compressed image
        self.shape = None # Image shape
        self.bridge = CvBridge() # ROS-CV bridge
        self.img_b64 = None # Base64 encoded image
        self.velocityR = None # Right wheel encoder velocity
        self.velocityL = None # Left wheel encoder velocity
        self.scan = None # Lidar scan

    def image_cb(self, data):
        img_original = self.bridge.imgmsg_to_cv2(data)
        self.shape = img_original.shape
        self.img_compressed = np.array(cv2.imencode('.jpg', img_original)[1]).tobytes()
        self.img_b64 = base64.b64encode(self.img_compressed)

    def GetImage(self, request, context):
        print("GetImage Got call: " + context.peer())

        if self.img_b64 is None or self.shape is None:
            context.set_code(grpc.StatusCode.UNAVAILABLE)
            context.set_details("No image has been received.")

        results = RobotState_pb2.ImageReply()
        results.img_b64 = self.img_b64
        results.width = self.shape[1]
        results.height = self.shape[0]
        return results

    def encR_callback(self, msg):
        self.velocityR = msg.data
        
    def encL_callback(self, msg):
        self.velocityL = msg.data

    def GetVelocity(self, request, context):
        print("GetVelocity Got call: " + context.peer())

        if self.velocityL is None or self.velocityR is None:
            context.set_code(grpc.StatusCode.UNAVAILABLE)
            context.set_details("No velocity has been received.")

        results_vel = RobotState_pb2.VelocityReply()
        results_vel.encL = self.velocityL
        results_vel.encR = self.velocityR
        return results_vel

    def scan_cb(self, scan):
        self.scan = scan

    def GetLidar(self, request, context):
        print("GetLidar Got call: " + context.peer())

        if self.scan is None:
            context.set_code(grpc.StatusCode.UNAVAILABLE)
            context.set_details("No LiDAR info has been received.")

        results_lidar = RobotState_pb2.LidarReply()
        results_lidar.angle_min = self.scan.angle_min
        results_lidar.angle_max = self.scan.angle_max
        results_lidar.range.extend(self.scan.ranges)
        results_lidar.range_max = self.scan.range_max
        return results_lidar


terminate = threading.Event()
def terminate_server(signum, frame):
    print("Got signal {}, {}".format(signum, frame))
    rclpy.shutdown()
    terminate.set()

if __name__ == '__main__':
    print("-------ROS to gRPC Wrapper--------")
    signal.signal(signal.SIGINT, terminate_server)

    print("Starting ROS node")
    rclpy.init()

    print("Starting gRPC Server")
    server_addr = "[::]:7042"
    service = RobotStateImpl()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    RobotState_pb2_grpc.add_RobotStateServicer_to_server(service, server)
    server.add_insecure_port(server_addr)
    server.start()
    print("gRPC Server listening on " + server_addr)

    print("Running ROS node")
    rclpy.spin(service)

    terminate.wait()
    print("Stopping gRPC Server")
    server.stop(1).wait()

    service.destroy_node()
    rclpy.shutdown()
    print("Exited")
