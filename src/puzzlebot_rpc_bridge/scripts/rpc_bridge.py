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
from sensor_msgs.msg import Image

import base64

class RobotStateImpl(RobotState_pb2_grpc.RobotStateServicer, Node):
    def __init__(self):
        Node.__init__(self, "robot_state_subscriber")
        self.image_sub = self.create_subscription(
            Image,
            "image_in",
            self.image_cb,
            10
        )
        print("Initialized gRPC Server")
        self.img_compressed = None
        self.shape = None
        self.bridge = CvBridge()
        self.img_b64 = None

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
