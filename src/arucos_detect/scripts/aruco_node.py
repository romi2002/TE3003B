#!/usr/bin/python3
#Ppmike aniade a un array los arucos que se estan detectando a diferencia del Airlab donde se incluye en un solo mensaje todo
#Airlab hace uso de sus propias funciones de aruco pose estimation, ppmike usa opencv (se utilizar opencv)

#Ros2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.logging import get_logger
from cv_bridge import CvBridge
import message_filters

# Python imports
import numpy as np
import cv2
from cv_bridge import CvBridge
import argparse
import sys

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from arucos_interfaces.msg import ArucoMarkers, ArucosDetected
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Header


# Define command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-vs", "--video_source", type=str, default="/camera",
                help="Video source ROS topic")
ap.add_argument("-ci", "--camera_info", type=str, default="/camera_info",
                help="Camera info ROS topic")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL",
                help="Type of ArUCo tag to detect")

# Use parse_known_args to ignore unknown args
args, unknown = ap.parse_known_args()
args = vars(args)

# Verify and get the ArUco dictionary
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


logger = get_logger('aruco_detector')

if args["type"] not in ARUCO_DICT:
    logger.get_logger().error(f"ArUCo tag type '{args['type']}' is not supported")
    sys.exit(0)

#aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]]) #modificar
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
aruco_params = cv2.aruco.DetectorParameters_create()


class ArucoNode(Node):
    
    def __init__(self):
        super().__init__('aruco_node')
        #timer_period = 0.05 #seconds
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.marker_size = 0.1 #Marker Size in Meters

        #Setup Publisher
        self.aruco_publisher_ = self.create_publisher(ArucosDetected, 'arucos_detected', 10)

        #Setup Subscriber
        self.image = self.create_subscription(Image, args["video_source"], self.imageproc_cb,10)
        self.caminfo_sub = self.create_subscription(CameraInfo, args["camera_info"], self.caminfo_cb,10)

        #self.timer =  self.create_timer(timer_period, self.process_aruco)


    #Camera Info Callback
    def caminfo_cb(self, data):
        self.camera_matrix = np.array(data.k).reshape(3, 3)
        self.distortion_coefficients = np.array(data.d)

    #Image Processing Callback
    def imageproc_cb(self, data):
        opencv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        grayscale = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, self.rejected = cv2.aruco.detectMarkers(grayscale, aruco_dict, parameters=aruco_params)
        if self.ids is not None:
            self.process_aruco()
        else:
            print("NO ARUCO detected")
    
    # #Process Detected Aruco Markers
    # def process_aruco(self):
    #     detected_arucos = []
    #     rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, self.camera_matrix, self.distortion_coefficients)
    #     for i, corner in enumerate(self.corners):
    #         center = corner.reshape((4,2)).mean(axis=0)
    #         pose = [tvecs[i][0][0],tvecs[i][0][1],tvecs[i][0][2]] #position in 3D space
    #         detected_arucos.append(ArucosDetected(id=int(self.ids[i][0]), pose=pose))
    #     self.aruco_publisher_.publish(ArucosDetected(arucos_detected=detected_arucos))
    #     self.get_logger().info('Publishing ArucosDetected')
    
        #Process Detected Aruco Markers
    def process_aruco(self):
        detected_arucos = ArucosDetected()
        detected_arucos.header = Header()
        detected_arucos.header.stamp = self.get_clock().now().to_msg()
        detected_arucos.header.frame_id = "camera"

        for i, corner in enumerate(self.corners):
            marker = ArucoMarkers()
            marker.header = detected_arucos.header
            marker.marker_ids = int(self.ids[i][0])
            pose = Pose()
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, self.camera_matrix, self.distortion_coefficients)
            pose.position.x = tvecs[0][0][0]
            pose.position.y = tvecs[0][0][1]
            pose.position.z = tvecs[0][0][2]
            # orientation calculation?
            marker.poses.append(pose)
            detected_arucos.arucos_detected.append(marker)

        print(marker.marker_ids)
        self.aruco_publisher_.publish(detected_arucos)
        self.get_logger().info('Publishing ArucosDetected')

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
