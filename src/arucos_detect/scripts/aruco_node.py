#!/usr/bin/python3
# Ppmike aniade a un array los arucos que se estan detectando a diferencia del Airlab donde se incluye en un solo mensaje todo
# Airlab hace uso de sus propias funciones de aruco pose estimation, ppmike usa opencv (se utilizar opencv)

# Ros2 Imports
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
from tf_transformations import quaternion_from_matrix

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, Point
from arucos_interfaces.msg import ArucoMarkers, ArucosDetected
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Header



# Define command line arguments
ap = argparse.ArgumentParser()
ap.add_argument(
    "-vs",
    "--video_source",
    type=str,
    default="/video_source/raw",
    help="Video source ROS topic",
)
ap.add_argument(
    "-ci",
    "--camera_info",
    type=str,
    default="/camera_info",
    help="Camera info ROS topic",
)
ap.add_argument(
    "-t",
    "--type",
    type=str,
    default="DICT_5X5_50",
    help="Type of ArUCo tag to detect",
)

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
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


logger = get_logger("aruco_detector")

if args["type"] not in ARUCO_DICT:
    logger.get_logger().error(f"ArUCo tag type '{args['type']}' is not supported")
    sys.exit(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
# aruco_params = cv2.aruco.DetectorParameters_create()
aruco_params = cv2.aruco.DetectorParameters()

# Custom marker_points depending on the aruco ID
def marker_points(aruco_size):
    array = np.array(
    [
        [-aruco_size / 2, aruco_size / 2, 0],
        [aruco_size / 2, aruco_size / 2, 0],
        [aruco_size / 2, -aruco_size / 2, 0],
        [-aruco_size / 2, -aruco_size / 2, 0],
    ],
    dtype=np.float32,
    )
    return array




class ArucoNode(Node):

    def __init__(self):
        super().__init__("aruco_node")

        # timer_period = 0.05 #seconds
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.wall_marker_size = 0.1  # Marker Size in Meters
        self.cube_marker_size = 0.05

        # Setup Publisher
        # self.aruco_publisher_ = self.create_publisher(
        #     ArucosDetected, "arucos_detected", 10
        # )
        self.aruco_marker_publisher_ = self.create_publisher(
            ArucoMarkers, "aruco_marker", 10
        )

        # Setup Subscriber
        self.image = self.create_subscription(
            Image, args["video_source"], self.imageproc_cb, 10
        )
        self.caminfo_sub = self.create_subscription(
            CameraInfo, args["camera_info"], self.caminfo_cb, 10
        )

    # Camera Info Callback
    def caminfo_cb(self, data):
        self.camera_matrix = np.array(data.k).reshape(3, 3)
        self.distortion_coefficients = np.array(data.d)

    # Image Processing Callback
    def imageproc_cb(self, data):
        opencv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        grayscale = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, self.rejected = cv2.aruco.detectMarkers(  # Corners es un array de de esquinas o sea cada elemento tiene 4
            grayscale,
            aruco_dict,
            parameters=aruco_params,
        )
        print(self.ids)
        if self.ids is not None:
            self.process_aruco()
        else:
            print("NO ARUCO detected")


    # Process Detected Aruco Markers
    def process_aruco(self):
        if self.camera_matrix is None or self.distortion_coefficients is None:
            self.get_logger().error("No camera matrix")
            return
        
        detected_arucos = ArucosDetected()
        detected_arucos.header = Header()
        detected_arucos.header.stamp = self.get_clock().now().to_msg()
        detected_arucos.header.frame_id = "camera"



        # Del array de multiples esquinas se hace iteracion de cada conjunto (4)
        for i, corner in enumerate(self.corners):  # creo que estoy haciendo un solo aruco deberia de modificar el codigo para que sean todos?
            
            marker = ArucoMarkers()
            marker.header = detected_arucos.header
            marker.marker_id = int(self.ids[i][0])

            
            if marker.marker_id == 6:
                _, rvec, tvec = cv2.solvePnP(
                marker_points(self.cube_marker_size),
                corner,
                self.camera_matrix,
                self.distortion_coefficients,
                False,
                cv2.SOLVEPNP_IPPE_SQUARE,
                )
            else:
                _, rvec, tvec = cv2.solvePnP(
                marker_points(self.wall_marker_size),
                corner,
                self.camera_matrix,
                self.distortion_coefficients,
                False,
                cv2.SOLVEPNP_IPPE_SQUARE,
                )

            # Calcular centroide del aruco
            centroid = Point()
            moments = cv2.moments(corner[0])            
            centroid.x = float(moments['m10'] / moments['m00'])
            centroid.y = float(moments['m01'] / moments['m00'])
            centroid.z = 0.0 # Imagen es en 2D
            marker.centroid = centroid

            # print(f"rvec {rvec} tvec {tvec} {tvec.shape}")
            marker.pose.position.x = tvec[0][0]
            marker.pose.position.y = tvec[1][0]
            marker.pose.position.z = tvec[2][0]

            # Rotation vector to rotation matrix
            rotation_matrix = np.eye(4)
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec))[0] ## checar[][]??

            quaternion = quaternion_from_matrix(rotation_matrix)

            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]


            detected_arucos.detections.append(marker)
            self.aruco_marker_publisher_.publish(marker)
            # self.get_logger().info("Publishing ArucoMarkers")
            # cv2.aruco.drawAxis(grayscale, self.mtx, self.dst, rvec[i], tvec[i], 0.05)  # checar rvec[i]?

        #self.aruco_publisher_.publish(detected_arucos)
        self.get_logger().info("Publishing ArucosDetected")


def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()