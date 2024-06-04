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
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge
import argparse
import sys

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, Point
from arucos_interfaces.msg import ArucoMarkers, ArucosDetected
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Header, Float64


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
    default="DICT_5X5_100",
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

# aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]]) #modificar
aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
aruco_params = cv2.aruco.DetectorParameters()


class ArucoNode(Node):

    def __init__(self):
        super().__init__("aruco_node")

        # timer_period = 0.05 #seconds
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.marker_size = 0.1  # Marker Size in Meters
        
        # TODO Move this to a parameter
        # In degrees
        self.camera_rpy = np.array((-90, 0, -90))
        r = R.from_euler('xyz', self.camera_rpy, degrees=True)
        self.camera_xyz = np.array((0, 0, 0))

        self.camera_transform_matrix = np.zeros((4, 4))
        # First 3x3 is rotation matrix
        self.camera_transform_matrix[0:3, 0:3] = r.as_matrix()
        self.camera_transform_matrix[0, 3] = self.camera_xyz[0]
        self.camera_transform_matrix[1, 3] = self.camera_xyz[1]
        self.camera_transform_matrix[2, 3] = self.camera_xyz[2]
        self.camera_transform_matrix[3, 3] = 1
        self.get_logger().info(f"Camera Transform Matrix:\n{self.camera_transform_matrix}")

        # Setup Publisher
        self.aruco_publisher_ = self.create_publisher(
            ArucosDetected, "arucos_detected", 10
        )

        # Setup Subscriber
        self.image = self.create_subscription(
            Image, args["video_source"], self.imageproc_cb, 10
        )
        self.caminfo_sub = self.create_subscription(
            CameraInfo, args["camera_info"], self.caminfo_cb, 10
        )

        # self.timer =  self.create_timer(timer_period, self.process_aruco)

    # Camera Info Callback
    def caminfo_cb(self, data):
        self.camera_matrix = np.array(data.k).reshape(3, 3)
        self.distortion_coefficients = np.array(data.d)

    # Image Processing Callback
    def imageproc_cb(self, data):
        opencv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        grayscale = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, self.rejected = cv2.aruco.detectMarkers(
            grayscale, aruco_dict, parameters=aruco_params
        )
        self.process_aruco()

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

    # Process Detected Aruco Markers
    def process_aruco(self):
        if self.camera_matrix is None or self.distortion_coefficients is None:
            self.get_logger().error("No camera matrix")
            return
        
        detected_arucos = ArucosDetected()
        detected_arucos.header = Header()
        detected_arucos.header.stamp = self.get_clock().now().to_msg()
        detected_arucos.header.frame_id = "camera"

        marker_points = np.array(
            [
                [-self.marker_size / 2, self.marker_size / 2, 0],
                [self.marker_size / 2, self.marker_size / 2, 0],
                [self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0],
            ],
            dtype=np.float32,
        )

        for i, corner in enumerate(self.corners):
            marker = ArucoMarkers()
            marker.header = detected_arucos.header
            marker.marker_id = int(self.ids[i][0])

            _, rvec, tvec = cv2.solvePnP(
                marker_points,
                corner,
                self.camera_matrix,
                self.distortion_coefficients,
                False,
                cv2.SOLVEPNP_IPPE_SQUARE,
            )
            #print(f"rvec {rvec} tvec {tvec} {tvec.shape}")
            rot, _ = cv2.Rodrigues(rvec)
            marker_matrix = np.zeros((4, 4))
            # First 3x3 is rotation matrix
            marker_matrix[0:3, 0:3] = rot
            marker_matrix[0, 3] = tvec[0][0]
            marker_matrix[1, 3] = tvec[1][0]
            marker_matrix[2, 3] = tvec[2][0]
            marker_matrix[3, 3] = 1
            
            transformed_marker = self.camera_transform_matrix @ marker_matrix
            #print(f"{self.camera_transform_matrix}")

            x = transformed_marker[0, 3]
            y = transformed_marker[1, 3]
            z = transformed_marker[2, 3]
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            
            marker.range.data = np.hypot(x, y)
            marker.bearing.data = np.arctan2(y, x)
            detected_arucos.detections.append(marker)

            marker.marker_id = int(self.ids[i][0])

            _, rvec, tvec = cv2.solvePnP(
                marker_points,
                corner,
                self.camera_matrix,
                self.distortion_coefficients,
                False,
                cv2.SOLVEPNP_IPPE_SQUARE,
            )
            #print(f"rvec {rvec} tvec {tvec} {tvec.shape}")
            rot, _ = cv2.Rodrigues(rvec)
            marker_matrix = np.zeros((4, 4))
            # First 3x3 is rotation matrix
            marker_matrix[0:3, 0:3] = rot
            marker_matrix[0, 3] = tvec[0][0]
            marker_matrix[1, 3] = tvec[1][0]
            marker_matrix[2, 3] = tvec[2][0]
            marker_matrix[3, 3] = 1
            
            transformed_marker = self.camera_transform_matrix @ marker_matrix
            print(f"{self.camera_transform_matrix}")

            x = transformed_marker[0, 3]
            y = transformed_marker[1, 3]
            z = transformed_marker[2, 3]
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            
            marker.range.data = np.hypot(x, y)
            marker.bearing.data = np.arctan2(y, x)

            # Calcular centroide del aruco
            centroid = Point()
            moments = cv2.moments(corner[0])            
            centroid.x = float(moments['m10'] / moments['m00'])
            centroid.y = float(moments['m01'] / moments['m00'])
            centroid.z = 0.0 # Imagen es en 2D
            marker.centroid = centroid
            marker.area.data = moments['m00']

            detected_arucos.detections.append(marker)

        self.aruco_publisher_.publish(detected_arucos)


def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
