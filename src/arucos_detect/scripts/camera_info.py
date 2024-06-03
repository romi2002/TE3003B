#!/usr/bin/python3
# Ros2 Imports
import rclpy
from rclpy.node import Node

# Python imports
import numpy as np
import cv2
import yaml
import os

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory



# Calibration YAML extract CameraInfo
yaml_path = os.path.join(get_package_share_directory('arucos_detect'), "yaml", "ost.yaml")

def extract_camera_info(yaml_file):
    """Load camera info from a YAML file."""
    with open(yaml_file, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)  # Modified

    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.k = calib_data["camera_matrix"]["data"]
    camera_info_msg.r = calib_data["rectification_matrix"]["data"]
    camera_info_msg.p = calib_data["projection_matrix"]["data"]

    return camera_info_msg


class CalibrationPublisher(Node):

    def __init__(self):
        super().__init__("calibration_publisher")
        self.publisher_ = self.create_publisher(CameraInfo, "camera_info", 10)
        timer_period = 5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = extract_camera_info(yaml_path)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    calibration_publisher = CalibrationPublisher()
    rclpy.spin(calibration_publisher)
    calibration_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
