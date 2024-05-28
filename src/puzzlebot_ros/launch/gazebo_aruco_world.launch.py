#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory("puzzlebot_ros"),
        "gazebo",
        "world_aruco_cv.sdf"
    )
    resource_path = os.path.join(
        get_package_share_directory("puzzlebot_ros"),
        "gazebo",
    )
    plugin_path = os.path.join(
        get_package_share_directory("puzzlebot_ros"),
        "gazebo",
        "plugins"
    )
    gazebo_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path],
        name='gazebo_sim', 
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': resource_path,
                        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': plugin_path})
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/model/puzzlebot1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/lidar@sensor_msgs/msg/LaserScan]gz.msgs.LaserScan',
                   '/VelocitySetR@std_msgs/msg/Float32]gz.msgs.Float',
                   '/VelocitySetL@std_msgs/msg/Float32]gz.msgs.Float',
                   '/VelocityEncR@std_msgs/msg/Float32[gz.msgs.Float',
                   '/VelocityEncL@std_msgs/msg/Float32[gz.msgs.Float',
                   '/robot_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist'
                  ],
        output='screen'
    )
    
    # Launch a static transform publisher node
    static_transform_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=["0", "0", "0",    "0", "0", "0",    "base", "puzzlebot1/chassis/camera_sensor",]
    )
    
    ld = [gazebo_sim, bridge, static_transform_camera]

    return LaunchDescription(ld)
