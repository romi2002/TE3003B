#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the path to the package share directory
    package_share_directory = get_package_share_directory('puzzlebot_ros')

    # Specify the full path to the world file
    world_file_path = os.path.join(package_share_directory, 'gazebo/world_empty.sdf')

    # Execute Gazebo simulation with the specified world file
    gazebo_sim = ExecuteProcess(cmd=['gz', 'sim', world_file_path], name='gazebo_sim', output='screen')
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/ControlR@std_msgs/msg/Float32]gz.msgs.Float',
                   '/ControlL@std_msgs/msg/Float32]gz.msgs.Float',
                   '/VelocityEncR@std_msgs/msg/Float32[gz.msgs.Float',
                   '/VelocityEncL@std_msgs/msg/Float32[gz.msgs.Float'
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
