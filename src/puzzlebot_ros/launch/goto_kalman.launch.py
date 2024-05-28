#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
           
    pkg_puzzlebot_ros = get_package_share_directory('puzzlebot_ros')
    
    # GoTo
    go_to = Node(
        package='puzzlebot_ros',
        executable='goto_point',
        name='go_to_node',
        arguments=[],
        output='screen'
    )
    
    kalman = Node(
        package='puzzlebot_ros',
        executable='kalman',
        name='kalman_node',
        arguments=[],
        output='screen'
    )
            
    ld = [kalman, go_to]

    return LaunchDescription(ld)
