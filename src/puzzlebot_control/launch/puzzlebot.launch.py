from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "micro_ros_agent",
            executable = "micro_ros_agent",
            name = 'micro_ros_agent',
            arguments = ["serial", "-D", "/dev/hackerboard"],
            parameters=[],
            output = 'screen'
        ),
        Node(
            package = "puzzlebot_control",
            executable = "cmd_vel_node.py",
            name = "cmd_vel_node"
        ),
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            output = 'screen'
        ),
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "laser"],
            output = 'screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("sllidar_ros2"),
                    'launch',
                    'sllidar_a1_launch.py'
                    ])
            ]),
            launch_arguments={
                'serial_port': '/dev/lidar'
            }.items()
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("foxglove_bridge"),
                    'launch',
                    'foxglove_bridge_launch.xml'
                    ])
            ])
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ros_deep_learning"),
                    'launch',
                    'video_source.ros2.launch'
                    ])
            ]),
            launch_arguments={
                'width': '1280',
                'height': '720',
                'flip': 'none'
            }.items()
        ),
        Node(
            package = "arucos_detect",
            executable = "camera_info.py",
        ),
    ])