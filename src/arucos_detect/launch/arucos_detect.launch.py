
# Incluir Ros deep learning camera launch -------- LISTO -- por probar
# Incluir Camera Info ---------------------------- LISTO -- por probar
# Incluir Aruco node ----------------------------- LISTO -- por probar


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
            package = 'arucos_detect',
            executable = 'aruco_node.py',
            name = 'aruco_node'
        ),
        Node(
            package = 'arucos_detect',
            executable = 'camera_info.py',
            name = 'camera_info_node'
        ),
        IncludeLaunchDescription(  
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("foxglove_bridge"),
                    'launch',
                    'foxglove_bridge_launch.xml'
                    ])
            ]),
            launch_arguments={
                'serial_port': '/dev/lidar'
            }.items()
        ),
        IncludeLaunchDescription(  # launch camera
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ros_deep_learning"),
                    'launch',
                    'video_source.ros2.launch'
                    ])
            ]),
            launch_arguments={
                'input_width': '320',
                'input_height': '240',
                'framerate': '60',
                'flip': 'none'
            }.items()
        )

    ])