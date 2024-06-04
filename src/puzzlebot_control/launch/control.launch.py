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
            package="puzzlebot_control",
            executable ="gripper_test.py",
            name="gripper_node",
            remappings = [
                ("/cmd_vel", "/gripper/cmd_vel"),
                ("/ServoAngle", "/gripper/servo_angle")
            ],
        ),
        Node(
          package="puzzlebot_control",
          executable="bug0_node.py",
          name="bug_node",
          remappings = [
              ("/cmd_vel", "/controller/cmd_vel")
          ]  
        ),
        Node(
            package="puzzlebot_control",
            executable="ekf_slam.py"
        )
    ])