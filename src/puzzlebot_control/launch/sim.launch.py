from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    urdf_tutorial_path = FindPackageShare('puzzlebot_control')
    default_model_path = PathJoinSubstitution(['urdf', 'puzzlebot.urdf'])
    default_rviz_config_path = PathJoinSubstitution([urdf_tutorial_path, 'rviz', 'urdf.rviz'])

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'puzzlebot_control',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    # Agregar nodos para los paquetes robot_state_publisher, puzzlebot_control, y tf2_ros
    ld.add_action(Node(
        package='puzzlebot_control',
        executable='kinematic_node',
        name='kinematic_node',
        output='screen'
    ))

    ld.add_action(Node(
        package='puzzlebot_control',
        executable='odometry_node.py',
        name='odometry_node',
        output='screen'
    ))

    ld.add_action(Node(
        package='puzzlebot_control',
        executable='joint_state_node',
        name='joint_state_pub_node',
        output='screen'
    ))

    # ld.add_action(Node(
    #     package='puzzlebot_control',
    #     executable='controller_node',
    #     name='controller_node',
    #     output='screen'
    # ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    ))

    return ld