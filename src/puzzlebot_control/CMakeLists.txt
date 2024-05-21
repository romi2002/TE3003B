cmake_minimum_required(VERSION 3.8)
project(puzzlebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

add_executable(controller_node src/controller_node.cpp)
add_executable(kinematic_node src/kinematic_sim.cpp)
add_executable(joint_state_node src/joint_state_publisher_node.cpp)
add_executable(potential_field_node src/potential_field_node.cpp)
add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(kinematic_node rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs)
ament_target_dependencies(joint_state_node rclcpp geometry_msgs sensor_msgs tf2_geometry_msgs std_msgs)
ament_target_dependencies(potential_field_node rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs sensor_msgs)
ament_target_dependencies(odometry_node rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs sensor_msgs nav_msgs)

INSTALL(TARGETS
  controller_node
  potential_field_node
  kinematic_node
  odometry_node
  joint_state_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/cmd_vel_node.py
  src/odometry_node.py
  scripts/set_point.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  models
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
