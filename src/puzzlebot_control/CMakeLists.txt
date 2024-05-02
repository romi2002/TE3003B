cmake_minimum_required(VERSION 3.8)
project(puzzlebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(potential_field_node src/potential_field_node.cpp)
ament_target_dependencies(potential_field_node rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs sensor_msgs)

INSTALL(TARGETS
  controller_node
  potential_field_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
