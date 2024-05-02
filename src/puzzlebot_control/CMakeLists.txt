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

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs tf2 tf2_ros)

INSTALL(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
