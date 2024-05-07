cmake_minimum_required(VERSION 3.8)
project(puzzlebot_rpc_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
message(STATUS "gRPC version: ${gRPC_VERSION}")
get_target_property(GRPC_CPP_LIBRARY gRPC::grpc++ LOCATION)
message(STATUS "gRPC C++ library location: ${GRPC_CPP_LIBRARY}")
get_target_property(GRPC_CPP_INCLUDE_DIRS gRPC::grpc++ INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "gRPC C++ include directories: ${GRPC_CPP_INCLUDE_DIRS}")

add_subdirectory(proto)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rpc_bridge src/rpc_bridge.cpp)
ament_target_dependencies(rpc_bridge rclcpp std_msgs)
target_link_libraries(rpc_bridge 
  puzzlebot_proto  
  gRPC::grpc++ 
  gRPC::grpc
  gRPC::grpc++_reflection
)

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp std_msgs)

install(TARGETS
  rpc_bridge
  test_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
