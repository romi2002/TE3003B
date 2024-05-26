cmake_minimum_required(VERSION 3.8)
project(puzzlebot_rpc_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Protobuf REQUIRED)

include(FetchContent)
FetchContent_Declare(
  gRPC
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG        v1.40.0
)
set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailable(gRPC)

add_subdirectory(proto)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(JPEG REQUIRED)

add_executable(rpc_bridge src/rpc_bridge.cpp)
ament_target_dependencies(rpc_bridge rclcpp std_msgs)
target_link_libraries(rpc_bridge 
  puzzlebot_proto  
  Boost::boost
  ${JPEG_LIBRARIES}
  grpc++
)
target_include_directories(rpc_bridge PRIVATE ${Boost_INCLUDE_DIRS})

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp std_msgs)

install(TARGETS
  rpc_bridge
  test_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
