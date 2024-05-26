project(puzzlebot_proto)
set (CMAKE_CXX_STANDARD 14)

set (Protobuf_DEBUG ON)
find_package(Protobuf REQUIRED)

include(FetchContent)
FetchContent_Declare(
  gRPC
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG        v1.40.0
)
set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailable(gRPC)

set(PROTO_FILES
    Test.proto
)

add_library(puzzlebot_proto ${PROTO_FILES})
target_link_libraries(puzzlebot_proto
    PUBLIC
        protobuf::libprotobuf
        grpc++
)
target_include_directories(puzzlebot_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# set(PROTO_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
# target_include_directories(puzzlebot_proto PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")

# Compile protobufs and grpc files
#get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)
protobuf_generate(TARGET puzzlebot_proto LANGUAGE cpp)
#protobuf_generate(TARGET puzzlebot_proto LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc PLUGIN "protoc-gen-grpc=${grpc_cpp_plugin_location}")
protobuf_generate(TARGET puzzlebot_proto LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc)
