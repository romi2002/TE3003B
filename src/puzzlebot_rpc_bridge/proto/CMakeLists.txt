project(puzzlebot_proto)
set (CMAKE_CXX_STANDARD 14)

set (Protobuf_DEBUG ON)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
set(CMAKE_VERBOSE_MAKEFILE on)


set(PROTO_FILES
    RobotState.proto
)

file(GLOB_RECURSE GOOGLE_PROTO_FILES
    google/api/annotations.proto
    google/api/http.proto
)

add_library(puzzlebot_proto ${PROTO_FILES} ${GOOGLE_PROTO_FILES})
target_link_libraries(puzzlebot_proto
    PUBLIC
        protobuf::libprotobuf
        gRPC::grpc
        gRPC::grpc++
)
target_include_directories(puzzlebot_proto PUBLIC 
    ${CMAKE_CURRENT_BINARY_DIR}
)

# set(PROTO_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
# target_include_directories(puzzlebot_proto PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")

# Compile protobufs and grpc files
get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)
protobuf_generate(
    TARGET puzzlebot_proto
    LANGUAGE cpp
)

protobuf_generate(
    TARGET puzzlebot_proto
    LANGUAGE grpc 
    GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc 
    PLUGIN "protoc-gen-grpc=${grpc_cpp_plugin_location}"
)
